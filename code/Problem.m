classdef Problem<handle
% A basic problem class for mesh optimization problem with constraints
% on singular values. Implements the code required for running a single
% iteration of Algorithm 2.
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Controlling Singular Values with Semidefinite Programming".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Shahar Kovalsky (http://www.wisdom.weizmann.ac.il/~shaharko/)
%        and Noam Aigerman
%        (http://www.wisdom.weizmann.ac.il/~noamaig/)
%
% Further edited by Angjoo Kanazawa to jointly learn the bounds on
% each face.
% This version allows taking Y for more than 1 shape.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    properties
        X; % input vertices
        tris; % triangulation
        Y; % output vertices (yalmip optimization variables)
        INIT_Y; % initialization for Y
        vols; % vector of element volumes
        distortion=nan; % vector of element distortions
        flipped=nan; % vector of element flips
        faces; % class array of elements (faces)
        objType; % objective for optimization (see ObjectiveEnum)
        spaceType; % objective for optimization (see SpaceEnum)
        C; % Bounds on each face (if global 1x1)
        target_dim; % source mesh dimension
        objVal=nan; % objective value
        objective; % objective yalmip optimization variable)
        objectiveConstraints; % objective related constraints (in epigraph form)
        recalc_objective=true;
        yalmip_prm; % yalmip's parameters
        log; % log
        % AJ:
        recalc_softConstraints = true;
        scale_softConstraints = 1000;
        user_objective;
        
        Projection;
        num_image;
        num_face;
        learn_C;
        base_C = 1.01; % bias when learning
        constSpaceConstraints = [];
        c_objective = []; % L1 norm on stiffness
        c_constraint = [];
        s_lambda = 1; % Weight for stiffness.
        o_lambda = 1; % Weight for objective.
    end
    
    methods
        function obj=Problem(X,tris,C,objType,spaceType,Y,PJ,weights)
            stopper=Stopper('Constructing problem... ');
            obj.faces=Face();
            obj.vols=compute_volumes(tris,X);
            obj.X=X;
            obj.INIT_Y=X;
            obj.tris=tris;
            obj.C=C(:);
            obj.Projection = PJ;
            if isfield(weights, 'stiffness')
                obj.s_lambda = weights.stiffness;
            end
            obj.o_lambda = weights.objective;
            % Check dimension
            if length(obj.C) > 1 && length(obj.C) ~= size(tris,1)
                warning(sprintf(['BAD! # of C (%d) has to equal the # of faces (%d)\' ...
                                 'n'], size(obj.C,2), size(tris,1)));
                keyboard
            end
            % If C is a sdpvar, we need to learn it.
            obj.learn_C = ~isempty(obj.C) && isa(obj.C(1), 'sdpvar');
            
            obj.objType=objType;
            obj.spaceType=spaceType;

            if iscell(Y)
                obj.num_image = length(Y);
                % Repeat for each image.
                % obj.C = repmat(obj.C(:), [1, obj.num_image]);
            else
                obj.num_image = 1;
            end

            obj.Y = Y;
            obj.target_dim=size(X,2);
            obj.num_face = size(tris, 1);
            obj.faces(obj.num_face, obj.num_image)=Face(); % crappy preallocation - fix somehow
            k = sdpvar(obj.num_face, obj.num_image, 'full'); % define here for speedup
            K = sdpvar(obj.num_face, obj.num_image, 'full'); % define here for speedup
            % calculate the differentials
            for i = 1:size(tris,1),
                tri=tris(i,:);
                s_tri=X(tri,:)';
                B = eye(size(s_tri,2)) - 1/size(s_tri,2);
                P = B/(s_tri*B);
                P(abs(P)<=1e2*eps) = 0;
                for j = 1:obj.num_image
                    obj.faces(i, j).A = obj.Y{j}(tri,:)'*P;
                    obj.faces(i, j).K=K(i, j);
                    obj.faces(i, j).k=k(i, j);
                end
            end
            obj.log.t_Problem=stopper.stop;

            obj.setYalmipParams();

            %% Generate constraints that has to be defined once.
            obj.computeConstantSpaceConstraints();
            obj.computeStiffnessObjective();
        end
        
        function setYalmipParams(obj)
            obj.yalmip_prm = sdpsettings;
            obj.yalmip_prm.solver = '+mosek';
            obj.yalmip_prm.verbose = 0;
            obj.yalmip_prm.quadprog.Algorithm = 'interior-point-convex';
            obj.yalmip_prm.cachesolvers = 1;
            % debug
            % obj.yalmip_prm.verbose = 2;
            % obj.yalmip_prm.debug = 1;
            % obj.yalmip_prm.showprogress = 1;
        end
        
        
        function computeConstantSpaceConstraints(obj)
            if obj.spaceType==SpaceEnum.NONE
                return;
            end
            stopper=Stopper('Generating one-time space constraints (%s)... ', char(obj.spaceType));
            if ~isa(obj.faces(1).K,'double') % do not try to constrain if K are doubles
                Cmat = repmat(obj.C, [1 obj.num_image]);
                assert(all(size(obj.faces) == size(Cmat)));
                if obj.learn_C
                    Cmat = obj.base_C + Cmat(:)';
                else
                    Cmat = Cmat(:)';
                end
                switch obj.spaceType
                  case SpaceEnum.BI
                    if obj.learn_C
                        % As SOCP:
                        inside = [[obj.faces.k] + Cmat; 2*ones(size(Cmat)); ...
                                  [obj.faces.k] - Cmat];
                        c = cone(inside) + ([obj.faces.K] <= Cmat);
                    else
                        c = (([obj.faces.k] >= 1./Cmat) + ...
                             ([obj.faces.K] <= Cmat));
                    end
                  otherwise
                    error('constraint type not properly defined in code');
                end
            else
                warning('k,K seem to be constants -> k,K constraints are not generated');
            end
            obj.constSpaceConstraints = c;
            obj.log.t_constSpaceConstraints=stopper.stop;
        end
        
        function c = generateSpaceConstraints(obj)
            stopper=Stopper('Generating space constraints (%s)... ', char(obj.spaceType));
            c=cell(numel(obj.faces) + 1,1);
            % case of no constraints
            if obj.spaceType==SpaceEnum.NONE
                return;
            end
            % generate per-face constraints
            % TODO: Consider parfor.
            for i=1:numel(obj.faces)
                face=obj.faces(i);
                A=face.A;
                F=face.F;
                K=face.K;
                k=face.k;

                FA = F'*A;
                % bound maximal singular value (eq. (5) in the paper)
                constraint_upper = ([K*eye(size(A)) A; A' K*eye(size(A))]>=0);
                % bound the minimal singular value (eq. (6) and (11.d) in the paper)
                constraint_lower = ((FA+FA')/2 - k*eye(size(A,1))>=0);
                c{i}=constraint_upper+constraint_lower;
            end
            % Add constaint space constraints that were defined in constructor.
            c{end} = obj.constSpaceConstraints;
            % finish
            fprintf(' %d face constraints generated... ',nnz(~cellfun(@isempty,c)));
            obj.log.t_generateObjective=stopper.stop;
        end
        
        function [o, o_c]=generateObjective(obj)
            stopper=Stopper('Generating objective (%s)... ', char(obj.objType));
            if obj.recalc_objective 
                obj.objective = [];
                obj.objectiveConstraints = [];
                switch obj.objType
                    case ObjectiveEnum.AAAP 
                        % get adjacent faces
                        TR = triangulation(obj.tris,obj.X);
                        N = TR.neighbors;
                        I = repmat((1:size(N,1))',[1 4]);
                        N = N(:);
                        I = I(:);
                        ff = isnan(N) | (I>N);
                        N(ff) = [];
                        I(ff) = [];
                        % calc functional
                        vols_sum = obj.vols(N)+obj.vols(I);
                        w = sqrt(vols_sum/sum(vols_sum));
                        W = bsxfun(@times,ones(obj.target_dim),permute(w,[3 ...
                                            2 1]));
                        t_smooth = sdpvar();
                        z = cell(obj.num_image, 1);
                        for i = 1:obj.num_image
                            A_here = reshape([obj.faces(:, i).A], ...
                                             obj.target_dim, obj.target_dim,[]);
                            Adiff_here = A_here(:, :, I) - A_here(:, :, N);
                            z{i} = W(:).*Adiff_here(:);
                        end
                        obj.objective = t_smooth;
                        obj.objectiveConstraints = cone(cat(1, z{:}), ...
                                                        t_smooth);
                        % need only be calculated once
                        obj.recalc_objective = false;
                    case ObjectiveEnum.ARAP 
                        A = reshape([obj.faces.A],obj.target_dim,obj.target_dim,[]);
                        F = reshape([obj.faces.F],obj.target_dim,obj.target_dim,[]);
                        w = sqrt(obj.vols/sum(obj.vols));
                        W = bsxfun(@times,ones(size(A(:,:,1))),permute(w,[3 ...
                                            2 1]));
                        W = repmat(W, [1 1 obj.num_image]);
                        z = W(:).*(A(:)-F(:));
                        t = sdpvar;
                        obj.objective = t;
                        obj.objectiveConstraints = cone(z,t);
                    otherwise
                        error('no such functional');
                end
            else
                fprintf('SKIPPING ');
            end
            o=obj.objective;
            o_c=obj.objectiveConstraints;
            obj.log.t_generateSpaceConstraints=stopper.stop;
        end
        function updateDistortion(obj)
            Aval=reshape(double(cat(2,obj.faces.A)), obj.target_dim, ...
                         obj.target_dim,[]);
            switch obj.spaceType
                case {SpaceEnum.NONE}
                    for i = 1:numel(obj.faces),
                        A = Aval(:,:,i);
                        obj.distortion(i) = cond(A);
                        obj.flipped(i) = det(A)<0;
                    end
                case {SpaceEnum.BI}
                    for i = 1:numel(obj.faces),
                        A = Aval(:,:,i);
                        s = svd(A);
                        obj.distortion(i) = max(s(1),1/s(end));
                        obj.flipped(i) = det(A)<0;
                    end
                otherwise
                        error('Distortion is not defined');
            end
        end
        function updateFrames(obj)
            stopper=Stopper('Updating frames... ');
            for i = 1:obj.num_image
                Aval = reshape(value([obj.faces(:, i).A]), ...
                               obj.target_dim, obj.target_dim, []);
                for j = 1:obj.num_face
                    A = Aval(:, :, j);
                    [U, ~, V] = closest_rotation(A);
                    obj.faces(j, i).F = U*V';
                end
                
            end
            obj.log.t_updateFrames=stopper.stop;
                        
        end
        function c=gatherConstraints(obj,o_c,s_c,u_c,stiff_c)
            stopper=Stopper('Gathering constraints... ');
            c=catRecursive(s_c{:})+o_c+u_c+stiff_c;
            obj.log.t_gatherConstraints=stopper.stop;
        end;
        
        function runSolver(obj,c,o)
            stopper=Stopper('Calling solvesdp... ');
            res = solvesdp(c,o,obj.yalmip_prm);
            if(res.problem)
                warning(yalmiperror(res.problem));
            end
            obj.objVal=double(o);

            obj.log.solverStatus=res.problem;
            obj.log.t_runSolver=stopper.stop;
            obj.log.t_solver=res.solvertime;
            fprintf('Solver time: (%.2g secs)\n', res.solvertime);
            if obj.learn_C
                frac0 = [value(obj.objective), value(obj.user_objective), value(obj.c_objective)];
                frac = (abs(frac0)./sum(abs(frac0))) * 100;
                fprintf(['\nObjective pct: deform obj %.4g user obj ' ...
                         '%.4g stiffness sparsity %.4g (%g %g %g])\n'], frac, frac0);
            else                
                frac0 = [value(obj.objective) value(obj.user_objective)];
                frac = (abs(frac0)./sum(abs(frac0))) * 100;
                fprintf(['\nObjective pct: deform obj %.4g user obj ' ...
                         '%.4g (%g %g])\n'], frac, frac0);
            end
        end;

        
        function optimize(obj)
            stopper=Stopper('Optimizing:\n');
            % Deformation objective.
            [o, o_c]=obj.generateObjective();
            % Space constraints.
            s_c=obj.generateSpaceConstraints();
            % User constraint (as soft objective)
            [obj.user_objective, user_c] = obj.Projection.getObjective();
            if obj.learn_C
                all_o = o + obj.user_objective + obj.c_objective;
                c=obj.gatherConstraints(o_c,s_c,user_c,obj.c_constraint);
            else
                all_o = (o + obj.user_objective);
                c=obj.gatherConstraints(o_c,s_c,user_c,[]);
            end
            obj.runSolver(c, all_o);
            fprintf('Overall optimization time: ');
            obj.log.t_optimize=stopper.stop;
        end
        
        function visualize(obj)
            tri=obj.tris;
            Y=double(obj.Y);
            patch('Faces',tri,'Vertices',Y,'FaceColor', ...
                  'c', 'EdgeColor', [0.7, 0.7, 0.7], 'EdgeAlpha', 0.5, 'LineWidth', 0.3);
            d=max(obj.distortion);
            f=nnz(obj.flipped);
            title(sprintf('Distortion: %g\nFlipped: %d\nObjective: %d',d,f,obj.objVal));
            axis equal;
        end
        
        function dispReport(obj)
            d=max(obj.distortion);
            f=nnz(obj.flipped);
            fprintf('Distortion: %g\nFlipped: %d\nObjective: %g\n',d,f,obj.objVal);
        end
        
        function setFrames(obj,source)
            if isempty(source)
                Id = eye(obj.target_dim,obj.target_dim);
                for i=1:numel(obj.faces)
                    obj.faces(i).F = Id;
                end
            elseif isa(source,'Problem')
                [obj.faces.F] = source.faces.F;
            elseif isequal(size(source),size(obj.Y)),
                obj.INIT_Y=source;
                for i = 1:obj.num_image
                    assign(obj.Y{i}, double(source{i}));
                end
                obj.updateFrames;
                obj.updateDistortion;
            else
                error('invalid source type')
            end;
        end
        
        function visualizeSurface(obj) %Not used.
            X=double(obj.Y);
            tri=obj.tris(:,1:3);
            trimesh(tri,X(:,1), X(:,2), X(:,3),'EdgeColor','k','FaceColor','flat','FaceVertexCData',obj.distortion','CDataMapping','scaled','FaceLighting','flat','BackFaceLighting','lit');%,'VertexNormals',VN);
        end
        
        function calcObjectiveVal(obj)
            % calculate objective (useful when objective is defined in epigraph form
            try
                stopper=Stopper('Initializing objective:\n');
                [o, o_c]=obj.generateObjective();
                c = o_c + (obj.Y==double(obj.Y));
                solvesdp(c, o, obj.yalmip_prm);
                obj.objVal=double(o);
                obj.log.t_calcObjectiveVal=stopper.stop;
            catch
                warning('Objective value could not be initialized');
            end
        end
        
        function computeStiffnessObjective(obj)
            if obj.learn_C
                % weight by the number of images.
                % t = sdpvar(obj.num_face, 1);
                obj.c_objective = obj.num_image * obj.s_lambda * norm(obj.C, 1);
                obj.c_constraint = obj.C >= 0;
                % In epigraph form
                % obj.c_objective = obj.num_image * obj.s_lambda * ...
                %     sum(t);
                % obj.c_constraint =  ((-t<= obj.C) + (obj.C<= t) + ...
                %     (obj.C >= 0));
            end
        end
    end
end

