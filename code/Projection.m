classdef Projection<handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Given 2D coordinates for vertices in the 3D model, computes the
% projection matrix and provides this positional constraint to Problem.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        Y; % Surface vertices, SDP var from ControlMesh.
        Ytri; % Triangulation of Y.
        num_image;
        camera; 
        anchors; % {nImg x 1}
        anchor_coords; % {nImg x 1}
        log = []; 
        t; % sdpvar (epigraph)
        objective; % objective yalmip optimization variable).
        constraint; % objective related constraints (in epigraph
                    % form).
        scale; % TODO: Remove this in future
        regularizerWeight = 1;
        recalc_objective = true;
        proj_fun = @get_projection;
    end
    
    methods
        function obj=Projection(Y,Ytri,anchors, anchor_coords, weight, scale)
            fprintf('Initializing Projection object... ');
            obj.Y=Y;
            obj.Ytri=Ytri;
            obj.anchors = anchors;
            obj.anchor_coords = anchor_coords;
            obj.t=sdpvar;
            obj.num_image = length(anchors);
            obj.camera = Camera();
            obj.camera(obj.num_image, 1) = Camera();
            obj.regularizerWeight = weight;
            % TODO: Remove later. 
            obj.scale = scale;
            
            fprintf('Done.\n');
        end
        
        % When camera is updated, the objective is recalculated on its
        % next call.
        function updateCamera(obj)
            obj.updateLog();
            error = 0;
            for i = 1:obj.num_image
                Yi = value(obj.Y{i});
                [M, t, err, model] = obj.proj_fun(obj.anchor_coords{i}, Yi(obj.anchors{i}, :)');
                error = error + err;
                obj.camera(i).M = M;
                obj.camera(i).t = t;
                obj.camera(i).K = model.K;
                obj.camera(i).R = model.R;
            end
            fprintf('Total reprojection error %g\n', error);
            obj.recalc_objective = true;
        end
        
        function initCameras(obj, initial_cams)
            obj.updateLog();
            for i = 1:obj.num_image
                obj.camera(i).M = initial_cams{i}(1:2,1:3);
                obj.camera(i).t = [0; 0];
                obj.camera(i).K = eye(3);
                obj.camera(i).R = initial_cams{i};
            end
            obj.recalc_objective = true;
        end

        
        function [o, o_c]=getObjective(obj)
            if isempty(obj.camera(1).M)
                error('Camera is not set yet\n');
            end
            if obj.recalc_objective
                goal = cat(2, obj.anchor_coords{:});
                rhs = cell(obj.num_image, 1);
                for i = 1:obj.num_image
                    Yi = obj.Y{i};
                    rhs{i} = obj.camera(i).M * Yi(obj.anchors{i}, :)'...
                             + repmat(obj.camera(i).t, [1 length(obj.anchors{i})]);
                end

                rhs = cat(2, rhs{:});
                diff = goal(:) - rhs(:);
                % TODO: remove scale later.
                obj.objective = obj.scale .* obj.t;
                obj.constraint = cone(diff, obj.t);
                % Don't recalc until camera changes.
                recalc_objective = false;
            end
            o = obj.regularizerWeight * obj.objective;
            o_c = obj.constraint;
            
        end
        
        function updateLog(obj)
            if isempty(obj.camera(1).M)
                return;
            end
            indLast = length(obj.log);
            ind = indLast + 1;
            for i = 1:obj.num_image
                obj.log(ind).cameras(i) = copy(obj.camera(i));
            end
        end
        
        function [h]=visualizeCameras(obj, camera_id, fig_id)
        if nargin == 1
            Rs = cell(obj.num_image, length(obj.log)+1);
            ts = cell(obj.num_image, length(obj.log)+1);
            for i = 1:length(obj.log)
                cams = obj.log(i).cameras;
                [Rs{:, i}] = deal(cams.R);
                [ts{:, i}] = deal(cams.t);
            end
            % also get from current one.
            [Rs{:, end}] = deal(obj.camera.R);
            [ts{:, end}] = deal(obj.camera.t);
            for i = 1:obj.num_image
                model.R = cat(3, Rs{i,:});
                model.t = cat(2, ts{i,:});
                Y_here = value(obj.Y{i})';
                model.S = Y_here(:, obj.anchors{i});
                model.K = eye(3);%dummy
                sfigure(10); clf;
                plot_cameras(model, 10);
                title(sprintf('image %d/%d', i, obj.num_image));
                cameratoolbar;
                keyboard
            end
        else
            Rs = cell(length(obj.log) + 1, 1);
            ts = cell(length(obj.log) + 1, 1);
            for i = 1:length(obj.log)
                Rs{i} = [obj.log(i).cameras(camera_id).R];
                ts{i} = [obj.log(i).cameras(camera_id).t];
            end
            Rs{end} = obj.camera(camera_id).R;
            ts{end} = obj.camera(camera_id).t;
            model.R = cat(3, Rs{:});
            model.t = cat(2, ts{:});
            Y_here = value(obj.Y{camera_id})';
            model.S = Y_here(:, obj.anchors{camera_id});
            model.K = eye(3);
            plot_cameras(model, fig_id);
            title(sprintf('image %d/%d', camera_id, obj.num_image));
            cameratoolbar;
        end
        end
        
        function saveCameras(obj, targetFName)
            fprintf('Saving camera history... ');
            cam_history = reshape([obj.log.cameras], obj.num_image,length(obj.log));
            save(targetFName, 'cam_history');
            fprintf('Done.\n');
        end
        
        
        function setRegularizerWeight(obj,w)
            obj.regularizerWeight = w;
            fprintf('User Constraint weight updated to %g\n',w);
        end
        
    end
end
