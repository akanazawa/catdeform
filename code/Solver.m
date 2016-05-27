classdef Solver<handle
% Wrapper class for mesh optimization problem with constraints
% on singular values. 
% Can be used for various mesh optimization problems
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Controlling Singular Values with Semidefinite Programming".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Shahar Kovalsky (http://www.wisdom.weizmann.ac.il/~shaharko/)
%        and Noam Aigerman   (http://www.wisdom.weizmann.ac.il/~noamaig/)
% Edited by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    properties
        problem; % problem instance
        lastY; % prev value, for convergence detection
        maxIter = 1000; % maximal number of optimization iterations
        tolY = 1e-3; % stopping criteria
        tolObj = 1e-3; % stopping criteria
        log=[]; % optimization log
        plotLogPerIter=true;
        % AJ: save intermediate VTK for each solver iter.
        do_aj_save = true;
        baseDeformedFName;
        baseControlFName;
        CM;
        lastObjVal;
        lastA;
        algorithm; % Decides whether Camera gets updated in solve()
                   % or not.
        lastCam; 
        lastDist;
    end
    
    methods
        function obj=Solver(problem)
            obj.problem=problem;
        end
        
        function solve(obj, iter)
            fprintf('\n====================================================\n');
            % visualize initial state
            if ~chkFlag('supressVis')
                sfigure(200);
                obj.problem.visualize();
                drawnow; pause(0.01);
            end
            if ~exist('iter', 'var')
                iter=1;
                % log initial state
                fprintf('Initial State\n');
                obj.problem.dispReport();
                obj.updateLog(0);
                
                % initilialize solver loop
                if obj.do_aj_save
                    obj.save_aj(obj.baseControlFName, obj.baseDeformedFName, ...
                                0);
                end
            end
            lastY = value(cat(1, obj.problem.Y{:}));
            obj.lastY = bsxfun(@minus, lastY, mean(lastY));
            obj.lastA = vec(value([obj.problem.faces.A]));
            obj.lastObjVal= Inf;
            obj.lastCam = zeros(2*obj.problem.num_image, 4);
            obj.lastDist = obj.problem.distortion;

            while(iter<=obj.maxIter)
                
                t_start = tic;
                fprintf('\n==========================\n');
                fprintf('Solver iter %d\n', iter);
                
                if (obj.algorithm == AlgorithmEnum.CAM_N_FRAME) | ...
                    (obj.algorithm == AlgorithmEnum.WARM)
                    % Update projection matrix
                    obj.problem.Projection.updateCamera();
                    if ~chkFlag('supressVis')
                        % visualize camera?
                    end
                end
                
                obj.problem.optimize();
                obj.problem.updateDistortion();
                obj.problem.dispReport();
                obj.updateLog(iter);
                
                if ~chkFlag('supressVis')
                    sfigure(200); cla;
                    obj.problem.visualize();
                    if obj.plotLogPerIter,
                        sfigure(201);
                        obj.plotLog();
                    end;
                    drawnow; pause(0.01);
                end;
                
                % Save if needed
                if obj.do_aj_save
                    obj.save_aj(obj.baseControlFName, ...
                                obj.baseDeformedFName, iter);
                end

                if obj.problem.learn_C
                    Cmat = obj.problem.base_C + repmat(value(obj.problem.C), [1 ...
                                        obj.problem.num_image]);
                else
                    % Cmat = repmat(obj.problem.C, [1 ...
                    %                     obj.problem.num_image]);
                    Cmat = obj.problem.C;
                end
                bad=obj.problem.distortion(:)>Cmat(:)+1e-2 | vec(obj.problem.flipped);
                if all(vec(double(cat(1, obj.problem.Y{:}))) == 0)
                    fprintf('Bad sol!!\n');  
                    if length(obj.problem.Y) > 1
                        keyboard; 
                    else
                        assign(obj.problem.Y{1}, lastY);
                    end
                    break;
                end                
                
                % update frames
                % Y = value(cat(1, obj.problem.Y{:}));
                Y = value(obj.problem.Y);

                %%CENTER IT to remove translation ambiguity. Note
                %%only necessary for 3D-2D.
                if length(Y) == 1
                    Y = value(cat(1, Y{:}));
                    Y = bsxfun(@minus, Y, mean(Y));
                else
                    for i = 1:length(Y)
                        Y{i} = bsxfun(@minus, value(Y{i}), value(mean(Y{i})));
                    end
                    Y = cat(1, Y{:});
                end
                
                A = vec(value([obj.problem.faces.A]));
                obj.problem.updateFrames();
                
                % conclude iteration
                t_stop = toc(t_start);
                fprintf('iteration time: %.2f secs\n', t_stop);
                fprintf('==========================\n');
                iter=iter+1;
                % stopping criteria
                % Y_diff = norm(vec(Y-obj.lastY),'inf');
                Y_diff = norm(vec(Y-obj.lastY)) ./ norm(vec(obj.lastY));
                if Y_diff <obj.tolY
                    fprintf('Stopping (tolY) %g @ iter %d\n', Y_diff, ...
                            iter);
                    break;
                else
                    fprintf('Y difference %g\n', Y_diff);
                end
                
                A_diff = norm(vec(A-obj.lastA),'inf');
                if A_diff <obj.tolY
                    fprintf('Stopping (tolA) %g @ iter %d\n', A_diff, ...
                            iter);
                    % break;
                else
                    fprintf('A difference %g\n', A_diff);
                end
                
                % AJ: stop if objective is not changing.
                obj_diff = (obj.lastObjVal - obj.problem.objVal) ./obj.lastObjVal;
                if abs(obj_diff) < obj.tolObj                
                    fprintf(['Stopping Objective value is not changing ' ...
                             '%g @ iter %d\n'], obj_diff, iter);
                    break;
                elseif obj.lastObjVal < obj.problem.objVal
                    fprintf(['problem! objective went up from %.5g to ' ...
                             '%.5g (reldiff %g)\n'], obj.lastObjVal, obj.problem.objVal, ...
                            obj_diff);
                else
                    fprintf('Objective val difference %g\n', obj_diff);
                end
                
                %% IF cam_n_frame, check convergence of camera.
                if (obj.algorithm == AlgorithmEnum.CAM_N_FRAME) |...
                    (obj.algorithm == AlgorithmEnum.WARM)
                    cam_here = [reshape([obj.problem.Projection.camera.M], [], 3), ...
                                vec([obj.problem.Projection.camera ...
                                     .t])];
                    cam_diff = norm(vec(cam_here - obj.lastCam));
                    if cam_diff < obj.tolY
                        fprintf(['Camera converged %g @ iter %d\n'], ...
                                cam_diff, iter);
                    else
                        fprintf('Camera difference %g\n', cam_diff);
                    end
                    obj.lastCam = cam_here;
                end

                if 0
                    obj.problem.Projection.visualizeCameras();
                    sfigure(1); clf; 
                    % patch('Faces',obj.CM.fineTri,'Vertices', ...
                    %       obj.CM.coeff*obj.lastY, 'FaceColor', 0.7*ones(1,3), 'EdgeColor','k');
                    % hold on;
                    % patch('Faces',obj.CM.fineTri,'Vertices', ...
                    %       obj.CM.coeff*Y, 'FaceColor', [240,128,128]/255, 'EdgeColor','k');
                    % axis image;
                    patch('Faces',obj.CM.coarseTri,'Vertices', ...
                          obj.lastY, 'FaceColor', 0.7*ones(1,3), 'EdgeColor','k');
                    hold on;
                    patch('Faces',obj.CM.coarseTri,'Vertices', ...
                          Y, 'FaceColor', [240,128,128]/255, 'EdgeColor','k');
                    axis image;
                    patch('Faces',obj.CM.coarseTri,'Vertices', ...
                          obj.lastY, 'FaceColor', 'flat','FaceVertexCData',obj.lastDist', 'EdgeColor','k');
                    hold on;
                    patch('Faces',obj.CM.coarseTri,'Vertices', ...
                          Y, 'FaceColor', 'flat','FaceVertexCData',obj.problem.distortion', 'EdgeColor','k');
                    axis image;
                    cameratoolbar;
                    obj.lastDist = obj.problem.distortion;%debugonly
                end
                
                obj.lastY=Y;
                obj.lastObjVal=obj.problem.objVal;                            
            end
            fprintf('Done.\n');
        end
        
        function updateLog(obj,iter)
            indLast=length(obj.log);
            ind=indLast+1;
            obj.log(ind).iter=iter;
            obj.log(ind).time=now;

            if obj.problem.learn_C
                obj.log(ind).C=obj.problem.base_C + value(obj.problem.C);
            else
                obj.log(ind).C=double(obj.problem.C);
            end
            obj.log(ind).objVal=obj.problem.objVal;
            obj.log(ind).distortion=obj.problem.distortion;
            obj.log(ind).flipped=obj.problem.flipped;
            obj.log(ind).problem_log=obj.problem.log;
            fprintf('Updating log item #%d (%s)\n',ind,datestr(now));
        end
        function plotLog(obj,range)
            clf;
            % prepare data
            iter=cat(1,obj.log.iter);
            objVal=cat(1,obj.log.objVal);
            for i=1:length(obj.log),
                maxDist(i)=max(obj.log(i).distortion);
                numFlips(i)=nnz(obj.log(i).flipped);
            end
            counter=1:length(iter);
            if ~exist('range','var')
                range=counter(1:end);
            end
            % plot
            AX = gca;
            plot(counter(range),objVal(range), 'b');
            addaxis(counter(range),maxDist(range), 'g');
            addaxislabel(1,'Objective');
            addaxislabel(2,'Maximal distortion');
            if any(numFlips)
                addaxis(counter(range),numFlips(range),[min(numFlips) ...
                                    max(numFlips)+1], 'r');
                addaxislabel(3,'Number of flips');
            end
            
            % add vertical lines for resets
            ind=range(iter(range)==0);
            for i=1:length(ind),
                line([ind(i) ind(i)],get(AX(1),'YLim'),'Color',[.5 .5 .5],'LineStyle',':')
            end
        end
        
        function save_aj(obj, controlFName, deformedFName, iter)
            fprintf('Saving intermediate result... ');
            controlFName = cellfun(@(x)sprintf(x, iter), controlFName, 'UniformOutput', false);
            deformedFName = cellfun(@(x)sprintf(x, iter), deformedFName, 'UniformOutput', false);

            coeff = obj.CM.coeff;
            for i = 1:length(obj.CM.fineY)
                clear v_scalars t_scalars;
                v_scalars.nVert = 1:size(obj.CM.fineY{i},1);
                Y_centered = value(obj.CM.fineY{i});
                Y_centered = bsxfun(@minus, Y_centered, mean(Y_centered));
                output_vtk_surf(deformedFName{i},Y_centered,obj.CM.fineTri, ...
                                v_scalars,[]);
            end

            if obj.problem.learn_C
                bounds = value(obj.problem.C) + obj.problem.base_C;
                if all(isnan(value(bounds))) 
                    bounds = zeros(size(bounds));
                end
            else                
                bounds = obj.problem.C;
            end

            obj.CM.saveCM(controlFName, obj.problem.distortion, ...
                          obj.problem.flipped, bounds, []);
        end
        
    end
end
