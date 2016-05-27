classdef ControlMesh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% CONTROLMESH: The tetrahedral mesh that we modify.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties
        %vars of the actual control mesh
        coarseX;
        coarseY;
        coarseTri;
        fineX;
        fineTri;
        fineY;
        coeff; % matrix s.t. fineX=coeff*coarseX
        nSamplesSave = 50;
        % for stiffness: 
        stiffness = [];
        num_img;
    end
    
    methods
        function obj=ControlMesh(fineX,fineTri,coarseX,coarseTri,coeff,num_img)
            obj.fineX=fineX;
            obj.fineTri=fineTri;
            obj.coarseX=coarseX;
            obj.coarseTri=coarseTri;
            obj.coeff = coeff;
            obj.num_img = num_img;
            fprintf('Deriving fine variables for %d images\n', ...
                    obj.num_img);
            for i = 1:obj.num_img
                obj.coarseY{i} = sdpvar(size(obj.coarseX,1), size(obj.coarseX,2), 'full');
                assign(obj.coarseY{i}, obj.coarseX);
                obj.fineY{i}=obj.coeff*obj.coarseY{i};
            end
        end
        
        function visualize(obj)
            for i = 1:obj.num_img
                sfigure(100+i);
                curFineY = value(obj.fineY{i});
                curCourseY = value(obj.coarseY{i});
                patch('Faces',obj.fineTri,'Vertices', curFineY, ...
                      'FaceColor', 'none', 'EdgeColor', 'b', ...
                      'LineStyle', 'none', 'Marker', '.', 'MarkerSize', ...
                      5);
                hold on;
                if ~isempty(obj.stiffness)
                    stiffness = value(obj.stiffness);
                    patch('Faces',obj.coarseTri,'Vertices',curCourseY,'FaceColor', ...
                          'flat','FaceVertexCData', stiffness, 'EdgeColor', [0.7, 0.7, ...
                                        0.7], 'EdgeAlpha', 0.5, ...
                          'LineWidth', 0.3);
                    colormap(spring);
                else
                    patch('Faces',obj.coarseTri,'Vertices',curCourseY,'FaceColor', ...
                          'c', 'EdgeColor', [0.7, 0.7, 0.7], ...
                          'EdgeAlpha', 0.5, 'LineWidth', 0.3);
                end
                axis equal;
                cameratoolbar;
                cameratoolbar('SetCoordSys','z');
                
                set(gcf, 'color', 'w');
                keyboard
            end
        end
        
        function reset()
        %% Resets sdpvar to initial coarseX.
            for i = 1:obj.num_img
                assign(obj.coarseY{i}, obj.coarseX);
            end                
        end
        
        function saveCM(obj, controlFName, distortion, flipped, ...
                        bounds, frames)
            if numel(distortion) == 1 
                distortion = distortion * ones(size(obj.coarseTri, 1), obj.num_img);
                flipped = flipped * ones(size(obj.coarseTri, 1), obj.num_img);
            else
                distortion = reshape(distortion, size(obj.coarseTri, 1), obj.num_img);
                flipped = reshape(flipped, size(obj.coarseTri, 1), ...
                                  obj.num_img);
            end
            fprintf('Saving CM... ');
            nDigits=ceil(log10(size(obj.coeff,1)));
            for j = 1:length(obj.fineY)
                clear v_scalars;
                v_scalars.X = obj.coarseX(:,1);
                v_scalars.Y = obj.coarseX(:,2);
                v_scalars.Z = obj.coarseX(:,3);

                t_scalars = [];
                t_scalars.distortion = distortion(:, j);
                t_scalars.flipped = flipped(:, j);
                % save stiffness label for face (N_coarse_face x 1)
                if numel(bounds) == 1
                    t_scalars.bounds = bounds * ones(size(obj.coarseTri, ...
                                                             1), 1);
                else
                    t_scalars.bounds = bounds;                    
                end
                
                % save rotation for each face.
                if ~isempty(frames)
                    t_scalars.frame_roll = zeros(length(frames), 1);
                    t_scalars.frame_pitch = zeros(length(frames), 1);
                    t_scalars.frame_yaw = zeros(length(frames), 1);
                    for i = 1:length(frames)
                        [t_scalars.frame_roll(i) t_scalars.frame_pitch(i) t_scalars.frame_yaw(i)] = rotationMatrix(frames{i});
                    end
                end
                Y_centered = value(obj.coarseY{j});
                Y_centered = bsxfun(@minus, Y_centered, mean(Y_centered));
                output_vtk_tets(controlFName{j},Y_centered, ...
                                obj.coarseTri,v_scalars,t_scalars);
            end % For each image.
            fprintf('Done.\n');
        end

    end
end
