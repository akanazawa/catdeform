classdef AlgorithmEnum<handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        % If WARM, the values to warm up through (ex:linspace(1.01,2,10))
        vals;
    end
    enumeration
        CAM_N_FRAME; % Where in one step both camera and frame is
                     % updated. For baseline NO_BOUNDS, this is used because
                     % there is no constraint to do warm (incremental) deformation.
        WARM; % Annealing is applied on CAM_N_FRAME
              % where in the outerloop the shape is
              % allowed to slowly deform more i.e.
              % the regularization param (C or s_lam) is
              % slowly inc/decreased respecively.
              % Used for UNIFORM and STIFFNESS
    end
    
end

