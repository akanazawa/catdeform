classdef ArticulationEnum<handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    properties
        % If PREDEFINED, gets stiffness values from here.
        source_file;
        % If UNIFORM, uses this value everywhere.
        uniform_C;
        % If LEARN, this is the weight for the sparse regularization
        s_lambda = 1; 
    end
    enumeration
        UNIFORM;
        LEARN;
        PREDEFINED;
        NONE;
    end
    
end

