classdef Camera<handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Class representing a single camera element.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        M; % Motion (Intrinsic + Extrinc)
        t; % Translation (Intrinsic + Extrinc)
        
        % Extra info.
        K; % Intrinsic
        R; % Extrinsic 
        T; % Extrinsic
    end
    
    methods
        function new = copy(this)
            new = feval(class(this));            
            p = properties(this);
            for i = 1:length(p);
                new.(p{i}) = this.(p{i});
            end
        end
    end
end
