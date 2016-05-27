classdef Face<handle
% Class representing a single mesh element (face)
%
% Input:
% T - triangulation
% X - vertices
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Controlling Singular Values with Semidefinite Programming".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Shahar Kovalsky (http://www.wisdom.weizmann.ac.il/~shaharko/)
%        and Noam Aigerman   (http://www.wisdom.weizmann.ac.il/~noamaig/)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        A; % differential
        F; % rotation (frame), given by the polar decomposition
        K; % upper bound on singular values
        k; % lower bound on singular values
    end
end

