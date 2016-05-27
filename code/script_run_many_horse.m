function script_run_many_horse()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Horse version of script_run_many_cat.m, see script_run_many_cat.m for details.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('param_cat', 'file')
    addpath('./param');
end
IMG_DIR = '../data/horse_train/';
%% ---------- No bounds ----------
meta.optimSpace = SpaceEnum.NONE;
meta.algorithm = AlgorithmEnum.CAM_N_FRAME;
meta.articulation = ArticulationEnum.NONE;
%% ---------- Uniform ----------
% meta.algorithm = AlgorithmEnum.WARM;
% meta.optimSpace = SpaceEnum.BI;
% meta.articulation = ArticulationEnum.UNIFORM;
% meta.articulation.uniform_C = 2;
% meta.algorithm.vals = linspace(1.01, 2, 10);
%----------
meta.objective = ObjectiveEnum.ARAP;
meta.symmetric_stiffness = true;
meta.user_lambda = 10;

images = {...
    'frontalrun.png',
    'gallop.png',
    'graze.jpg',
    'grazing.png',
    'leftjog.png',
    'lookleft.png',
    'ote.png',
    'profile.png',
    'run.png',
    'straightleft.png',
    'white.png',...
         }; 

global supressVis; % supress visualization
for i = 1:length(images)
    param = param_horse({fullfile(IMG_DIR, images{i})}, meta);
    app_deform2d(param);
end
    

