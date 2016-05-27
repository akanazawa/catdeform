function script_run_many_cat()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Script for the ablation studies, to compute the deformation for 10 cats
% with 'UNIFORM' and 'NO_BOUND' method as discussed in the paper.
%
% Default setting runs the UNIFORM setting.
%
% For NO_BOUNDS, uncomment the "No Bounds" section (line 19-20) and comment
% the "Uniform" block (line 23-27). 
%
% For re-using a pre-learned stiffness, uncomment line 30-34, also adjust the
% meta.articulation.source_file appropriately (You can run script_joint_cat.m
% or download the results from
% http://www.umiacs.umd.edu/~kanazawa/code/catdeform_results.tar.gz
% and put it in project root.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initialize;

IMG_DIR = '../data/cat_train/';

%% ---------- No Bounds ----------
% meta.algorithm = AlgorithmEnum.CAM_N_FRAME;
% meta.optimSpace = SpaceEnum.NONE;
%% ---------- Uniform ----------
% meta.algorithm = AlgorithmEnum.WARM;
% meta.optimSpace = SpaceEnum.BI;
% meta.articulation = ArticulationEnum.UNIFORM;
% meta.articulation.uniform_C = 2;
% meta.algorithm.vals = linspace(1.01, 2, 10);
%% ---------- Example of reusing learned stiffness: ----------
% meta.algorithm = AlgorithmEnum.WARM;
% meta.optimSpace = SpaceEnum.BI;
% meta.articulation = ArticulationEnum.PREDEFINED;
% meta.articulation.source_file = '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontal-jumping-left_walking-licking-rolled-running-sit_curled-sitting_profile-twisting-walking/stiffness.mat';
% meta.algorithm.vals = logspace(-1.3, -0.3, 10);
%----------
meta.objective = ObjectiveEnum.ARAP;
meta.user_lambda = 10;

images = {...
    'frontal.jpg',
    'jumping.jpg',
    'left_walking.jpg',
    'licking.jpg',
    'rolled.png',
    'running.jpg',
    'sit_curled.png',
    'sitting_profile.jpg',
    'twisting.jpg',
    'walking.jpg'...
    };
% Reuse experiment:
% images = {...
%     'black.jpg',
%     'halfsit.png',
%          }; 

global supressVis; % supress visualization
for i = 1:length(images)
    param = param_cat({fullfile(IMG_DIR, images{i})}, meta);
    app_deform2d(param);
end
    

