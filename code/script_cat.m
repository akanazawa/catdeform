function script_cat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Same script as script_run_many_cat.m, but runs on a single image as opposed
% to a list of images. See description in script_run_many_cat.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initialize;

img_dir = '../data/cat_train/';

img_set = {[img_dir, 'twisting.jpg']};

%% ---------- No bounds ----------
% meta.algorithm = AlgorithmEnum.CAM_N_FRAME;
% meta.optimSpace = SpaceEnum.NONE;
% meta.articulation = ArticulationEnum.NONE;
%% ---------- Uniform ----------
meta.algorithm = AlgorithmEnum.WARM;
meta.optimSpace = SpaceEnum.BI;
meta.articulation = ArticulationEnum.UNIFORM;
meta.articulation.uniform_C = 2;
meta.algorithm.vals = linspace(1.01,2,10);
%% ---------- Example of reusing learned stiffness: ----------
% meta.algorithm = AlgorithmEnum.WARM;
% meta.optimSpace = SpaceEnum.BI;
% meta.articulation = ArticulationEnum.PREDEFINED;
% meta.articulation.source_file = ['../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontal-jumping-left_walking-licking-rolled-running-sit_curled-sitting_profile-twisting-walking/stiffness.mat'];
% meta.algorithm.vals = linspace(0.1, 1, 10);

%% ---------- Basic params
meta.objective = ObjectiveEnum.ARAP;
meta.user_lambda = 10;

param = param_cat(img_set,meta);

global supressVis; % supress visualization
h = tic;
app_deform2d(param);
toc(h);
