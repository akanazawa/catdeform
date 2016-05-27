function script_joint_cat()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Script to solve the stiffness + deformation for 10 cats.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initialize;

global supressVis; % supress visualization 
meta.algorithm = AlgorithmEnum.WARM;
meta.optimSpace = SpaceEnum.BI;
meta.objective = ObjectiveEnum.ARAP;
%----------
meta.articulation = ArticulationEnum.LEARN;
meta.algorithm.vals = logspace(-1.3, -0.3, 10);
%----------
meta.user_lambda = 10;

% ---------- Symmetric stiffness
% Setting this on makes sures that the stiffness assignments on
% the tet are symmetrical thus cutting the # of stiffness paramters by half.
% The symmetry on the tet mesh was computed using a rough heuristic in code
% util/assign_symmetry2tet, with user supplied axis of symmetry.
% Results are similar with and without, but symmetry gives slightly.better results..
meta.symmetric_stiffness = true;
%----------
IMG_DIR = '../data/cat_train/';

img_set = {...
    {'frontal.jpg',
     'jumping.jpg',
     'left_walking.jpg',
     'licking.jpg',
     'rolled.png',
     'running.jpg',
     'sit_curled.png',
     'sitting_profile.jpg',
     'twisting.jpg',
     'walking.jpg',
    },
          };

param = param_cat(fullfile(IMG_DIR, img_set{1}), meta);
app_deform2d(param);
