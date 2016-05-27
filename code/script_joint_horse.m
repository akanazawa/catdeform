function script_joint_horse()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Script to solve the stiffness + deformation for 10 horses.
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
meta.symmetric_stiffness = true;

IMG_DIR = '../data/horse_train/';

img_set = {...
    {...
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
    },
          };

param = param_horse(fullfile(IMG_DIR, img_set{1}), meta);
app_deform2d(param);

