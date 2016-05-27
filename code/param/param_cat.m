function param = param_cat(img_set, major_param)
%%%%%%%%%%%%%%%%%%%%
% Defines parameter for deformation.
% Deforms the input image according to its annotation.
% Parameters for:
% 0. Optimization weights (lambdas)
% 1. Optimization type (Objective & Space constraints)
% 2. Tetrahedron generation
% 3. Algorithm Iteration
% Computes output name for results.

if nargin == 1
    major_param = [];
end

initialize;

% Sort input.
img_set = sort(img_set);

%% Optimization Weights 
%%----------
if isfield(major_param, 'user_lambda')
    param.user_lambda = major_param.user_lambda;
else
    param.user_lambda = 10;
end
str.lambda = sprintf('ulam_%g', param.user_lambda);

%% Optimization settings
%%----------
if isfield(major_param, 'objective')
    param.objective = major_param.objective;
else
    param.objective = ObjectiveEnum.ARAP;
end
if isfield(major_param, 'optimSpace')
    param.optimSpace = major_param.optimSpace;
else
    param.optimSpace = SpaceEnum.BI;
end
str.opt = sprintf('%s_%s', char(param.objective), char(param.optimSpace));

if param.optimSpace ~= SpaceEnum.NONE
    if isfield(major_param, 'articulation')
        param.articulation = major_param.articulation;
    else
        param.articulation = ArticulationEnum.LEARN;
        param.articulation.s_lambda = 0.05;
    end
    switch param.articulation
      case ArticulationEnum.LEARN
        str.opt = [str.opt, sprintf('_slam_%g', ...
                               param.articulation.s_lambda)];
      case ArticulationEnum.UNIFORM
        str.opt = [str.opt, sprintf('_uniformC_%g', ...
                                    param.articulation.uniform_C)];
      case ArticulationEnum.PREDEFINED
        param.articulation.source_file = major_param.articulation.source_file;
    end
else
    param.articulation = ArticulationEnum.NONE;
end


%% Algorithm type
%%----------
if isfield(major_param, 'algorithm')
    param.algorithm = major_param.algorithm;
else
    param.algorithm = AlgorithmEnum.WARM;
    param.algorithm.vals = logspace(-1.3, -0.3, 10);
end

str.algorithm = sprintf('%s', char(param.algorithm));

if param.algorithm == AlgorithmEnum.WARM
    % Add this info.
    if param.articulation == ArticulationEnum.LEARN
        if param.algorithm.vals(1) < param.algorithm.vals(end)
            % Flip for learning.
            param.algorithm.vals = param.algorithm.vals(end:-1:1);
        end
        str.opt = sprintf('%s_%s_slam_warm_%g_to_%g_nstep_%d', ...
                          char(param.objective),...
                          char(param.optimSpace),...
                          param.algorithm.vals(1),...
                          param.algorithm.vals(end),...
                          length(param.algorithm.vals));
    elseif param.articulation == ArticulationEnum.UNIFORM
        assert(param.algorithm.vals(1) < param.algorithm.vals(end));
        str.opt = sprintf('%s_%s_uniformC_warm_%g_to_%g_nstep_%d', ...
                          char(param.objective),...
                          char(param.optimSpace),...
                          param.algorithm.vals(1),...
                          param.algorithm.vals(end),...
                          length(param.algorithm.vals));
    elseif param.articulation == ArticulationEnum.PREDEFINED
        assert(param.algorithm.vals(1) < param.algorithm.vals(end));
        str.opt = sprintf('%s_%s_predefinedStiffness_from10cats_warm_%g_to_%g_nstep_%d', ...
                          char(param.objective),...
                          char(param.optimSpace),...
                          param.algorithm.vals(1),...
                          param.algorithm.vals(end),...
                          length(param.algorithm.vals));        
    end
end

%% Tetrahedron generation
%%----------
if isfield(major_param, 'tetgen')
    param.tetgen_targetfacecount = major_param.tetgen.facecount;
    param.tetgen_maxvol = major_param.tetgen.maxvol;
else
    param.tetgen_targetfacecount = 500;
    param.tetgen_maxvol = 300;
end
param.tetgen_maxtetcount = 3000;
param.tetgen_maxretries = 50;
% Needed for certain mesh that has problem with intersecting faces.
param.tetgen_repairmesh = true;
% param.CM_type = 'iso2mesh'; % Use CGAL simplification
param.CM_type = 'manual_iso2mesh'; %manually simplified mesh
if strcmp(param.CM_type, 'manual_iso2mesh')
    param.tetgen_simplified_mesh = '../data/models/cat_nomouth_v240f476.ply';
    param.tetgen_targetfacecount = 380;
    param.tetgen_maxvol = 1;
end

param.CM_coordsType = 'mls';
param.CM_mlsSimga = 0.4; 
param.CM_mlsOrder = 1;

if strcmp(param.CM_type , 'manual_iso2mesh');
    [~, simplified_name, ~] = fileparts(param.tetgen_simplified_mesh);
    str.tet = sprintf('manual-%s-tetface_%d_maxvol_%d', ...
                      simplified_name, ...
                      param.tetgen_targetfacecount, ...
                      param.tetgen_maxvol);
else
    str.tet = sprintf('tetface_%d_maxvol_%d', param.tetgen_targetfacecount, ...
                      param.tetgen_maxvol);
end
str.tet_detail = sprintf('maxcount_%d_%s_%s_sig_%g', ...
                         param.tetgen_maxtetcount, param.CM_type, ...
                         param.CM_coordsType, param.CM_mlsSimga);

%% Symmetric stiffness settings
% computed by assign_symmetry2tet.m
if param.articulation == ArticulationEnum.LEARN 
    if isfield(major_param, 'symmetric_stiffness')
        param.symmetric_stiffness = major_param.symmetric_stiffness;
    else
        param.symmetric_stiffness = false;
    end
    if param.symmetric_stiffness
        sym_path = '../data/models/symmetric_stiffness_id_cat_nomouth_v240f476-tetface_380_maxvol_1_maxcount_3000_manual_iso2mesh_mls_sig_0.4.mat';
        t = load(sym_path);
        % Holds ids (shared by symmetric tets) 
        param.stiffness_id = t.stiffness_id;
    end
else
    param.symmetric_stiffness = false;
end

%% Iteration settings
%%----------
param.deform_max_iter = 15;
param.camera_max_iter = 15;
% Cam N frame settings
param.cam_n_frame_max_iter = 30;
% Warm settings
param.warm_iterPerC = 8;

%% Model settings
%%----------
param.off_file = ['../data/models/cat0_nointfaces_centered.off'];

%% Get annotation for the image.
%%----------
param.num_img = length(img_set);
param.img_path = img_set;

for i = 1:param.num_img
    [pathstr, img_name, ext] = fileparts(img_set{i});
    param.img_name{i} = img_name;
    param.img_anno_path{i} = regexprep(img_set{i}, ['\',ext,'$'], ...
                                 '_annotated_wcat0.mat');
    % Preprocess (center/scale & flip) the anchors.
    load(param.img_anno_path{i}, 'annotation');
    flipped = annotation.twoD;
    % IMPORTANT: for camera estimation to work correctly, the Y
    % axis of the 2D observation has to be in usual cartesian xy-order i.e. going
    % vertically down decreases the Y-value, unlike the way the image coordinates are read.
    flipped(:, 2) = -flipped(:, 2);
    normed = normalize_points(flipped');
    param.anchor_coords{i} = normed;
    param.anchors{i} = annotation.v_ids;

end

%% Turn on for visualizing the annotation in 2D and 3D.
if 0
    [X, Xtri]=readMesh(param.off_file);
    for i = 1:param.num_img
        I = imread(param.img_path{i});
        load(param.img_anno_path{i}, 'annotation');
        twoD_pts = annotation.twoD;
        marked = X(param.anchors{i}, :);

        %% Draw
        sfigure(1); clf;
        subplot(121);
        imagesc(I); axis image off;
        hold on;

        plot(twoD_pts(:, 1), twoD_pts(:, 2), 'o', 'MarkerSize', 8, 'MarkerEdgeColor', ...
             'w', 'MarkerFaceColor', 'r');
        for j = 1:size(twoD_pts, 1)
            text(twoD_pts(j, 1), twoD_pts(j, 2), sprintf('%d',j), ...
                 'Color', 'w');
        end

        subplot(122);
        patch('Faces',Xtri,'Vertices', X,'EdgeColor',[0.7 0.7 0.7], ...
              'FaceColor','w','Marker', '.', 'MarkerSize', ...
              5, 'MarkerEdgeColor', 'k');
        hold on;
        plot3(marked(:, 1), marked(:, 2), marked(:, 3), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', ...
              'w', 'MarkerFaceColor', 'r');
        for j = 1:size(marked, 1)
            text(marked(j, 1), marked(j, 2), marked(j, 3), sprintf('%d',j), ...
                 'Color', 'k');
        end
        plot3(0,0,0,'y*', 'MarkerSize', 20);
        cameratoolbar;
        campos([-22 -38 7]);
        axis equal;
        grid on;
        set(gcf, 'color', 'w');
        keyboard
    end
end
%% Save names.
% $Base = results/<ALG_TYPE>/cat_<tettrgt>_<maxvol>
% If more than one image at once
%    AND no learning (happens least)
%       $Base/<objective>_<space>_<ulam>/<setname>_nolearning/{img_name}/
%    WITH learning
%       $Base/<objective>_<space>_<ulam>_<slam>/<setname>/{img_name}/
% diary, log, etc goes under $Base/<objective>_<space>_<ulam>_<slam>/<setname>/
% If one image
%    NO learning
%       $Base/<objective>_<space>_<ulam>/<img_name>/
%    WITH learning
%       $Base/<objective>_<space>_<ulam>_<slam>/<img_name>/
% diary, log, etc goes under $base/<objective>_<space>_<ulam>_<slam>/<img_name>/

%%----------
if isempty(strfind(param.off_file, 'nointfaces'))
    param.base_dir = sprintf('../results/%s/catorig_%s/', str.algorithm, str.tet);
else
    param.base_dir = sprintf('../results/%s/cat_%s/', str.algorithm, str.tet);
end

% Save tet & base CM.
param.tet_out = fullfile(param.base_dir, [str.tet_detail, '.mat']);

do_learn = param.optimSpace ~= SpaceEnum.NONE && param.articulation ...
        == ArticulationEnum.LEARN;

if param.symmetric_stiffness
    str.lambda = [str.lambda '_symmetric-stiffness'];
end

if param.num_img > 1
    set_name =  strjoin(param.img_name,'-');
    if ~do_learn
        set_name = [set_name, '_nolearning'];
    end
    param.base_result_dir = fullfile(param.base_dir, ...
                                     sprintf('%s_%s', str.opt, str.lambda),...
                                     set_name);
else
    param.base_result_dir = fullfile(param.base_dir, ...
                                     sprintf('%s_%s', str.opt, str.lambda));
end

for i = 1:param.num_img
    param.output_dir{i} = fullfile(param.base_result_dir, param.img_name{i});
end

% Other stuff to write.
if param.num_img == 1
    param.done_out = fullfile(param.output_dir{1}, 'done.mat');
    param.log_out = fullfile(param.output_dir{1}, 'result_log');
    param.cam_out = fullfile(param.output_dir{1}, 'cameras.mat');
    param.diary_out = fullfile(param.output_dir{1}, ...
                               'diary_%s.txt');
    if do_learn
        param.s_out = fullfile(param.output_dir{1}, 'stiffness.mat');
    end
else
    param.done_out = fullfile(param.base_result_dir, 'done.mat');
    param.log_out = fullfile(param.base_result_dir, 'result_log');
    param.cam_out = fullfile(param.base_result_dir, 'cameras.mat');
    param.diary_out = fullfile(param.base_result_dir, 'diary_%s.txt');
    if do_learn
        param.s_out = fullfile(param.base_result_dir, 'stiffness.mat');
    end
end

param.str = str;



