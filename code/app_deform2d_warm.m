function app_deform2d_warm(param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code implementing the paper "Learning 3D Deformation of Animals from 2D Images".
% Disclaimer: The code is provided as-is for academic use only and without any guarantees. 
%             Please contact the authors to report any bugs.
% Written by Angjoo Kanazawa (http://www.umiacs.umd.edu/~kanazawa/)
%         based on (https://github.com/shaharkov/ContSingVal)
% ==========
% Main driver that sets up solvers to deform a template 3D model based on 2D
% inputs.
% This one applies warm start, where the regularization on deformation is
% slowly decreased to allow increasing levels of deformation. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% init
rng(1)
% close all;
% clear;
yalmip('clear');

%% copy parameters to variables
cellfun(@(x) assignin('caller',x,param.(x)), fields(param));
%% Test if this already exists or not.
% Hack test..
if exist(done_out) | exist([log_out, '.fig'], 'file')
    fprintf('\n\n%s exists. Skipping.\n\n', log_out);
    return;
end

%% setup stuff
for i = 1:length(output_dir)
    exists_or_mkdir(output_dir{i});
end
%% start a diary
diary(sprintf(param.diary_out, datestr(now, 'ddmmmyy_HHMMSS')));
disp(param) % display parmeters (for logging)
% if exist('articulation', 'var')
%     disp(param.articulation)
% end
fprintf('Images:\n')
fprintf('\t%s\n', param.img_name{:});
fprintf('Output:\n');
fprintf('\t%s\n', param.output_dir{:});
%% load data
[X, Xtri]=readMesh(off_file);
if ~exist(param.tet_out, 'file')
    fprintf('Computing tet and coefficients..\n');
    %% Compute the coarse tet
    switch CM_type
      case 'iso2mesh'
        tetgen_keepratio = tetgen_targetfacecount/size(Xtri,1);
        [node,elem,face]=surf2mesh_retry(X,Xtri,min(X),max(X),tetgen_keepratio,tetgen_maxvol,tetgen_maxtetcount,tetgen_maxretries,tetgen_repairmesh);
        CM_TR = triangulation(elem(:,1:4), node);
      case 'manual_iso2mesh'
        [X_simp, Xtri_simp]=readMesh(tetgen_simplified_mesh);
        tetgen_keepratio = tetgen_targetfacecount/size(Xtri_simp,1);
        [node,elem,face]=surf2mesh_retry(X_simp,Xtri_simp,min(X_simp),max(X_simp),tetgen_keepratio,tetgen_maxvol,tetgen_maxtetcount,tetgen_maxretries,tetgen_repairmesh);
        CM_TR = triangulation(elem(:,1:4), node);
        % Visualize: 
        % sfigure();
        % patch('Faces',CM_TR.ConnectivityList,'Vertices', CM_TR.Points, ...
        %       'FaceColor', 'c', 'EdgeColor', 'b', ...
        %       'LineWidth', 0.3);
        % axis equal;
        % cameratoolbar;
        % keyboard
        % cameratoolbar('SetCoordSys','z');
      otherwise
        error('invalid CM_type');
    end

    %% Compute the relative coordinates    
    prmCoords.coordsType = CM_coordsType;
    prmCoords.mlsSimga = CM_mlsSimga;
    prmCoords.mlsOrder = CM_mlsOrder;
    coeff = computeRelativeCoordinates3D(CM_TR.Points, ...
                                         CM_TR.ConnectivityList, ...
                                         X, Xtri, prmCoords);
    save(param.tet_out, 'CM_TR', 'coeff');
else
    fprintf('Loading tet.\n');
    load(param.tet_out, 'CM_TR', 'coeff');
end
%% Create control mesh for each image.
% Note: MATLAB is pass-by-reference if object is a handle (class) object.
% CM = ControlMesh(X, Xtri, CM_TR.Points, CM_TR.ConnectivityList, ...
%                  coeff, num_img);
coarseX  = CM_TR.Points;
coarseX = bsxfun(@minus, coarseX, mean(coarseX));
CM = ControlMesh(X, Xtri, CM_TR.Points, CM_TR.ConnectivityList, ...
                 coeff, num_img);

%% Setup projection class. (Weight for user constraint goes here).
scale = norm(max(CM.coarseX)-min(CM.coarseX));

PJ = Projection(CM.fineY, Xtri, anchors, anchor_coords, user_lambda, scale);


switch articulation
  case ArticulationEnum.NONE
    vecC = 0;
  case ArticulationEnum.UNIFORM
    vecC = articulation.uniform_C * ones(size(CM.coarseTri, 1), 1);
  case ArticulationEnum.PREDEFINED
    % load articulation.source_file;
    t = load(articulation.source_file);
    assert(length(t.stiffness) == size(CM.coarseTri, 1));
    biasC = t.bias;
    vecC = t.stiffness + biasC;
  case ArticulationEnum.LEARN
    lambda.stiffness = articulation.s_lambda;
    if symmetric_stiffness
        C_source = sdpvar(length(unique(stiffness_id)), 1, 'full');
        vecC = C_source(stiffness_id);
    else
        vecC = sdpvar(size(CM.coarseTri, 1), 1, 'full');
    end
    CM.stiffness = vecC;
end


% Weights.
lambda.objective = 1;

solver = Solver(Problem(CM.coarseX, CM.coarseTri, vecC, objective, ...
                        optimSpace, CM.coarseY, PJ, lambda));
solver.problem.setFrames(CM.coarseY);
solver.CM = CM;
solver.maxIter = warm_iterPerC;
solver.algorithm = algorithm;

% ---------- Solve ----------
h = tic;
for iter = 1:length(algorithm.vals)
    solver.baseDeformedFName = fullfile(output_dir, ...
                                        sprintf('out_deformed_(%02d%%02d).vtk', ...
                                                iter));
    solver.baseControlFName = fullfile(output_dir, ...
                                       sprintf('out_control_(%02d%%02d).vtk', iter));
    if articulation == ArticulationEnum.LEARN
        solver.problem.s_lambda = algorithm.vals(iter);
        solver.problem.computeStiffnessObjective();
        fprintf('$$$$$Solve with s_lam %g (%d/%d)$$$$$\n', algorithm.vals(iter),...
                iter, length(algorithm.vals));
    elseif articulation == ArticulationEnum.UNIFORM
        solver.problem.C = algorithm.vals(iter) * ones(size(CM.coarseTri, 1), 1);;
        % Update constraints with this new C.
        solver.problem.computeConstantSpaceConstraints();
        fprintf('$$$$$Solve with C %g (%d/%d)$$$$$\n', algorithm.vals(iter),...
                iter, length(algorithm.vals));
    elseif articulation == ArticulationEnum.PREDEFINED
        solver.problem.C = algorithm.vals(iter) .* vecC + biasC;
        fprintf('$$$$$Solve with stiffness * weightt %g (%d/%d)$$$$$\n', algorithm.vals(iter),...
                iter, length(algorithm.vals));
    end
    solver.solve();
end
fprintf('**********everything took %g seconds**********\n', toc(h));
% ---------- Save ----------
save(done_out, 'log_out');

% Save stiffness values.
if solver.problem.learn_C
    stiffness = value(solver.problem.C);
    bias = solver.problem.base_C;
    save(s_out, 'stiffness', 'bias');
end
PJ.saveCameras(cam_out); 

% Save log.
h = sfigure(1); clf;
set(gcf, 'color', 'w');
solver.plotLog();
export_fig(log_out, '-jpg', '-painters','-native');
saveas(h, log_out, 'fig');

blue = [156 199 234]./255;
for i = 1:num_img
    % h = sfigure(10); clf;
    % PJ.visualizeCameras(i, 10);
    % export_fig([output_dir{i}, '/camera'], '-jpg', '-zbuffer','-native');
    hfig = sfigure(1); clf;
    color = repmat(blue, size(CM.fineX,1), 1);
    fineY = double(CM.fineY{i});
    fineY = bsxfun(@minus, fineY, mean(fineY, 1));
    hp = patch('Faces',CM.fineTri,'Vertices', fineY, 'FaceColor', 'interp', 'FaceVertexCData', ...
               color, 'EdgeColor', 'none');
    axis image off; cameratoolbar;
    lighting gouraud;
    cameratoolbar;
    camtarget([0 0 0]);
    % Set direction oriented up in the scene.
    set(hp, 'DiffuseStrength',0.7);
    set(hp, 'SpecularColorReflectance',0.5)
    set(hp, 'SpecularExponent', 30);
    set(hp, 'AmbientStrength', 0.5);
    set(hp, 'EdgeLighting', 'gouraud');

    R = PJ.camera(i).R;
    scale = mean(sqrt(sum(fineY.^2, 2)));
    cam_center = 27 * scale * R(3,:);
    camup(R(2,:));
    campos(cam_center);
    camlight('headlight');
    set(gcf, 'color', 'None');

    if num_img == 1
        save_name = strrep(done_out, 'done.mat', 'final.png');
    else
        save_name = fullfile(output_dir{i}, 'final.png');
    end
    set(gcf, 'Position', [1000 -40 1000 1000]);
    export_fig(hfig, save_name, '-transparent','-opengl', '-native');
end

fprintf('Worked on:\n');
fprintf('\t%s\n', param.output_dir{:});
diary('off');
