function assign_symmetry2tet()

%% For horse:
% mesh_file = '../data/models/myhorse0.ply';
% tet_file = '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/maxcount_5000_manual_iso2mesh_mls_sig_0.4.mat';
% axis_path = ['../data/models/cat0_nointfaces_centered_symaxis.mat'];
% save_path = '../data/models/symmetric_stiffness_id_myhorse0_v305f606-tetface_500_maxvol_1_maxcount_5000_manual_iso2mesh_mls_sig_0.4.mat';
%% For cat:
mesh_file = '../data/models/cat0_nointfaces_centered.off';
tet_file = '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/maxcount_3000_manual_iso2mesh_mls_sig_0.4.mat';
axis_path = ['../data/models/cat0_nointfaces_centered_symaxis.mat'];
save_path = '../data/models/symmetric_stiffness_id_cat_nomouth_v240f476-tetface_380_maxvol_1_maxcount_3000_manual_iso2mesh_mls_sig_0.4.mat';
t = load(axis_path);
plane_normal = t.annotation.plane_normal;

[X, Xtri]=readMesh(mesh_file);
t = load(tet_file);
tetX = t.CM_TR.Points;
tetXtri = t.CM_TR.ConnectivityList;

%% For every face, figure out the centroid
tet_centers = zeros(size(tetXtri, 1), 3);
for i = 1:size(tetXtri, 1)
    tet_centers(i, :) = mean(tetX(tetXtri(i,:), :));
end

%% Mirror the centroids wrt plane.
proj = (tet_centers * plane_normal(1:3) + plane_normal(4)) / sum(plane_normal(1:3).^2);
ref_centers = tet_centers - bsxfun(@times, 2 * proj,  plane_normal(1:3)');

% Find the closest flipped centroid if it maps to itself, it gets its own stiffness. OW they share stiffness.
sym_id = zeros(size(tetXtri, 1), 2);
for i = 1:size(tetXtri, 1)
    [minpt, min_id] = min(sum(bsxfun(@minus, ref_centers(i,:), tet_centers).^2, 2));
    sym_id(i, :) = [i, min_id];
end

%% Identify those tets that the plane goes through.
special = sym_id(:, 1) == sym_id(:, 2);
face_side = zeros(size(tetXtri, 1), 1);
face_side(special) = 1;

% Find which side the centroids land in.
y = tet_centers * plane_normal(1:3) + plane_normal(4); 
side = y;
side(y < 0) = -1;
side(y > 0) = 1;
side(special) = 0;
sum(side == -1)
%% Assign unique ID to the side with smaller # of centroids.
[~, min_id] = min([sum(side == -1)  sum(side == 1)]);
if min_id == 1
    keep_these = side == -1 | side == 0;
else
    keep_these= side == 1 | side == 0;
end

unique_tet_ids = 1:sum(keep_these);
stiffness_id = -ones(size(tetXtri, 1), 1);
stiffness_id(keep_these) = unique_tet_ids;

% Now find the assignment of the side that's not kept.
stiffness_id(~keep_these) = stiffness_id(sym_id(~keep_these,2));

save(save_path, 'stiffness_id');

assert(sum(stiffness_id == -1) == 0);
%%Visualize it.
unique_color_id = 1:(sum(keep_these & side ~=0)+1);
color_id = -ones(size(tetXtri, 1), 1);
color_id(keep_these & side ~= 0) = unique_color_id(1:end-1);
color_id(side == 0) = unique_color_id(end);
color_id(~keep_these) = color_id(sym_id(~keep_these,2));
cmap = distinguishable_colors(length(unique_color_id));
sfigure(1); clf; hold on;
subplot(121);
% patch('Faces',tetXtri,'Vertices', tetX, ...
%       'FaceVertexCData', color_id,...
%       'FaceColor', 'flat', 'EdgeColor', 'b', ...
%       'LineStyle', '-', 'Marker', '.', 'MarkerSize', ...
%       5); 
tetramesh(tetXtri, tetX, color_id);
colormap(cmap);
axis image;
cameratoolbar;
subplot(122);
tetramesh(tetXtri, tetX, color_id);
% patch('Faces',tetXtri,'Vertices', tetX, ...
%       'FaceVertexCData', color_id,...
%       'FaceColor', 'flat', 'EdgeColor', 'b', ...
%       'LineStyle', '-', 'Marker', '.', 'MarkerSize', ...
%       5); 
colormap(cmap);
axis image;
cameratoolbar;
keyboard

