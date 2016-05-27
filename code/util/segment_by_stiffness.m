function segment_by_stiffness()
%% Super simple segmentation based on normalized cut. 
if ~exist('ncutW', 'file')
    % Change path to your normalize cut. 
    addpath(genpath('/scratch1/projects/Ncut_9'));
end
%% For horse:
% mesh_file = '../data/models/myhorse0.ply';
% stiffness_file = '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontalrun-gallop-graze-grazing-leftjog-lookleft-ote-profile-run-straightleft-white/stiffness.mat';
% tet_file = '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/maxcount_5000_manual_iso2mesh_mls_sig_0.4.mat';
%% For cat:
mesh_file = '../data/models/cat0_nointfaces_centered.off';
stiffness_file = '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontal-jumping-left_walking-licking-rolled-running-sit_curled-sitting_profile-twisting-walking/stiffness.mat';
tet_file = '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/maxcount_3000_manual_iso2mesh_mls_sig_0.4.mat';

[par, fname, ext] = fileparts(stiffness_file);
save_file = fullfile(par, [fname '_segments.mat']);

load(stiffness_file);
[X, Xtri]=readMesh(mesh_file);
t = load(tet_file);
tetX = t.CM_TR.Points;
tetXtri = t.CM_TR.ConnectivityList;
coeff = t.coeff;

% Adjacency graph of tet vertices * distances between vertices..
% This is shahar's triangulation2adjacency.
A_dist = triangulation2adjacency(tetXtri, tetX);

%% Compute distance based on stiffness.
% Stiffness is on every face, transfer it to vertices, by taking mean/max of the stiffness of faces it's connected to.
vert_stiffness = zeros(size(tetX, 1), 1);
vert_stiffness_max = zeros(size(tetX, 1), 1);
for i = 1:size(tetX, 1)
    isface = find(any(tetXtri == i, 2));
    vert_stiffness(i) = mean(stiffness(isface));
    vert_stiffness_max(i) = max(stiffness(isface));
end

sfigure(10); clf; hold on;
patch('Faces',tetXtri,'Vertices', tetX, ...
      'FaceVertexCData', stiffness, ...
      'FaceColor', 'flat', 'EdgeColor', 'b', ...
      'LineStyle', 'none');
axis equal;
cameratoolbar;
set(gcf, 'color', 'w');

% Compute difference between vertex stiffness if there's an edge.
A_stiffness = (A_dist);
A_stiffness_max = (A_dist);
ff = find(A_stiffness);
[ffr,ffc]=ind2sub(size(A_dist),ff);
A_stiffness(ff) = sqrt(sum((vert_stiffness(ffr)-vert_stiffness(ffc)).^2,2));
A_stiffness_max(ff) = sqrt(sum((vert_stiffness_max(ffr)-vert_stiffness_max(ffc)).^2,2));
% Make final affinity
Aff_dist = zeros(size(A_dist));
Aff_stiff = zeros(size(A_dist));
Aff_stiff_max = zeros(size(A_dist));
Aff_dist(ff) = exp(-A_dist(ff) ./ std(A_dist(ff)));
Aff_stiff(ff) = exp(-A_stiffness(ff) ./ std(A_stiffness(ff)));
Aff_stiff_max(ff) = exp(-A_stiffness_max(ff) ./ std(A_stiffness_max(ff)));
Aff = Aff_dist + Aff_stiff;


%% Cluster with NCuts (needs seg#).
num_cluster = 7;
assg_base = ncutW(Aff_dist, num_cluster);
assg = ncutW(Aff_stiff, num_cluster);
clusters = zeros(size(assg, 1), 1);
clusters_base = zeros(size(assg_base, 1), 1);
for i = 1:size(assg, 1)
    clusters(i) = find(assg(i,:));
    clusters_base(i) = find(assg_base(i,:));
end
patch('Faces',tetXtri,'Vertices', tetX, ...
      'FaceVertexCData', clusters_base, ...
      'FaceColor', 'flat', 'EdgeColor', 'b', ...
      'LineStyle', 'none');

% Cluster with Affinity Propagation.
% clusters = apcluster(Aff, 0.01);

% Extend this to fineX by using MLS coeffs.
assg_fine_base = coeff * assg_base;
assg_fine = coeff * assg;
clusters_fine = zeros(size(coeff, 1), 1);
clusters_fine_base = zeros(size(coeff, 1), 1);
for i = 1:size(assg_fine, 1)
    [~, clusters_fine(i)] = max(assg_fine(i,:));
    [~, clusters_fine_base(i)] = max(assg_fine_base(i,:));
end
clusters_fine_face = zeros(size(Xtri, 1), 1);
clusters_fine_face_base = zeros(size(Xtri, 1), 1);
for i = 1:size(Xtri, 1)
    clusters_fine_face(i) = mode(clusters_fine(Xtri(i,:)));
    clusters_fine_face_base(i) = mode(clusters_fine_base(Xtri(i,:)));
end
colorbrew = [141,211,199
             255,255,179
             190,186,218
             251,128,114
             128,177,211
             253,180,98
             179,222,105
             252,205,229];
colorbrew = colorbrew(1:num_cluster,:);
colorbrew = colorbrew./255;
hid = sfigure(100); clf;
hp = patch('Faces',Xtri,'Vertices', X, ...
      'FaceVertexCData', clusters_fine_face, ...
      'FaceColor', 'flat', 'EdgeColor', 'none');
colorbrew = colorbrew(randperm(num_cluster),:);
colormap(colorbrew);
axis image off;
lighting gouraud;
cameratoolbar;
set(hp, 'DiffuseStrength',0.7);
set(hp, 'SpecularColorReflectance',0.4)
set(hp, 'SpecularExponent', 20);
set(hp, 'AmbientStrength', 0.5);
keyboard
% horse:
campos([26.8806  -15.8592    7.5040]);
%cat: 
% campos([24.3112  -28.8581   11.8694]);
camlight('headlight');
set(gcf, 'color', 'none');
set(gcf, 'Position', [1000 -40 1000 1000]);
% out_name = '../figures/raw/cat_seg.png';
out_name = '../figures/raw/horse_seg.png';
export_fig(out_name, '-transparent', '-opengl', '-native', '-a4');

hid_base = sfigure(300); clf;
colorbrew = colorbrew(randperm(size(colorbrew,1)),:);
colormap(colorbrew);
axis image off;
cameratoolbar;
set(hp, 'DiffuseStrength',0.7);
set(hp, 'SpecularColorReflectance',0.4)
set(hp, 'SpecularExponent', 20);
set(hp, 'AmbientStrength', 0.5);
campos([26.8806  -15.8592    7.5040]);
camlight('headlight');
set(gcf, 'color', 'none');
set(gcf, 'Position', [1000 -40 1000 1000]);
keyboard
out_name = '../figures/raw/horse_seg_baseline.png';
export_fig(out_name, '-transparent', '-opengl', '-native', '-a4');


sfigure(200); clf;
patch('Faces',Xtri,'Vertices', X, ...
      'FaceVertexCData', clusters_fine, ...
      'FaceColor', 'flat', 'EdgeColor', 'none', ...
      'LineStyle', '-');
axis equal;
cameratoolbar;
set(gcf, 'color', 'w');
keyboard
