function save_stiffness_horse()

stiffness_file = '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness_0922/frontalrun-gallop-graze-grazing-leftjog-lookleft-ote-profile-run-straightleft-white/stiffness.mat';
load(stiffness_file);

%% Template tet.
tet_file = '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/maxcount_5000_manual_iso2mesh_mls_sig_0.4.mat';
t = load(tet_file);
tetX = t.CM_TR.Points;
tetXtri = t.CM_TR.ConnectivityList;

%% Deformed tet:
% vtk_file = '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness_0922/frontalrun-gallop-graze-grazing-leftjog-lookleft-ote-profile-run-straightleft-white/white/out_control_(1008).vtk';
% [tetX,tetXtri,misc] = read_vtk_more(vtk_file);
% tetX = tetX'; tetXtri = tetXtri';

tet_centers = zeros(size(tetXtri, 1), 3);
for i = 1:size(tetXtri, 1)
    tet_centers(i, :) = mean(tetX(tetXtri(i,:), :));
end

hid =sfigure(); clf; hold on;
scatter3(tet_centers(:, 1), tet_centers(:, 2), tet_centers(:, 3), 25, log(stiffness+0.01), 'filled')
cmap = jet(256);
axis image; cameratoolbar; axis off;
set(gcf, 'color', 'none');
keyboard
out_name = '../figures/raw/cat_stiffness.png';
export_fig(hid, out_name, '-transparent', '-painters', '-a4', '-native');










