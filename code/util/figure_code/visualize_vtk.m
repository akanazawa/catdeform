function visualize_vtk(vtk_path)

[coarseX,coarseTri,misc] = read_vtk_more(vtk_path);
[fineY,fineTri,fineMisc] = read_vtk_more(vtk_path);
fineY = bsxfun(@minus, fineY, mean(fineY, 2));
blue = [156 199 234]./255;
color = repmat(blue, size(fineY,2), 1);
scale = mean(sqrt(sum(bsxfun(@minus, fineY, mean(fineY, ...
                                                 2)).^2)));
%% Compute cameras

%% Plot surface
sfigure(1); clf; hold on;
hp = patch('Faces',fineTri','Vertices', fineY', 'FaceColor', 'interp', 'FaceVertexCData', ...
           color, 'EdgeColor', 'blue');
axis image off;
lighting gouraud;
cameratoolbar;
camtarget([0 0 0]);
% Set direction oriented up in the scene.
set(hp, 'DiffuseStrength',0.7);
set(hp, 'SpecularColorReflectance',0.5)
set(hp, 'SpecularExponent', 30);
set(hp, 'AmbientStrength', 0.5);
set(hp, 'EdgeLighting', 'gouraud');
camup(camera_here.R(2,:));
campos(cam_center);
camlight('headlight');
% camlight('headlight', 'infinite');
% material shiny;
set(gcf, 'color', 'none');
% set(gcf, 'Position', [1000 -40 1000 1000]);
% export_fig([img_out_name,'.png'], '-transparent', '-opengl', ['-' ...
%                     'native']);
