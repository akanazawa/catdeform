function plot_cameras(model, fig_id)
%%%%%%%%%%%%%%%%%%%%
% Plot shape and draw affine cameras around it.
% Angjoo Kanazawa
%%%%%%%%%%%%%%%%%%%%
if nargin < 2
    fig_id = sfigure;
else
    sfigure(fig_id);
end
S = model.S;
S = bsxfun(@minus, S, mean(S, 2));
scatter3(S(1,:), S(2,:), S(3,:), ...
         'b', 'filled');
hold on;
% World origin.
plot3(0, 0, 0, 'mo', 'MarkerSize', 8);

scale = mean(sqrt(sum(bsxfun(@minus, S, mean(S, 2)).^2)));

hasK = isfield(model, 'R') && isfield(model, 'K');

if ~hasK
    num_cameras = size(model.M, 1)/2;
else
    num_cameras = size(model.R, 3);
end
% isAffine = size(model.t, 1) == 2 || size(model.t, 1) == num_cameras*2;
radius = 10; % All cameras will be plotted on this radius sphere.
face_color = linspecer(num_cameras);
for i = 1:num_cameras
    % 4 corners of image plane.
    if ~hasK % No 3x3xN rotation matrix.
        row_here = 2*i - 1:2*i;
        R = rotationMatrix(model.M(row_here, :));
        t = model.t(row_here);
    else
        R = model.R(:, :, i);
        t = model.t(1:2, i);
        % cam_plane(3,:) = sign(model.t(end,i)).*cam_plane(3,:);
    end
    cam_center = radius * scale * R(3,:);
    cam_axis = scale * [R(1:2,:); -R(3,:)];
    w_axes = bsxfun(@plus, cam_axis, cam_center);
    plot3(cam_center(1), cam_center(2), cam_center(3), 'k*', ...
          'MarkerSize', 10);
    % Draw the camera axis.
    line([cam_center(1), w_axes(1,1)], ...
         [cam_center(2), w_axes(1,2)], ...
         [cam_center(3), w_axes(1,3)], 'Color', 'r'); %x
    line([cam_center(1), w_axes(2,1)], ...
         [cam_center(2), w_axes(2,2)], ...
         [cam_center(3), w_axes(2,3)], 'Color', 'g'); %y
    line([cam_center(1), w_axes(3,1)], ...
         [cam_center(2), w_axes(3,2)], ...
         [cam_center(3), w_axes(3,3)], 'Color', 'b'); %z

    % Draw the principal plane.
    w_pplane = bsxfun(@plus,  [1 1 1; 1 -1 1; -1 -1 1; -1 1 1] * cam_axis, cam_center);
    patch('vertices', w_pplane, 'faces', [1 2 3 4], 'FaceColor', ...
          'flat', 'FaceVertexCData', face_color(i,:));
    patch('vertices', [w_pplane; cam_center], 'faces', [1 2 5; 2 3 5; 3 4 5; 4 1 5], 'FaceColor', ...
          'flat', 'FaceVertexCData', [0.7 0.7 0.7]);
end
axis image;
set(gcf, 'color', 'w');
cameratoolbar;
