function [M, t, error, new_modelwp, reproj, proj] = get_projection(w,X,prev_model,recompute)
%%%%%%%%%%%%%%%%%%%%
% Computes projection matrix that matches the 2D observation and the
% 3D points for one image.
% Inputs:
% W - 2 x P, 2D observations of P points.
% S - 3 x P, the 3D coordinates of the P points.
% prev_model - (optional) if provided, uses previous camera model as initialization.
% Output:
% M - 2 x 3, motion matrix
% t - 2 x 1, translation
% projection - 3 x 4, projection matrix
% error - reprojection error with the computed camera.
%
% Initialize with the linear method for Affine camera.
% DLT HZ Ch7.3 pg 185
% + Then solve for camera matrices only using SBA.
% Angjoo Kanazawa
%%%%%%%%%%%%%%%%%%%%
% Last row encodes visibility.
points = [w; ones(1, size(w, 2))];
if nargin == 2
    % Get linear solution as initialization
    M = DLT(w, X);
    linear_model.S = X;
    linear_model.M = M(1:2, 1:3);
    linear_model.t = M(1:2, end);
    % Call SBA.
    % Last arg is the camera type, false = weak perspective, true =
    % orthographic proj.
    new_modelwp = refine_with_sba_affine(points, linear_model, ...
                                         'mot', false);
else
    linear_model.S = X;
    linear_model.R = prev_model.R;
    linear_model.M = prev_model.M;
    linear_model.t = prev_model.t;
    %% Compute t and s without SBA.
    rotated_pts = prev_model.R * X;
    Ktmp = size(w, 2);
    res = [repmat(eye(2), size(w,2),1) vec(rotated_pts(1:2,:))]...
          \w(:);
    linear_model.t = [res(1) ;res(2)];
    linear_model.K = [res(3), 1, res(3)];
    if nargin == 4 && recompute == 1
        new_modelwp = refine_with_sba_affine(points, linear_model, ...
                                             'mot', false);
    else
        new_modelwp = linear_model;
    end
end

% For orthographic
% projection = [new_model.M new_model.t; 0 0 0 1];
% For weak-perspective 
K = diag([new_modelwp.K(1), new_modelwp.K(3), 1]);
new_modelwp.K = K;
ext_wp = [new_modelwp.R(1:2,:) new_modelwp.t; 0 0 0 1];
projection_wp = K*ext_wp;

% plot_cameras(new_modelwp, 21);
% cameratoolbar;

% reprojection error:
reproj_wp =  inhomo(projection_wp * homo(X));
error_wp = norm(w - reproj_wp);
error = error_wp;
M = K(1:2,1:2) * new_modelwp.R(1:2,:);
t = K(1:2, 1:2) * new_modelwp.t;
reproj = reproj_wp;
proj = projection_wp;

function M = DLT(w, X)
% Normalize the 3D coordinates.
[X_normalized, ~, mu, scale] = normalize_points(X);
U = [scale 0 0 -scale * mu(1);
     0 scale 0 -scale * mu(2);
     0 0 scale -scale * mu(3);
     0 0 0 1];
% Compute 2*N x 8 equation derived from
% x = P*X ==> x cross P*X = 0.
A = zeros(2*size(X, 2), 8);
for i = 1:size(X, 2)
    A(2*i-1:2*i, :) = [zeros(1,4) -X_normalized(:, i)' 1;
                       X_normalized(:, i)' 1 zeros(1, 4)];
end

% Normalize 2D coordinates and remember the transformation.
[w_norm, ~, mu, scale] = normalize_points(w);
% b is (-y_i, x_i)
b = reshape([-w_norm(2,:)' w_norm(1,:)']', 2 * size(w, 2), []);
T = [scale 0 -scale * mu(1);
     0 scale -scale * mu(2);
     0 0 1];
% Catch rank-deficient warning.
warn = warning('error', 'MATLAB:rankDeficientMatrix');
try
    p = A\b;
catch err
    % A is rank-deficient,,,.
    fprintf('\t\t@ get_projection: %s\n', err.message);
    keyboard
end

R = [p(1:3)'; p(5:end-1)'];
% Convert to closest rotation matrix. 
R = rotationMatrix(rotationMatrix(R));
R = R(1:2, :);
% Compute t.
t = mean(w_norm - R*X_normalized, 2);
% Put R and t together into 3x4 + denormalize.
M = inv(T) * [R t; 0 0 0 1] * U;
% Restore warning to non-error.
warning(warn);

