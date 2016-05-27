function [new_model] = refine_with_sba_affine(points, model, ...
                                              refinement_type, isCalibrated)
%%%%%%%%%%%%%%%%%%%%
% Refines 3D model {M, t, S} using SBA & SBA affine projection
% files from the excellent Vincent's SfM toolbox.
% isCalibrated = 1 will assume internal calibration matrix is
% eye(3), i.e. the orthographic projection assumption.
% isCalibrated = 0 will optimize for alpha_x, alpha_y, i.e. weak
% perspective projection.
%
% Angjoo Kanazawa
%%%%%%%%%%%%%%%%%%%%

% Parameterization for each extrinsic camera params:
% 7 x 1: 4 for quaternion, 3 for translation.
% quaternions have the scalar part as the first element i.e. a rotation by angle TH
% around the unit vector U=(Ux, Uy, Uz) should be specified as
% cos(TH/2) Ux*sin(TH/2) Uy*sin(TH/2) Uz*sin(TH/2).

% Parameterization for 3D Shape:
% X Y Z NFrames(total number of images its visible) FRAME0 x0 y0
% FRAME1 x1 y1 etc..

% Parameterization for each intrinsic camera params:
% 5 x 1: focalX, focalY, ppoint x, ppoint y, aspect ratio, skew factor
if nargin < 4
    isCalibrated = true;
end
% Refiment type: 'motstr' for motion & structure, 'mot' motion
% only, 'str' for structure only (this option is only for the rigid case)
if nargin < 3
    refinement_type = 'motstr';
end

num_points = size(model.S, 2);
num_fixed_points = 0;
num_images = size(model.M, 1) ./ 2;
num_fixed_images = 0;
% num_points by num_images matrix nonzero (i,j) indicates point i
% is visible in image j.
visibility_mask = permute(points(3, :, :), [2 3 1]);
% vector of doubles holding the measurements (i.e., image
% projections) laid out as (x_11, .. x_1m, ..., x_n1, .. x_nm),
% where x_ij is the projection of the i-th point on the j-th image.
% If point i is not visible in image j, x_ij is missing.
points_2NP = permute(points(1:2, :, :), [1 3 2]);
measurements = vec(points_2NP(:, ~~visibility_mask'));
num_measurement_params = 2;
num_shape_params = 3;

% Separate into K and R, as 3 x 3 x N and get quaternion
% representation for R.
K_all0 = zeros(3, num_images);
R0 = zeros(3, 3, num_images);
t0 = zeros(2, num_images);
for i = 1:num_images
    row_here = 2*(i-1) + 1:2*i;
    r1 = model.M(row_here(1),:); r2 = model.M(row_here(2),:);
    if ~isCalibrated
        % Or right after scaled orthographic SfM this is just:
        K_all0(:, i) = [norm(r1), 0, norm(r2)]';
        % Apply K_inv to obtrain R, t.
        t0(:, i) = model.t(row_here) ./ [norm(r1); norm(r2)];
        r1 = r1 ./ norm(r1);
        r2 = r2 ./ norm(r2);
        R0(:, :, i) = [r1; r2; cross(r1, r2)];
    else
        R0(:, :, i) = [r1; r2; cross(r1, r2)];
        t0(:, i) = model.t(row_here);
    end
end
Q0 = quaternion(R0);

% fixedK specifies which of the 3 internal calibraionparameters are FIXED.
if isCalibrated
    num_cam_params = 6; % 4 rotation 2 translation only.
    fixedK = ~~ones(3, 1);
else
    % fixedK = ~~zeros(3, 1); % or [0 1 0];
    fixedK = [0 1 0]; % or [0 1 0];
    num_cam_params = 6 + sum(~fixedK); % 4 rotation 2 translation, 3 internal
                           % (focalX, skew, focalY)
end
K0 = K_all0(~fixedK, :);
% Initial parameter estimates from (a1, ..., am, b1, ..., bn),
% aj, bi being the j-th image and i-th point parameters, respectively
P0 = [vec([Q0; t0; K0]); model.S(:)]';


% Name of function that computes the projection & its Jacobian.
% The form XX@YY, refers to a function named XX loaded from the shared
% library YY (i.e., a .so file in Un*x, a .dll in Windows).
projection_so = '@../toolboxes/vincent_sfm/sbaProjection.so';
if isCalibrated
    projection_fun = 'affinetr3k1k2k3k4k5kap1kap2pp1pp2Ignored';
else
    projection_fun = 'affinetr3k4k5kap1kap2pp1pp2Ignored';
end
projection_path  = [projection_fun projection_so];
if strcmp(refinement_type, 'motstr')
    projection_jacobian_path  = [projection_fun 'Jac' ...
                        projection_so];
elseif strcmp(refinement_type, 'mot')
    projection_jacobian_path  = [projection_fun 'JacRT' ...
                        projection_so];
elseif strcmp(refinement_type, 'str')
    projection_jacobian_path  = '';
    refinement_type = 'mot';
end
    
max_iter = 500;
verbose = 0;
opts = [];
% fprintf('\nstart sba..\r');

[ ret P info ] = sba(num_points, num_fixed_points, num_images, ...
                     num_fixed_images, visibility_mask, ...
                     P0, num_cam_params, num_shape_params, measurements, ...
                     num_measurement_params, projection_path, ...
                     projection_jacobian_path, max_iter, verbose, ...
                     opts, refinement_type,...
                     K_all0, fixedK);
if (ret == -1)
    new_model = model;
else
    % Unravel P.
    motion = reshape(P(1:num_cam_params * num_images), num_cam_params, num_images);
    rotation = quaternion(motion(1:4, :));
    if ~isCalibrated
        K_all(~fixedK, :) = motion(7:end, :);
        new_model.K = K_all;
        new_model.R = rotation;
        new_model.t = motion(5:6, :);
    else
        % make into 2N x 3
        new_model.M = reshape(permute(rotation(1:2, :, :), [1 3 2]), [], 3);
        new_model.t = vec(motion(5:6, :));
    end
    new_model.S = reshape(P((num_cam_params * num_images) + 1:end), 3, num_points);
end

