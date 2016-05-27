function [x_new, mu_scaled, mu, scale] = normalize_points(x)
% Make 2D/3D points centered at origin and average distance from the
% mean to be sqrt(2).
% Return the original center (scaled) as well.
% x = (2|3|4) x N;
% Angjoo Kanazawa
is_homo = 0;
if all(x(end,:) == 1)
    x = x(1:end-1, :);
    is_homo = 1;
end

if size(x, 1) == 3 % 3D points
    scale_factor = sqrt(3);
else
    scale_factor = sqrt(2);
end

mu = mean(x, 2);
x_new = bsxfun(@minus, x, mu);
dist = sqrt(sum(x_new.^2));
scale = scale_factor ./ mean(dist(:)); % average distance from origin is
                                  % sqrt(2).
x_new = x_new .* scale;
mu_scaled = mu .* scale;

if is_homo
    x_new = [x_new; ones(1, size(x_new, 2))];
    mu_scaled = [mu_scaled; 0];
end
