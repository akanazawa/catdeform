function [R, t, scale, error, S1_hat] = compute_similarity(S1, S2)
%%%%%%%%%%%%%%%%%%%%
% Computes a similarity transform (sR, t) that takes a set of 3D points S1 (3 x N)
% closest to a set of 3D points S2, where R is an 3x3 rotation matrix, t 3x1
% translation, s scale. i.e. the orthogonal Procrutes problem.
% Nice derivation at:
% http://graphics.stanford.edu/courses/cs205a-13-fall/assets/notes/chapter6.pdf
%
% Angjoo Kanazawa 10/29/2014
%%%%%%%%%%%%%%%%%%%%
% 1. Remove mean.
mu1 = mean(S1, 2);
mu2 = mean(S2, 2);
X1 = bsxfun(@minus, S1, mu1);
X2 = bsxfun(@minus, S2, mu2);

N = size(X2, 2);
% 2. Compute variance of X1 used for scale.
var1 = sum(sum(X1.^2));
% var1 = mean(sqrt(sum(X1.^2)));

% 3. The outer product of X1 and X2.
K = X1*X2';
% 4. Solution that Maximizes trace(R'K) is R=U*V', where U, V are
% singular vectors of K.
[U, Sigma, V] = svd(K);
% Construct Z that fixes the orientation of R to get det(R)=1.
Z = eye(size(U, 1));
Z(end) = det(U*V');
% Construct R.
R = V * Z * U';

% 5. Recover scale.
scale = trace(R*K)  / var1;

% 6. Recover translation.
t = mu2 - scale.*(R * mu1);

% 7. Error:
S1_hat = inhomo([scale.*R t; zeros(1,size(R,1)) 1] * homo(S1));
error = norm(S2 - S1_hat, 'fro') ./ ...
        norm(S2, 'fro');
