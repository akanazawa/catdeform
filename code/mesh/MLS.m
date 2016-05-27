function coeff = MLS(Xi,X,W,ord,tol)
% calculate sparse MLS coefficient matrix coeff
% such that coeff*f is an approximation of f(X) given f(Xi)
% W is size(X,1) x size(Xi,1)
% remove coefficient <tol
%
% EXAMPLE USAGE:
% % prepare data
% Xi = (1:5)';
% X = (0:0.1:6)';
% sigma = 1;
% W = exp(-getEuclideanDistance(X',Xi').^2/sigma^2);
% ord = 2;
% tol = 1e-8;
% % get MLS coefficients
% coeff = MLS(Xi,X,W,ord,tol);
% % randomly sample points
% fi = rand(size(Xi));
% % interpolate
% f = coeff*fi;
% % plot
% figure;
% plot(Xi,fi,'r*');
% hold all
% plot(X,f,'b');

% set order of polynomials for approximation
m = getPolyPwrsTotalOrder(3, ord);
m = permute(m,[3 2 1]); % polynomials in the 3rd dim

% precalculations
Xi_pwr_m = bsxfun(@power,Xi,m);
Xi_pwr_m = reshape(Xi_pwr_m, size(Xi_pwr_m,1),[]);

% get coefficients for each fine point
coeff_cell = cell(size(X,1),1);
% textprogressbar('Calculating MLS coefficients: ');
fprintf('Calculating MLS coefficients: ');
for i = 1:size(X,1)
    % set current fineX and its powers
    curX = X(i,:);
    curX_pwr_m = bsxfun(@power,curX,m);
    curX_pwr_m = reshape(curX_pwr_m, 1, []);
    
    % get current weights
    Wi = diag(sqrt(W(i,:)));
    
    % calculate current coeff
    cur_coeff = curX_pwr_m * pinv(Wi*Xi_pwr_m) * Wi;
    cur_coeff(abs(cur_coeff)<=tol) = 0;
    
    % update coeff matrix
    %coeff(i,:) = sparse(cur_coeff);
    coeff_cell{i} = sparse(cur_coeff);
    
    % update progressbar
    % textprogressbar(round(i/size(X,1)*100));
end
% textprogressbar(' Done.');
fprintf(' Done.\n');

% organize matrix
coeff = sparse(cat(1,coeff_cell{:}));











end

