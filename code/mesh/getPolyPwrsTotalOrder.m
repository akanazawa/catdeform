function m = getPolyPwrsTotalOrder(d, n)
% get all polynomial powers of dimension d up to order n.
% returns a dx? array of powers

m = cell(d,1);
[m{:}] = ndgrid(0:n);
m = cellfun(@(x) reshape(x,[],1), m, 'UniformOutput', false);
m = cat(2,m{:});
ff = sum(m,2)<=n;
m = m(ff,:);

end

