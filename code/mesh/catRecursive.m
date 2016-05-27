function [ F ] = catRecursive( varargin )

if length(varargin)<=1,
    F = varargin{:};
else
    t = floor(length(varargin)/2);
    F = catRecursive(varargin{1:t}) + catRecursive(varargin{t+1:end});
end;

end

