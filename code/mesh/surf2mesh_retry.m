function [node,elem,face]=surf2mesh_retry(v,f,p0,p1,keepratio,maxvol,maxtetcount,maxretries,repair_mesh)

fprintf('==================================================================================\n');

% clear temporary folder
delete(mwpath('*.*')); 
keepratio_orig = keepratio;

if nargin < 9
    repair_mesh = 0;
end

if repair_mesh
    [v, f] = meshcheckrepair(v, f, 'meshfix');
end
% retry meshing
retry = true;
numretries = 0;
while retry
    numretries = numretries+1;
    if numretries>maxretries,
        error('maximal number of retries exceeded');
    end;
    try
        % region_list = [-0.4317 -1.168 -1.592; 0.355 -1.246 -1.54; -0.421 1.206 -1.525 ; 0.421 1.206 -1.525];
        [node,elem,face]=surf2mesh(v,f,p0,p1,keepratio,maxvol);
    catch
        keepratio = keepratio+0.001;
        continue;
    end;
    retry = false;
    fprintf('keepratio=%g worked (%g requested)\n', keepratio, keepratio_orig)
end
if ~exist('node','var')
    error('tetmesh generation failed');
end;
if size(elem,1)>maxtetcount
    error(sprintf('tet count (%d) exceeds limit (%d)',size(elem,1),maxtetcount))
end;

fprintf('==================================================================================\n\n');
