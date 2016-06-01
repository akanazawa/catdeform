function initialize

% set paths
if isempty(whos('global','path_def')),
    fprintf('- Adding toolbox paths\n');
    
    % common path
    % Newer & faster version.
    addpath(genpath(['../toolboxes/YALMIP-R20150919']));
    % Others
    addpath(genpath(['../toolboxes/toolbox_graph']));
    addpath(genpath(['../toolboxes/iso2mesh']));
    addpath(genpath(['../toolboxes/textprogressbar']));
    addpath(genpath(['../toolboxes/addaxis']));
    
    % for projection matrix computation
    addpath(genpath(['../toolboxes/sba_1.5']));
    addpath(genpath(['../toolboxes/vincent_sfm']));

    % Other utils
    addpath(genpath(['../toolboxes/misc']));
    addpath(genpath(['../toolboxes/export_fig']));

    addpath('mesh');
    addpath(genpath('util'));
    addpath('param');
    
    %% !!! Specify your mosek path here:
    MosekPath = '~/customModules/pkg_src/mosek/7/toolbox/r2013a'; 
    addpath(genpath(MosekPath));

    
    global path_def
end;
