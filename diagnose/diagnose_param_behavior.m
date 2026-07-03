modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('DefaultParameterBehavior: %s\n', get_param(mdl, 'DefaultParameterBehavior'));

fprintf('\n=== attitude_kp in base workspace? ===\n');
fprintf('exist(''attitude_kp'',''var'') in base = %d\n', evalin('base', "exist('attitude_kp','var')"));

fprintf('\n=== Model Workspace variables ===\n');
mw = get_param(mdl, 'ModelWorkspace');
try
    wsVars = whos(mw);
    for i = 1:numel(wsVars)
        fprintf('  %s\n', wsVars(i).name);
    end
catch e
    fprintf('ModelWorkspace read error: %s\n', e.message);
end

fprintf('\n=== hasVariable attitude_kp in Model Workspace ===\n');
try
    fprintf('  hasVariable = %d\n', mw.hasVariable('attitude_kp'));
    if mw.hasVariable('attitude_kp')
        fprintf('  value = %g\n', mw.getVariable('attitude_kp'));
    end
catch e
    fprintf('  error: %s\n', e.message);
end
