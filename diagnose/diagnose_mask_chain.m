modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

chain = {
    'quadcopter_package_delivery/Maneuver Controller'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control/Control Pitch'
};

for i = 1:numel(chain)
    blk = chain{i};
    fprintf('=== %s ===\n', blk);
    fprintf('Mask: %s\n', get_param(blk, 'Mask'));
    if strcmp(get_param(blk,'Mask'), 'on')
        fprintf('MaskInitialization:\n%s\n', get_param(blk, 'MaskInitialization'));
        fprintf('MaskNames: %s\n', strjoin(get_param(blk,'MaskNames'), ', '));
        fprintf('MaskValues: %s\n', strjoin(get_param(blk,'MaskValues'), ', '));
    end
    fprintf('\n');
end
