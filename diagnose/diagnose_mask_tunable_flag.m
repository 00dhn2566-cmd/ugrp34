modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blocks = {
    [mdl '/Maneuver Controller/Position Control']
    [mdl '/Maneuver Controller/Altitude and  YPR Control']
};

for i = 1:numel(blocks)
    blk = blocks{i};
    fprintf('=== %s ===\n', blk);
    names = get_param(blk, 'MaskNames');
    tunable = get_param(blk, 'MaskTunableValues');
    for j = 1:numel(names)
        fprintf('  %s : Tunable=%s\n', names{j}, tunable{j});
    end
    fprintf('\n');
end
