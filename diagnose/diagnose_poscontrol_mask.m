modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = [mdl '/Maneuver Controller/Position Control'];
fprintf('=== %s ===\n', blk);
fprintf('Mask: %s\n', get_param(blk, 'Mask'));
if strcmp(get_param(blk, 'Mask'), 'on')
    fprintf('MaskNames: %s\n', strjoin(get_param(blk,'MaskNames'), ', '));
    fprintf('MaskValues: %s\n', strjoin(get_param(blk,'MaskValues'), ', '));
end

pidBlk = [blk '/PID Controller'];
fprintf('\nPID Controller P/I/D:\n');
fprintf('  P: %s\n', get_param(pidBlk, 'P'));
fprintf('  I: %s\n', get_param(pidBlk, 'I'));
fprintf('  D: %s\n', get_param(pidBlk, 'D'));

ph = get_param(blk, 'PortHandles');
fprintf('\n%s ports: In=%d Out=%d\n', blk, numel(ph.Inport), numel(ph.Outport));
