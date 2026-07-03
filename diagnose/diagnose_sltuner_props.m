modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

io(1) = linio([mdl '/From'], 1, 'in');
io(2) = linio([mdl '/Quadcopter'], 1, 'out');

tunedBlocks = {
    'quadcopter_package_delivery/Maneuver Controller/Position Control/PID Controller'
};

ST = slTuner(mdl, tunedBlocks, io);
disp(properties(ST));
fprintf('\n=== ST.Options ===\n');
disp(ST.Options);
disp(properties(ST.Options));
