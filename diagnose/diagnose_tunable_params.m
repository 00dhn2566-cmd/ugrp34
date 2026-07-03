modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = 'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control/Control Pitch';
fprintf('MaskType: %s\n', get_param(blk, 'MaskType'));
fprintf('Controller: %s\n', get_param(blk, 'Controller'));
try
    fprintf('P: %s\n', get_param(blk, 'P'));
    fprintf('I: %s\n', get_param(blk, 'I'));
    fprintf('D: %s\n', get_param(blk, 'D'));
catch e
    fprintf('P/I/D read error: %s\n', e.message);
end

io(1) = linio([mdl '/Scope/Demux'], 1, 'in');
io(2) = linio([mdl '/Scope/In Bus Element'], 1, 'out');
tunedBlocks = {blk};
ST = slTuner(mdl, tunedBlocks, io);
ST.Options.RateConversionOptions.Method = 'tustin';

fprintf('\n=== showTunable(ST, blk) ===\n');
try
    showTunable(ST, blk);
catch e
    fprintf('showTunable error: %s\n', e.message);
end

fprintf('\n=== attitude_kp/ki/kd 실제 값 ===\n');
fprintf('  attitude_kp = %g\n', attitude_kp);
fprintf('  attitude_ki = %g\n', attitude_ki);
fprintf('  attitude_kd = %g\n', attitude_kd);

fprintf('\n=== 실제 systune 테스트 (더미 목표, Control Pitch 1개만) ===\n');
pts = getPoints(ST);
Req = TuningGoal.Tracking(pts{1}, pts{2}, 1, 0, 1);
opt = systuneOptions('Display', 'off');
try
    [ST_tuned, fSoft] = systune(ST, Req, opt);
    fprintf('systune 성공, fSoft=%g\n', fSoft);
catch e
    fprintf('systune error: %s\n', e.message);
end
