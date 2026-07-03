modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

names = {'kp_attitude','ki_attitude','kd_attitude', ...
         'kp_yaw','ki_yaw','kd_yaw', ...
         'kp_altitude','ki_altitude','kd_altitude'};
for i = 1:numel(names)
    if exist(names{i}, 'var')
        fprintf('%s = %g\n', names{i}, eval(names{i}));
    else
        fprintf('%s: NOT DEFINED\n', names{i});
    end
end

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('\n=== slTuner with variable names directly as tunedBlocks ===\n');
io(1) = linio([mdl '/Scope/Demux'], 1, 'in');
io(2) = linio([mdl '/Scope/Demux'], 2, 'in');
io(3) = linio([mdl '/Scope/Demux'], 3, 'in');
io(4) = linio([mdl '/Scope/In Bus Element'],  1, 'out');
io(5) = linio([mdl '/Scope/In Bus Element1'], 1, 'out');
io(6) = linio([mdl '/Scope/In Bus Element2'], 1, 'out');

tunedBlocks = {
    'quadcopter_package_delivery/Maneuver Controller/Position Control/PID Controller'
    'kp_attitude'
    'ki_attitude'
    'kd_attitude'
    'kp_yaw'
    'ki_yaw'
    'kd_yaw'
    'kp_altitude'
    'ki_altitude'
    'kd_altitude'
};
try
    ST = slTuner(mdl, tunedBlocks, io);
    ST.Options.RateConversionOptions.Method = 'tustin';
    pts = getPoints(ST);
    disp(pts);
    refNames = pts(1:3);
    actNames = pts(4:6);
    Req = TuningGoal.Tracking(refNames, actNames, 5, 0, 1);
    opt = systuneOptions('Display', 'off');
    [ST_tuned, fSoft] = systune(ST, Req, opt);
    fprintf('SUCCESS, fSoft=%g\n', fSoft);
catch e
    fprintf('FAILED: %s\n', getReport(e, 'basic'));
end
