modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
ph = get_param(blk, 'PortHandles');
fprintf('=== Altitude and YPR Control ports ===\n');
for i = 1:numel(ph.Inport)
    fprintf('  In%d name="%s"\n', i, get_param(ph.Inport(i), 'Name'));
end
for i = 1:numel(ph.Outport)
    fprintf('  Out%d name="%s"\n', i, get_param(ph.Outport(i), 'Name'));
end

blk2 = [mdl '/Maneuver Controller/Position Control/PID Controller'];
ph2 = get_param(blk2, 'PortHandles');
fprintf('\n=== Position Control/PID Controller ports ===\n');
for i = 1:numel(ph2.Inport)
    fprintf('  In%d name="%s"\n', i, get_param(ph2.Inport(i), 'Name'));
end
for i = 1:numel(ph2.Outport)
    fprintf('  Out%d name="%s"\n', i, get_param(ph2.Outport(i), 'Name'));
end

fprintf('\n=== Test: slTuner with ONLY Altitude and YPR Control ===\n');
io(1) = linio([mdl '/Scope/Demux'], 1, 'in');
io(2) = linio([mdl '/Scope/In Bus Element'], 1, 'out');
try
    ST1 = slTuner(mdl, {blk}, io);
    ST1.Options.RateConversionOptions.Method = 'tustin';
    pts = getPoints(ST1);
    Req = TuningGoal.Tracking(pts{1}, pts{2}, 1, 0, 1);
    opt = systuneOptions('Display', 'off');
    [~, fSoft] = systune(ST1, Req, opt);
    fprintf('OK, fSoft=%g\n', fSoft);
catch e
    fprintf('FAILED: %s\n', e.message);
end
