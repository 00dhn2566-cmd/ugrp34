modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

tunedBlocks = {
    'quadcopter_package_delivery/Maneuver Controller/Position Control/PID Controller'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control'
};

io(1) = linio([mdl '/Scope/Demux'], 1, 'in');
io(2) = linio([mdl '/Scope/Demux'], 2, 'in');
io(3) = linio([mdl '/Scope/Demux'], 3, 'in');
io(4) = linio([mdl '/Scope/In Bus Element'],  1, 'out');
io(5) = linio([mdl '/Scope/In Bus Element1'], 1, 'out');
io(6) = linio([mdl '/Scope/In Bus Element2'], 1, 'out');

ST = slTuner(mdl, tunedBlocks, io);
ST.Options.RateConversionOptions.Method = 'tustin';

pts = getPoints(ST);
refNames = pts(1:3);
actNames = pts(4:6);
Req = TuningGoal.Tracking(refNames, actNames, 5, 0, 1);
opt = systuneOptions('Display', 'off');

try
    [ST_tuned, fSoft] = systune(ST, Req, opt);
    fprintf('SUCCESS fSoft=%g\n', fSoft);
catch e
    fprintf('=== FULL ERROR REPORT ===\n%s\n', getReport(e, 'extended', 'hyperlinks', 'off'));
    fprintf('\n=== identifier: %s ===\n', e.identifier);
    for i = 1:numel(e.stack)
        fprintf('  at %s line %d\n', e.stack(i).name, e.stack(i).line);
    end
end
