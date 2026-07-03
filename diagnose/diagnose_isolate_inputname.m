modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

if isfile(fullfile(modelDir, 'trajectory.mat'))
    S = load(fullfile(modelDir, 'trajectory.mat'));
    timespot_spl = S.timespot_spl;
    spline_data  = S.spline_data;
    spline_yaw   = S.spline_yaw;
    waypoints    = S.waypoints';
else
    [waypoints, timespot_spl, spline_data, spline_yaw, ~] = quadcopter_package_select_trajectory(1);
end
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

io(1) = linio([mdl '/Scope/Demux'], 1, 'in');
io(2) = linio([mdl '/Scope/In Bus Element'], 1, 'out');

fprintf('=== Test A: Altitude and YPR Control ALONE ===\n');
try
    ST = slTuner(mdl, {[mdl '/Maneuver Controller/Altitude and  YPR Control']}, io);
    ST.Options.RateConversionOptions.Method = 'tustin';
    pts = getPoints(ST);
    Req = TuningGoal.Tracking(pts{1}, pts{2}, 1, 0, 1);
    [~, fSoft] = systune(ST, Req, systuneOptions('Display','off'));
    fprintf('  OK fSoft=%g\n', fSoft);
catch e
    fprintf('  FAILED: %s\n', e.message);
end

fprintf('\n=== Test B: Position Control/PID Controller ALONE ===\n');
try
    ST = slTuner(mdl, {[mdl '/Maneuver Controller/Position Control/PID Controller']}, io);
    ST.Options.RateConversionOptions.Method = 'tustin';
    pts = getPoints(ST);
    Req = TuningGoal.Tracking(pts{1}, pts{2}, 1, 0, 1);
    [~, fSoft] = systune(ST, Req, systuneOptions('Display','off'));
    fprintf('  OK fSoft=%g\n', fSoft);
catch e
    fprintf('  FAILED: %s\n', e.message);
end
