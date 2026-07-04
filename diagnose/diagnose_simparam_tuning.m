%% "Altitude and YPR Control" 블록 자체를 tunedBlocks로 잡는 대신,
%% attitude_kp/ki/kd를 Simulink.Parameter 객체로 만들어서 모델 전체(mdl)를
%% 대상으로 slTuner를 구성 -> 블록의 포트/버스 구조 문제를 우회할 수 있는지 테스트.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% attitude 게인만 우선 Simulink.Parameter로 교체 (테스트)
kp_attitude = Simulink.Parameter(kp_attitude);
kp_attitude.CoderInfo.StorageClass = 'Auto';
ki_attitude = Simulink.Parameter(ki_attitude);
ki_attitude.CoderInfo.StorageClass = 'Auto';
kd_attitude = Simulink.Parameter(kd_attitude);
kd_attitude.CoderInfo.StorageClass = 'Auto';

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

fprintf('=== slTuner(mdl, mdl, io) - 모델 전체에서 tunable Simulink.Parameter 자동 탐색 ===\n');
try
    ST = slTuner(mdl, mdl, io);
    ST.Options.RateConversionOptions.Method = 'tustin';
    pts = getPoints(ST);
    disp(pts);
    Req = TuningGoal.Tracking(pts{1}, pts{2}, 1, 0, 1);
    [ST_tuned, fSoft] = systune(ST, Req, systuneOptions('Display','off'));
    fprintf('  OK fSoft=%g\n', fSoft);
    fprintf('  Tuned kp_attitude = %g\n', ST_tuned.Blocks.kp_attitude.Value);
catch e
    fprintf('  FAILED: %s\n', e.message);
end
