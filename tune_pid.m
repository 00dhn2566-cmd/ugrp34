%% Retune the 5 PID Compensator blocks in Maneuver Controller for the FX450 CAD
% (Position Control, Pitch/Roll/Thrust/Yaw) using Control System Toolbox's
% systune. Uses the Scope subsystem's own input ports as the linearization
% reference (desired position, in2) and output (actual position, in1)
% analysis points, since those already carry exactly the signals logged as
% des_x/y/z1 and act_x/y/z1 in run_sample_sim.m.

addpath('Scripts_Data');
addpath('Models');
addpath('Libraries');
addpath(genpath('CAD'));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 선형화 기준이 될 operating trajectory 로드 (있으면 그대로 사용)
if isfile('trajectory.mat')
    S = load('trajectory.mat');
    timespot_spl = S.timespot_spl;
    spline_data  = S.spline_data;
    spline_yaw   = S.spline_yaw;
    waypoints    = S.waypoints';
else
    [waypoints, timespot_spl, spline_data, spline_yaw, ~] = quadcopter_package_select_trajectory(1);
end
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

tunedBlocks = {
    'quadcopter_package_delivery/Maneuver Controller/Position Control/PID Controller'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control/Control Pitch/PID Compensator Formula'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control/Control Roll/PID Compensator Formula'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control/Control Thrust/PID Compensator Formula'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control/Control Yaw/PID Compensator Formula'
};

io(1) = linio([mdl '/From'], 1, 'in');        % desired position (reference)
io(2) = linio([mdl '/Quadcopter'], 1, 'out'); % actual position (measured)

ST = slTuner(mdl, tunedBlocks, io);

pts = getPoints(ST);
disp(pts);
refName = pts{1};
actName = pts{2};

% 궤적 총 시간(34s 근방)에 맞춘 대략적인 정착시간 요구조건: 5초 내 정착, 정상상태 오차 0
Req = TuningGoal.Tracking(refName, actName, 5, 0, 1);

opt = systuneOptions('Display', 'iter');
[ST_tuned, fSoft, ~] = systune(ST, Req, opt);

fprintf('\n=== Tuned soft goal value: %.4f (want < 1) ===\n\n', fSoft);

for i = 1:numel(tunedBlocks)
    fprintf('--- %s ---\n', tunedBlocks{i});
    showTunable(ST_tuned, tunedBlocks{i});
end

save('tuned_pid.mat', 'ST_tuned', 'fSoft', 'tunedBlocks');
fprintf('\nSaved tuned controller object to tuned_pid.mat\n');
