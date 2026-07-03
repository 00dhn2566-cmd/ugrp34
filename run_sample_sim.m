%% Run quadcopter_package_delivery with a Python-generated sample trajectory
% Loads timespot_spl / spline_data / spline_yaw from trajectory.mat
% (produced by waypoints_to_maneuver_input.py) into the base workspace,
% then simulates the model and saves a result plot.

addpath('Scripts_Data');
addpath('Models');
addpath('Libraries');
addpath(genpath('CAD'));   % File Solid blocks store bare filenames (e.g. quadcopter_drone_arm.stp)
load_system('quadcopter_library');

quadcopter_package_parameters;   % drone_mass, propeller.*, qc_motor.*, PID gains, ...

S = load('trajectory.mat');
timespot_spl = S.timespot_spl;
spline_data  = S.spline_data;
spline_yaw   = S.spline_yaw;
waypoints    = S.waypoints';             % (5,3) -> (3,5): Ground/Trajectory expects 3xN like quadcopter_package_select_trajectory.m

wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

vars_before = who;
simOut = sim('quadcopter_package_delivery');
fprintf('Simulation finished successfully. class(simOut) = %s\n', class(simOut));

% sim() 도중 To Workspace 블록 등이 새로 만든 변수를 전부 sim_result.mat으로 저장
% (신호 이름을 미리 알 필요 없이, 어떤 신호가 로깅되든 그대로 받아지게 하기 위함)
vars_after = who;
new_vars = setdiff(vars_after, [vars_before; {'vars_before'}]);
result = struct();
for i = 1:numel(new_vars)
    v = eval(new_vars{i});
    if isnumeric(v)
        result.(new_vars{i}) = v;
    end
end
save('sim_result.mat', '-struct', 'result');
fprintf('Logged variables saved to sim_result.mat: %s\n', strjoin(fieldnames(result), ', '));

fig = figure('Visible','off');
plot3(spline_data(:,1), spline_data(:,2), spline_data(:,3), 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Sample trajectory fed into quadcopter\_package\_delivery');
grid on;
saveas(fig, 'run_sample_sim_result.png');
fprintf('Reference trajectory plot saved: run_sample_sim_result.png\n');
