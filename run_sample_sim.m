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

simOut = sim('quadcopter_package_delivery');

t   = simOut.tout;
logsout = simOut.get('logsout');

fig = figure('Visible','off');
plot3(spline_data(:,1), spline_data(:,2), spline_data(:,3), 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Sample trajectory fed into quadcopter\_package\_delivery');
grid on;
saveas(fig, 'run_sample_sim_result.png');

fprintf('Simulation finished. tout range: [%.3f, %.3f] s, %d samples\n', ...
    t(1), t(end), numel(t));
