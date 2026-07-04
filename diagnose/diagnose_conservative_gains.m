%% 포화 상태를 벗어나는지 확인하기 위해, attitude 게인을 훨씬 보수적으로 낮추고
%% run_sample_sim.m과 동일한 방식으로 시뮬레이션 돌려서 motor_cmd_w1~4 범위를 본다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;   % 기본값들 로드

% 원래: kp_attitude=128.505, ki_attitude=5.9203, kd_attitude=156.4
kp_attitude = 5;
ki_attitude = 0;
kd_attitude = 2;
fprintf('보수적 게인 적용: kp_attitude=%g, ki_attitude=%g, kd_attitude=%g\n', ...
    kp_attitude, ki_attitude, kd_attitude);

mdl = 'quadcopter_package_delivery';
load_system(mdl);

trajFile = fullfile(modelDir, 'trajectory.mat');
S = load(trajFile);
timespot_spl = S.timespot_spl;
spline_data  = S.spline_data;
spline_yaw   = S.spline_yaw;
waypoints    = S.waypoints';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

vars_before = who;
simOut = sim(mdl);
fprintf('Simulation finished. class(simOut)=%s\n', class(simOut));

vars_after = who;
new_vars = setdiff(vars_after, [vars_before; {'vars_before'}]);
result = struct();
for i = 1:numel(new_vars)
    v = eval(new_vars{i});
    if isnumeric(v)
        result.(new_vars{i}) = v;
    end
end
fprintf('로그된 변수: %s\n', strjoin(fieldnames(result), ', '));

if isfield(result, 'act_x1') && isfield(result, 'des_x1')
    fprintf('\n최종 act_x1 = %g, des_x1 = %g (목표 근접해야 함)\n', result.act_x1(end), result.des_x1(end));
end
