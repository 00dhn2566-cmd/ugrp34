%% Scope에서 읽은 act_yaw(Chassis.yaw)와, Control Yaw로 실제 들어가는
%% Filter Yaw 출력/입력을 동시에 로깅해서 서로 값이 다른지 직접 비교.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.05; ki_altitude = 0;    kd_altitude = 0.05;
kp_position = 1;    ki_position = 0;    kd_position = 0.5;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

dt = 0.01;
T = 5;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
spline_data = repmat(hoverPoint, N, 1);
spline_yaw = zeros(N, 1);
waypoints = [hoverPoint; hoverPoint + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

% (1) Scope 기준 act_yaw (Chassis.yaw)
scope = [mdl '/Scope'];
if isempty(find_system(scope, 'SearchDepth', 1, 'Name', 'To Workspace act_yaw'))
    twBlk = [scope '/To Workspace act_yaw'];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'act_yaw', 'SaveFormat', 'Array');
    srcPh = get_param([scope '/In Bus Element11'], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% (2) Control Yaw로 들어가는 오차(Add3 출력) 및 Filter Yaw 출력(필터링된 실제 요)
ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
add3 = [ypr '/Add3'];
filtYaw = [ypr '/Filter Yaw'];
taps = {add3, filtYaw};
tapVars = {'yaw_error', 'filter_yaw_out'};
for i = 1:numel(taps)
    twName = ['To Workspace ' tapVars{i}];
    if isempty(find_system(ypr, 'SearchDepth', 1, 'Name', twName))
        twBlk = [ypr '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', tapVars{i}, 'SaveFormat', 'Array');
        srcPh = get_param(taps{i}, 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(ypr, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
    end
end

vars_before = who;
simOut = sim(mdl);
vars_after = who;
new_vars = setdiff(vars_after, [vars_before; {'vars_before'}]);
result = struct();
for i = 1:numel(new_vars)
    v = eval(new_vars{i});
    if isnumeric(v)
        result.(new_vars{i}) = v;
    end
end

fprintf('=== act_yaw (Scope/Chassis.yaw) vs filter_yaw_out (Control Yaw가 실제 보는 값) ===\n');
if isfield(result, 'act_yaw')
    v = result.act_yaw;
    fprintf('act_yaw: first5=%s last5=%s (deg last=%g)\n', mat2str(v(1:5)'), mat2str(v(end-4:end)'), rad2deg(v(end)));
end
if isfield(result, 'filter_yaw_out')
    v = result.filter_yaw_out;
    fprintf('filter_yaw_out: first5=%s last5=%s (deg last=%g)\n', mat2str(v(1:5)'), mat2str(v(end-4:end)'), rad2deg(v(end)));
end
if isfield(result, 'yaw_error')
    v = result.yaw_error;
    fprintf('yaw_error: first5=%s last5=%s\n', mat2str(v(1:5)'), mat2str(v(end-4:end)'));
end
