%% 모터 스펙(power_max/torque_max) + 상위 PID(attitude/position)를 동시에
%% 현실적인 값으로 낮춰서, 단순 호버 테스트가 발산 없이 되는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% FX450 A2212 1000KV급 모터 현실적 스펙으로 조정 (근사치)
qc_motor.max_power = 160;     % W, 기존값 유지(이미 현실적 범위)
qc_motor.max_torque = 0.25;   % N*m, 기존 0.8 -> 소형 BLDC 현실적 범위로 축소
qc_max_power = qc_motor.max_power;

% 상위 PID를 원래값(kp_attitude=128.5 등)의 1/10 수준으로 축소 (중간값 시도)
kp_attitude = 12.85; ki_attitude = 0.59; kd_attitude = 15.64;
kp_yaw      = 20.56;  ki_yaw = 0.0059;   kd_yaw = 0.0782;
kp_altitude = 0.027;  ki_altitude = 0.007; kd_altitude = 0.035;
kp_position = 0.8;    ki_position = 0.004; kd_position = 0.32;

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

scope = [mdl '/Scope'];
attSrc  = {'In Bus Element8', 'In Bus Element9', 'In Bus Element11'};
attVars = {'act_roll', 'act_pitch', 'act_yaw'};
for i = 1:numel(attSrc)
    twName = ['To Workspace ' attVars{i}];
    if isempty(find_system(scope, 'SearchDepth', 1, 'Name', twName))
        twBlk = [scope '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', attVars{i}, 'SaveFormat', 'Array');
        srcPh = get_param([scope '/' attSrc{i}], 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
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

fprintf('=== roll/pitch/yaw (deg) ===\n');
for i = 1:numel(attVars)
    if isfield(result, attVars{i})
        v = result.(attVars{i});
        fprintf('  %s: min=%g max=%g last=%g\n', attVars{i}, rad2deg(min(v)), rad2deg(max(v)), rad2deg(v(end)));
    end
end
if isfield(result, 'act_x1')
    fprintf('\n=== 위치 (m) ===\n');
    fprintf('  x: %g -> %g\n', result.act_x1(1), result.act_x1(end));
    fprintf('  y: %g -> %g\n', result.act_y1(1), result.act_y1(end));
    fprintf('  z: %g -> %g\n', result.act_z1(1), result.act_z1(end));
end
