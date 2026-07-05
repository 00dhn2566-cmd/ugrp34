%% "PID 튜닝이 가능한 수준"(발산 없이 최소한 안정)까지 가기 위한 이분탐색.
%% 훨씬 더 보수적인 값부터 시작해서 roll/pitch/yaw/위치가 발산 안 하는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% 모터 스펙은 realistic하게(하지만 너무 타이트하지 않게) 유지
qc_motor.max_torque = 0.25;
qc_max_power = qc_motor.max_power;

% 훨씬 더 보수적인 attitude/yaw/altitude 게인 (원래값의 1/10이 아니라 극단적으로 축소)
kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.05; ki_altitude = 0;    kd_altitude = 0.05;
kp_position = 1;    ki_position = 0;    kd_position = 0.5;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 게인을 크게 낮춘 지금 상태에서, 프로펠러 방향(2,3번 반대)이 실제로
% 요에 영향을 주는지 재검증 (아까는 전체가 포화라 티가 안 났을 수 있음)
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end

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

for m = 1:4
    elec = sprintf('%s/Quadcopter/Electrical/Control%d', mdl, m);
    sigSrc  = {'ref', 'meas'};
    sigVars = {sprintf('motor%d_ref', m), sprintf('motor%d_meas', m)};
    for i = 1:numel(sigSrc)
        twName = ['To Workspace ' sigVars{i}];
        if isempty(find_system(elec, 'SearchDepth', 1, 'Name', twName))
            twBlk = [elec '/' twName];
            add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigVars{i}, 'SaveFormat', 'Array');
            srcPh = get_param([elec '/' sigSrc{i}], 'PortHandles');
            twPh  = get_param(twBlk, 'PortHandles');
            add_line(elec, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
        end
    end
end

% Control Yaw 블록의 입력(오차 e)이 실제로 어떤 신호를 받는지 -
% 그 소스를 찾아서 탭. 요 축 피드백이 끊겨있는지 확인하기 위함.
ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
yawBlk = [ypr '/Control Yaw'];
yawPh = get_param(yawBlk, 'PortHandles');
lineH = get_param(yawPh.Inport(1), 'Line');
if lineH ~= -1
    srcPortH = get_param(lineH, 'SrcPortHandle');
    if srcPortH ~= -1
        twBlk = [ypr '/To Workspace yaw_ctrl_input'];
        if isempty(find_system(ypr, 'SearchDepth', 1, 'Name', 'To Workspace yaw_ctrl_input'))
            add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'yaw_ctrl_input', 'SaveFormat', 'Array');
            twPh2 = get_param(twBlk, 'PortHandles');
            add_line(ypr, srcPortH, twPh2.Inport(1), 'autorouting', 'on');
        end
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

if isfield(result, 'yaw_ctrl_input')
    v = result.yaw_ctrl_input;
    fprintf('\n=== Control Yaw 입력(오차) ===\n');
    fprintf('  min=%g max=%g first5=%s last5=%s\n', min(v), max(v), ...
        mat2str(v(1:min(5,end))'), mat2str(v(max(1,end-4):end)'));
end

fprintf('\n=== 모터 ref/meas ===\n');
for m = 1:4
    refName = sprintf('motor%d_ref', m);
    measName = sprintf('motor%d_meas', m);
    if isfield(result, refName)
        fprintf('모터%d: ref last=%g, meas last=%g\n', m, result.(refName)(end), result.(measName)(end));
    end
end
