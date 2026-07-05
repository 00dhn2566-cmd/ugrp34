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

% (2) Control Pitch/Roll/Yaw 각각의 입력(오차) 소스를 실제로 찾아서 탭.
% 이름을 짐작(Add/Add1)하지 않고, 각 Control 블록의 Inport 소스를 추적.
ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
ctrlBlocks = {[ypr '/Control Pitch'], [ypr '/Control Roll'], [ypr '/Control Yaw']};
tapVars = {'pitch_error', 'roll_error', 'yaw_error'};
filtBlocks = {[ypr '/Filter Pitch'], [ypr '/Filter Roll'], [ypr '/Filter Yaw']};
filtVars = {'filter_pitch_out', 'filter_roll_out', 'filter_yaw_out'};

for i = 1:numel(ctrlBlocks)
    cph = get_param(ctrlBlocks{i}, 'PortHandles');
    lineH = get_param(cph.Inport(1), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            twName = ['To Workspace ' tapVars{i}];
            if isempty(find_system(ypr, 'SearchDepth', 1, 'Name', twName))
                twBlk = [ypr '/' twName];
                add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', tapVars{i}, 'SaveFormat', 'Array');
                twPh = get_param(twBlk, 'PortHandles');
                add_line(ypr, srcPortH, twPh.Inport(1), 'autorouting', 'on');
            end
        end
    end
end
for i = 1:numel(filtBlocks)
    twName = ['To Workspace ' filtVars{i}];
    if isempty(find_system(ypr, 'SearchDepth', 1, 'Name', twName))
        twBlk = [ypr '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', filtVars{i}, 'SaveFormat', 'Array');
        srcPh = get_param(filtBlocks{i}, 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(ypr, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
    end
end
taps = {};

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

fprintf('=== act_yaw (Scope/Chassis.yaw, 실제 물리) ===\n');
if isfield(result, 'act_yaw')
    v = result.act_yaw;
    fprintf('  last=%g (deg=%g)\n', v(end), rad2deg(v(end)));
end

fprintf('\n=== 각 축의 Filter 출력(컨트롤러가 실제 보는 측정값) vs 오차(error) ===\n');
axes_ = {'pitch','roll','yaw'};
for i = 1:numel(axes_)
    fv = sprintf('filter_%s_out', axes_{i});
    ev = sprintf('%s_error', axes_{i});
    if isfield(result, fv)
        v = result.(fv);
        fprintf('  %s: min=%g max=%g last=%g (deg last=%g)\n', fv, min(v), max(v), v(end), rad2deg(v(end)));
    end
    if isfield(result, ev)
        v = result.(ev);
        fprintf('  %s: min=%g max=%g last=%g\n', ev, min(v), max(v), v(end));
    end
end
