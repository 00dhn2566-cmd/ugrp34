%% "act_yaw"가 실제로는 Prop1.w(모터 속도)를 잘못 탭한 것이었다는 의심을 검증.
%% 진짜 Chassis.roll/pitch/yaw (In Bus Element4/3/5, 방금 덤프로 확인된 값)를
%% Control Pitch/Roll/Yaw의 실제 오차(Line/SrcPortHandle 추적, 세션5에서 이미 검증된 방식)와
%% 함께 로깅해서 진짜 그림을 다시 확인.

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

scope = [mdl '/Scope'];

% 검증된(방금 덤프로 확인된) 진짜 Chassis 매핑: E3=pitch, E4=roll, E5=yaw
realMap = {'In Bus Element3','real_pitch'; 'In Bus Element4','real_roll'; 'In Bus Element5','real_yaw'};
for i = 1:size(realMap,1)
    twName = ['To Workspace ' realMap{i,2}];
    if isempty(find_system(scope, 'SearchDepth', 1, 'Name', twName))
        twBlk = [scope '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', realMap{i,2}, 'SaveFormat', 'Array');
        srcPh = get_param([scope '/' realMap{i,1}], 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
    end
end

% 참고용으로 예전에 "act_yaw"라고 불렀던 것(In Bus Element11=Prop1.w)도 같이 찍어서 직접 비교
twName = 'To Workspace old_actyaw_was_prop1w';
if isempty(find_system(scope, 'SearchDepth', 1, 'Name', twName))
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'old_actyaw_was_prop1w', 'SaveFormat', 'Array');
    srcPh = get_param([scope '/In Bus Element11'], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% Control Pitch/Roll/Yaw 실제 오차 (Line/SrcPortHandle 추적, 세션5 방식 재사용)
ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
ctrlBlocks = {[ypr '/Control Pitch'], [ypr '/Control Roll'], [ypr '/Control Yaw']};
tapVars = {'pitch_error', 'roll_error', 'yaw_error'};
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

fprintf('=== 진짜 Chassis.roll/pitch/yaw (rad, deg) ===\n');
for name = {'real_roll','real_pitch','real_yaw'}
    n = name{1};
    if isfield(result, n)
        v = result.(n);
        fprintf('  %s: min=%g max=%g last=%g (deg: min=%g max=%g last=%g)\n', ...
            n, min(v), max(v), v(end), rad2deg(min(v)), rad2deg(max(v)), rad2deg(v(end)));
    else
        fprintf('  %s: 로깅 안됨\n', n);
    end
end

fprintf('\n=== (비교용) 옛날 "act_yaw"가 실제로 참조하던 값 (Prop1.w, rad/s 단위) ===\n');
if isfield(result, 'old_actyaw_was_prop1w')
    v = result.old_actyaw_was_prop1w;
    fprintf('  min=%g max=%g last=%g\n', min(v), max(v), v(end));
end

fprintf('\n=== Control Pitch/Roll/Yaw 오차 (Line 추적, 세션5 방식) ===\n');
for name = {'pitch_error','roll_error','yaw_error'}
    n = name{1};
    if isfield(result, n)
        v = result.(n);
        fprintf('  %s: min=%g max=%g last=%g\n', n, min(v), max(v), v(end));
    end
end
