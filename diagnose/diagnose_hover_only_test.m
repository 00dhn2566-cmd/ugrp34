%% 궤적 없이(한 점에 고정) 짧은 호버 테스트 - 그래도 pitch/altitude가 폭주하면
%% 게인 문제가 아니라 부호(피드백 극성)/배선 문제일 가능성이 높음.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 한 점에 5초간 가만히 떠있기만 하는 아주 단순한 궤적
dt = 0.01;
T = 5;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];   % 원점 위 1m
spline_data = repmat(hoverPoint, N, 1);
spline_yaw = zeros(N, 1);
% quadcopter_waypoints_to_path_vis는 floor(distance)*4개의 점을 보간하므로
% distance가 1보다 작으면 보간점이 0개가 되어 Spline 블록이 에러남 -> 2m 이상 벌림.
% (spline_data/실제 제어 피드는 여전히 hoverPoint에 고정이라 시각화용 waypoints만 영향받음)
waypoints = [hoverPoint; hoverPoint + [0 0 2]]';  % 3x2

wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

% 모터별 명령(w1~4)과 자세각(pitch) 로깅 추가
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
motorCmdSrc  = {'Add4', 'Add5', 'Add7', 'Add6'};
motorCmdVars = {'motor_cmd_w1', 'motor_cmd_w2', 'motor_cmd_w3', 'motor_cmd_w4'};
for i = 1:numel(motorCmdSrc)
    twName = ['To Workspace ' motorCmdVars{i}];
    if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', twName))
        twBlk = [mixer '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', motorCmdVars{i}, 'SaveFormat', 'Array');
        srcPh = get_param([mixer '/' motorCmdSrc{i}], 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(mixer, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
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
fprintf('로그된 변수: %s\n', strjoin(fieldnames(result), ', '));

if isfield(result, 'act_x1')
    fprintf('\n=== act_x1/y1/z1 (실제 위치) 처음/끝 ===\n');
    fprintf('  x: %g -> %g\n', result.act_x1(1), result.act_x1(end));
    fprintf('  y: %g -> %g\n', result.act_y1(1), result.act_y1(end));
    fprintf('  z: %g -> %g\n', result.act_z1(1), result.act_z1(end));
end
if isfield(result, 'motor_cmd_w1')
    fprintf('\n=== motor_cmd_w1~4 범위 ===\n');
    for k = 1:4
        v = result.(sprintf('motor_cmd_w%d', k));
        fprintf('  w%d: min=%g max=%g last=%g\n', k, min(v), max(v), v(end));
    end
end
