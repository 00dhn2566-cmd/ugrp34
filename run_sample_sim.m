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

% 짐(Package) 로고 CAD 선택: true = MathWorks 기본 예제 로고, false = 빈 패키지(로고 없음)
% Package 본체(BrickSolid, pkgSize 기반 단순 박스)는 두 경우 모두 그대로이고,
% Disengage Logic(투하 로직)도 영향 없음 — 순수 장식용 Logo 1/2 지오메트리만 켜고 끔.
use_default_package_branding = false;

load_system('quadcopter_package_delivery');
logo_blocks = {
    'quadcopter_package_delivery/Quadcopter/Load/Package/Logo 1'
    'quadcopter_package_delivery/Quadcopter/Load/Package/Logo 2'
};
if use_default_package_branding
    logo_state = 'off';
else
    logo_state = 'on';
end
for i = 1:numel(logo_blocks)
    set_param(logo_blocks{i}, 'Commented', logo_state);
end

% 투하(Disengage) 로직 on/off: true = 원래대로 dist_release/spd_release 조건에서 투하,
% false = 거리 임계값을 음수로 덮어써서 절대 투하 조건이 만족되지 않게 함.
% 배선(포트 연결)은 그대로 두고 Constant 블록 값만 바꾸는 방식이라 안전함.
enable_package_drop = true;

drop_dist_blocks = {
    'quadcopter_package_delivery/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'
    'quadcopter_package_delivery/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'
};
if enable_package_drop
    drop_dist_value = 'dist_release';
else
    drop_dist_value = '-1';   % 거리는 항상 0 이상이라 -1보다 작을 수 없음 -> 투하 조건 영원히 불만족
end
for i = 1:numel(drop_dist_blocks)
    set_param(drop_dist_blocks{i}, 'Value', drop_dist_value);
end

S = load('trajectory.mat');
timespot_spl = S.timespot_spl;
spline_data  = S.spline_data;
spline_yaw   = S.spline_yaw;
waypoints    = S.waypoints';             % (5,3) -> (3,5): Ground/Trajectory expects 3xN like quadcopter_package_select_trajectory.m

wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

% 우리 파트의 출력 경계: "각 모터에 들어가는 명령"(모터별 각속도 setpoint w1~w4)까지.
% 그 다음(모터 전기/역학, 프로펠러 공력 등 실제 물리 응답)은 Isaac Sim 쪽 역할이라
% 여기서는 Motor Mixer가 만든 w1~w4 신호만 To Workspace로 추가 로깅한다.
% (배선은 안 건드리고 기존 신호에서 분기만 추가 — Add4/5/7/6 순서가 각각 w1/w2/w3/w4에 대응.)
mixer = 'quadcopter_package_delivery/Maneuver Controller/Motor Mixer';
motorCmdSrc  = {'Add4', 'Add5', 'Add7', 'Add6'};   % w1, w2, w3, w4
motorCmdVars = {'motor_cmd_w1', 'motor_cmd_w2', 'motor_cmd_w3', 'motor_cmd_w4'};
for i = 1:numel(motorCmdSrc)
    twName = ['To Workspace ' motorCmdVars{i}];
    if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', twName))
        twBlk = [mixer '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, ...
            'VariableName', motorCmdVars{i}, 'SaveFormat', 'Array');
        srcPh = get_param([mixer '/' motorCmdSrc{i}], 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(mixer, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
    end
end

% motor_cmd_w1~w4는 Array 포맷이라 시간이 따로 안 붙어 있음 -> 같은 솔버 스텝마다
% 값을 찍는 Clock을 하나 추가해서 sim_time으로 같이 로깅 (행 순서가 그대로 대응됨).
mdl = 'quadcopter_package_delivery';
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Sim Time Clock'))
    add_block('simulink/Sources/Clock', [mdl '/Sim Time Clock']);
    add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace sim_time'], ...
        'VariableName', 'sim_time', 'SaveFormat', 'Array');
    clockPh = get_param([mdl '/Sim Time Clock'], 'PortHandles');
    twPh    = get_param([mdl '/To Workspace sim_time'], 'PortHandles');
    add_line(mdl, clockPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

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
