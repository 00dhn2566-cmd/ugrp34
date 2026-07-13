%% t=1.5~2.0s 사이에 roll이 -60도에서 +3도로 0.5초만에 튀는 게
%% (1) 자세 명령(cmd.roll/cmd.pitch, Position Control 출력) 자체가 그렇게 튀는 건지
%% (2) 아니면 실측값(Chassis.roll/Chassis.pitch) 쪽의 문제(오일러각/gimbal lock 아티팩트 등)인지
%% 구분하기 위해 cmd와 real을 같이 로깅해서 그 구간을 촘촘히 비교.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.5;  ki_altitude = 0.1;  kd_altitude = 0.3;
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;  % 원래값(디폴트 Kdrag=0.01)

mdl = 'quadcopter_package_delivery';
load_system(mdl);
% Kdrag는 원래(기본, 0.01) 그대로 - 실제로 뜨고 roll -90도가 나오는 조건.

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
% real_roll/real_pitch = Chassis.roll/Chassis.pitch (실측), cmd_roll/cmd_pitch = Element21/22 (roll/pitch, cmd 추정)
sigMap = {'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

fprintf('=== 베이스라인 호버 실행 (cmd vs real roll/pitch 비교) ===\n');
simOut = sim(mdl);
t = real_roll.time(:);
rReal = rad2deg(real_roll.signals.values(:));
pReal = rad2deg(real_pitch.signals.values(:));
rCmd  = rad2deg(cmd_roll.signals.values(:));
pCmd  = rad2deg(cmd_pitch.signals.values(:));

fprintf('\n=== t=1.0~2.5s 구간, 0.05s 간격으로 cmd vs real 비교 ===\n');
fprintf('  %6s  %10s %10s   %10s %10s\n', 't', 'real_roll', 'cmd_roll', 'real_pitch', 'cmd_pitch');
checkTimes = 1.0:0.05:2.5;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  %6.2f  %10.3f %10.3f   %10.3f %10.3f\n', t(idx), rReal(idx), rCmd(idx), pReal(idx), pCmd(idx));
end

fprintf('\n=== cmd_roll 전체 범위 (이게 ±60도 saturation 안에 있는지) ===\n');
fprintf('  cmd_roll : min=%.3f max=%.3f\n', min(rCmd), max(rCmd));
fprintf('  cmd_pitch: min=%.3f max=%.3f\n', min(pCmd), max(pCmd));
