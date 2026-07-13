%% Position Control 안의 PID Controller에 anti-windup을 켜서
%% (LimitOutput=on, Upper/LowerSaturationLimit=±pi/3, AntiWindupMode=back-calculation)
%% err2r_out 폭주(721도까지 발산)가 사라지고 roll/y 발산이 줄어드는지 확인.

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
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;  % 원래값

mdl = 'quadcopter_package_delivery';
load_system(mdl);
pc = [mdl '/Maneuver Controller/Position Control'];
pid = [pc '/PID Controller'];

% PID 출력 자체는 rad 단위 tilt 명령이 아니라 world-frame 가속도류 값이라
% 정확한 saturation limit을 얼마로 잡아야 할지 애매하므로, 일단 넉넉하게
% (Roll/Pitch Limit의 몇 배 정도로) 잡아서 "무한폭주"만 막는 완충 역할로 테스트.
% PID(s) 출력은 Matrix Multiply/Dir R/Err2R를 거치기 전이라 각도 단위가 아님 -
% 우선 큰 값(예: ±50)으로 시작해서 무한폭주만 막는 효과를 본다.
set_param(pid, 'LimitOutput', 'on');
set_param(pid, 'UpperSaturationLimit', '50');
set_param(pid, 'LowerSaturationLimit', '-50');
set_param(pid, 'AntiWindupMode', 'back-calculation');

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
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
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

fprintf('=== anti-windup 켠 뒤 베이스라인 호버 (IC=0) ===\n');
simOut = sim(mdl);
r = rad2deg(real_roll.signals.values(:));
p = rad2deg(real_pitch.signals.values(:));
y = real_y.signals.values(:);
z = real_z.signals.values(:);
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
fprintf('  y: min=%.3f max=%.3f last=%.3f (목표=0)\n', min(y), max(y), y(end));
fprintf('  z: last=%.4f (목표=1.0)\n', z(end));

fprintf('\n=== 이전(anti-windup 없음) 대비 비교 ===\n');
fprintf('  이전: roll min=-89.986 max=5.518 last=4.175, y max=5.779\n');
fprintf('  지금: roll min=%.3f max=%.3f last=%.3f, y max=%.3f\n', min(r), max(r), r(end), max(y));
