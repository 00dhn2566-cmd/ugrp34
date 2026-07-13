%% x,y 외란 테스트: 드론은 (0,0,1)에서 시작하는데 목표점을 (0.3,0.3,1.0)으로
%% 줘서 시작부터 x,y 오차 0.3m를 강제로 만들어냄.
%% Kdrag=4.222841(보정값) 상태에서 이 외란에 roll/pitch가 얌전히 반응하는지,
%% 아니면 (Kdrag 보정 전처럼) -90도급으로 폭주하는지 확인.
%% -> "roll이 깨끗했던 게 자극이 없어서 우연"이었는지 가려내는 테스트.
%% 클램프/새추레이션은 일절 안 넣음(순정 경로).

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
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% Kdrag 보정 + 피드포워드 재계산
propeller.Kdrag = 4.222841;
assignin('base', 'propeller', propeller);
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;
n_hover = sqrt(T_need / (propeller.Kthrust * air_rho * propeller.diameter^4));
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', num2str(n_hover, '%.4f'));

dt = 0.01;
T = 5;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
startPoint = [0, 0, 1.0];     % 드론 초기 위치 (waypoints(1,:)이 결정)
targetPoint = [0.3, 0.3, 1.0]; % 목표점을 옆으로 옮겨서 x,y 오차 0.3m 생성
spline_data = repmat(targetPoint, N, 1);
spline_yaw = zeros(N, 1);
% waypoints는 기존 작동 패턴(수직 이격 2점) 유지 - Spline 블록이 요구하는 형식.
% 초기 위치만 startPoint로 결정되고, 실제 추종 목표는 spline_data가 결정함.
waypoints = [startPoint; startPoint + [0 0 2]]';

mws = get_param(mdl, 'ModelWorkspace');
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'};
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

fprintf('=== x,y 외란 테스트: 시작 (0,0,1), 목표 (0.3,0.3,1), Kdrag=4.222841 ===\n');
simOut = sim(mdl);
r = rad2deg(real_roll.signals.values(:));
p = rad2deg(real_pitch.signals.values(:));
x = real_x.signals.values(:);
y = real_y.signals.values(:);
z = real_z.signals.values(:);
cR = rad2deg(cmd_roll.signals.values(:));
cP = rad2deg(cmd_pitch.signals.values(:));
t = real_z.time(:);

fprintf('  roll : min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
fprintf('  cmd_roll : min=%.3f max=%.3f\n', min(cR), max(cR));
fprintf('  cmd_pitch: min=%.3f max=%.3f\n', min(cP), max(cP));
fprintf('  x: min=%.4f max=%.4f last=%.4f (목표=0.3)\n', min(x), max(x), x(end));
fprintf('  y: min=%.4f max=%.4f last=%.4f (목표=0.3)\n', min(y), max(y), y(end));
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));

fprintf('\n=== 시간별 스냅샷 ===\n');
checkTimes = 0:0.2:5;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  t=%5.2fs: x=%7.4f y=%7.4f z=%7.4f | cmdR=%8.3f realR=%8.3f | cmdP=%8.3f realP=%8.3f\n', ...
        t(idx), x(idx), y(idx), z(idx), cR(idx), r(idx), cP(idx), p(idx));
end

fprintf('\n=== 판정 기준 ===\n');
fprintf('  roll/pitch가 수 도 이내로 얌전하면 -> Kdrag 보정 효과가 외란 하에서도 유효 (진짜 해결)\n');
fprintf('  -80~90도급으로 튀면 -> 이전의 "깨끗함"은 자극이 없어서 나온 우연 (자세루프 재튜닝 필요)\n');
