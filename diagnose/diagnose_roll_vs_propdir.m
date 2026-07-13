%% roll이 -90도 근처까지 떨어지는 현상이 프로펠러 방향 수정(Prop 2,3 Negative) 때문인지,
%% 아니면 그거와 무관하게(수정 전 기본 상태에서도) 일어나는 현상인지 A/B로 직접 비교.

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

% StructureWithTime 포맷 사용: 신호와 시간이 항상 같은 변수 안에서 정확히 짝지어짐
% (별도 Clock 블록 + 별도 To Workspace로 sim_time을 따로 찍으면 두 블록의
% 샘플링이 어긋날 때 시간축이 잘못 정렬될 수 있음 - 실제로 이번에 그 버그가 발생함)
scope = [mdl '/Scope'];
twName = 'To Workspace real_roll';
oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
if ~isempty(oldTw)
    delete_block(oldTw{1});
end
twBlk = [scope '/' twName];
add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'real_roll', 'SaveFormat', 'StructureWithTime');
srcPh = get_param([scope '/In Bus Element4'], 'PortHandles');
twPh  = get_param(twBlk, 'PortHandles');
add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');

% 고도(z)도 같이 로깅 - "추력 부족으로 계속 가라앉는다" 가설 확인용 (Chassis.pz = In Bus Element2)
twName2 = 'To Workspace real_z';
oldTw2 = find_system(scope, 'SearchDepth', 1, 'Name', twName2);
if ~isempty(oldTw2)
    delete_block(oldTw2{1});
end
twBlk2 = [scope '/' twName2];
add_block('simulink/Sinks/To Workspace', twBlk2, 'VariableName', 'real_z', 'SaveFormat', 'StructureWithTime');
srcPh2 = get_param([scope '/In Bus Element2'], 'PortHandles');
twPh2  = get_param(twBlk2, 'PortHandles');
add_line(scope, srcPh2.Outport(1), twPh2.Inport(1), 'autorouting', 'on');

propBlocks = {[mdl '/Quadcopter/Propeller 1/Thrust and Drag/Aerodynamic Propeller'], ...
              [mdl '/Quadcopter/Propeller 2/Thrust and Drag/Aerodynamic Propeller'], ...
              [mdl '/Quadcopter/Propeller 3/Thrust and Drag/Aerodynamic Propeller'], ...
              [mdl '/Quadcopter/Propeller 4/Thrust and Drag/Aerodynamic Propeller']};

fprintf('=== 현재(기본) 프로펠러 direction 상태 ===\n');
for i = 1:numel(propBlocks)
    try
        fprintf('  Prop%d: %s\n', i, get_param(propBlocks{i}, 'direction'));
    catch e
        fprintf('  Prop%d: 조회 실패 (%s)\n', i, e.message);
    end
end

fprintf('\n=== [A] 기본 상태(전부 Positive)로 호버 테스트 ===\n');
simOut = sim(mdl);
tA = real_roll.time(:);
yA = real_roll.signals.values(:);
zA = real_z.signals.values(:);
fprintf('  real_roll: min=%g max=%g last=%g (deg: min=%g max=%g last=%g), n=%d, t범위=[%g %g]\n', ...
    min(yA), max(yA), yA(end), rad2deg(min(yA)), rad2deg(max(yA)), rad2deg(yA(end)), numel(yA), tA(1), tA(end));
fprintf('  real_z: min=%g max=%g last=%g (목표=1.0m)\n', min(zA), max(zA), zA(end));

fprintf('\n=== [B] Prop2,3 -> Negative로 바꾸고 재테스트 ===\n');
set_param(propBlocks{2}, 'direction', 'sdl.enum.PropellerDirection.Negative');
set_param(propBlocks{3}, 'direction', 'sdl.enum.PropellerDirection.Negative');
simOut = sim(mdl);
tB = real_roll.time(:);
yB = real_roll.signals.values(:);
zB = real_z.signals.values(:);
fprintf('  real_roll: min=%g max=%g last=%g (deg: min=%g max=%g last=%g), n=%d, t범위=[%g %g]\n', ...
    min(yB), max(yB), yB(end), rad2deg(min(yB)), rad2deg(max(yB)), rad2deg(yB(end)), numel(yB), tB(1), tB(end));
fprintf('  real_z: min=%g max=%g last=%g (목표=1.0m)\n', min(zB), max(zB), zB(end));

fprintf('\n=== 결론 (A vs B) ===\n');
if isequal(yA, yB)
    fprintf('  => 완전히 동일함(bit-identical). 방향 수정과 무관.\n');
else
    fprintf('  => 다름. 방향 수정이 roll 거동에 실제로 영향을 줌.\n');
end

%% [B](방향 수정 적용, 더 나은 상태) 파형을 정확한 시간 정렬로 다운샘플 출력
fprintf('\n=== [B] real_roll(t), real_z(t) 다운샘플 (약 100포인트, 정확한 시간 정렬) ===\n');
n = numel(tB);
stride = max(1, floor(n/100));
for idx = 1:stride:n
    fprintf('  t=%6.3f  roll=%9.4f deg  z=%7.4f m\n', tB(idx), rad2deg(yB(idx)), zB(idx));
end
fprintf('  t=%6.3f  roll=%9.4f deg  z=%7.4f m (마지막)\n', tB(end), rad2deg(yB(end)), zB(end));

[minVal, minIdx] = min(yB);
fprintf('\n  [B] roll 최소값 지점: t=%6.4f, roll=%9.4f deg\n', tB(minIdx), rad2deg(minVal));
