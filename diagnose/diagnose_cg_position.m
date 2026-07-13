%% (K) 도립진자 가설 0순위 확인: 패키지(1kg)가 기체 대비 위에 있는지 아래에 있는지.
%% Scope 버스의 Chassis.pz(E2)와 Load.px/py/pz(E26/27/28)를 같이 로깅해서
%% t=0(초기 배치)과 직후 시점의 상대 높이를 직접 비교.
%% Load.pz > Chassis.pz 이면 패키지가 위 -> CG가 추력면 위 -> 도립진자 확정.

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

dt = 0.01;
T = 1;
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

set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','chassis_z'; 'In Bus Element','chassis_x'; 'In Bus Element1','chassis_y'; ...
          'In Bus Element26','load_x'; 'In Bus Element27','load_y'; 'In Bus Element28','load_z'};
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

fprintf('=== CG/패키지 위치 확인 (Chassis vs Load 좌표 직접 비교) ===\n');
simOut = sim(mdl);
t = chassis_z.time(:);
cz = chassis_z.signals.values(:); cx = chassis_x.signals.values(:); cy = chassis_y.signals.values(:);
lz = load_z.signals.values(:);    lx = load_x.signals.values(:);    ly = load_y.signals.values(:);

fprintf('\n  %5s | %8s %8s | %8s (Load.z - Chassis.z)\n', 't', 'Chassis.z', 'Load.z', 'dz');
for ct = [0 0.05 0.1 0.2 0.3 0.5 0.8 1.0]
    [~, idx] = min(abs(t - ct));
    fprintf('  %5.2f | %8.4f %8.4f | %+8.4f\n', t(idx), cz(idx), lz(idx), lz(idx)-cz(idx));
end
fprintf('\n  t=0 수평 오프셋: Load-Chassis dx=%+.4f dy=%+.4f\n', lx(1)-cx(1), ly(1)-cy(1));

fprintf('\n=== 판정 ===\n');
dz0 = lz(1) - cz(1);
if dz0 > 0.01
    fprintf('  Load가 Chassis보다 %.3fm 위 -> CG가 추력면 위 -> (K) 도립진자 가설 확정!\n', dz0);
elseif dz0 < -0.01
    fprintf('  Load가 Chassis보다 %.3fm 아래 (정상 슬렁 로드 배치) -> 패키지 위치는 원인 아님.\n', -dz0);
else
    fprintf('  Load와 Chassis가 거의 같은 높이 (%.4fm 차이).\n', dz0);
end
