%% Spherical Joint(회전 자유도)의 PositionTargetRotationSequenceAngles를 통해
%% 초기 자세에 작은 각도 외란을 주입. 축 매핑(어느 성분이 pitch/roll에 대응하는지)은
%% 모르니, 먼저 [0,10,0](가운데=Y=pitch로 추정)으로 테스트해서 t=0 시점 real_pitch/real_roll
%% 값으로 매핑을 확인한 뒤, 5초 응답이 roll처럼 크게 튀는지 비교.

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
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw)
        delete_block(oldTw{1});
    end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

sph = [mdl '/Quadcopter/6 DOF/Joints/Spherical Joint'];
fprintf('=== 기존 IC 각도: %s ===\n', get_param(sph, 'PositionTargetRotationSequenceAngles'));

% [0,10,0] 로 설정해서 어느 축이 pitch/roll에 대응하는지 확인
set_param(sph, 'PositionTargetRotationSequenceAngles', '[0,10,0]');
fprintf('\n=== [테스트A] IC=[0,10,0]deg 적용 후 t=0 근처 값으로 축 매핑 확인 ===\n');
simOut = sim(mdl);
r = real_roll.signals.values(:); p = real_pitch.signals.values(:); y = real_yaw.signals.values(:);
tr = real_roll.time(:);
fprintf('  t=0 근처: roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg\n', ...
    rad2deg(r(1)), rad2deg(p(1)), rad2deg(y(1)));
fprintf('  전체 범위: roll min=%.3f max=%.3f last=%.3f\n', rad2deg(min(r)), rad2deg(max(r)), rad2deg(r(end)));
fprintf('  전체 범위: pitch min=%.3f max=%.3f last=%.3f\n', rad2deg(min(p)), rad2deg(max(p)), rad2deg(p(end)));
fprintf('  전체 범위: yaw min=%.3f max=%.3f last=%.3f\n', rad2deg(min(y)), rad2deg(max(y)), rad2deg(y(end)));

% [10,0,0] 도 확인 (혹시 첫번째 축이 pitch일 수도 있으니)
set_param(sph, 'PositionTargetRotationSequenceAngles', '[10,0,0]');
fprintf('\n=== [테스트B] IC=[10,0,0]deg 적용 후 t=0 근처 값 ===\n');
simOut = sim(mdl);
r2 = real_roll.signals.values(:); p2 = real_pitch.signals.values(:); y2 = real_yaw.signals.values(:);
fprintf('  t=0 근처: roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg\n', ...
    rad2deg(r2(1)), rad2deg(p2(1)), rad2deg(y2(1)));
fprintf('  전체 범위: roll min=%.3f max=%.3f last=%.3f\n', rad2deg(min(r2)), rad2deg(max(r2)), rad2deg(r2(end)));
fprintf('  전체 범위: pitch min=%.3f max=%.3f last=%.3f\n', rad2deg(min(p2)), rad2deg(max(p2)), rad2deg(p2(end)));
fprintf('  전체 범위: yaw min=%.3f max=%.3f last=%.3f\n', rad2deg(min(y2)), rad2deg(max(y2)), rad2deg(y2(end)));
