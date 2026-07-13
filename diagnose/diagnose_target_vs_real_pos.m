%% 목표 위치(Traj.pos, Scope의 'pos' Element15로 추정)와 실제 위치(Chassis.px/py/pz)를
%% t=0부터 촘촘하게 같이 로깅해서, 정말로 서로 가까운 상태에서 시작하는지,
%% 아니면 목표 위치 자체가 뭔가 이상하게 움직이는지(예상과 다른 값) 확인.

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

dt = 0.01;
T = 3;
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
% real_x/y/z = Chassis.px/py/pz (실측), tgt_pos = Element15 'pos' (목표로 추정, 벡터일 수 있음)
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element15','tgt_pos'};
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

fprintf('=== 목표 위치(tgt_pos) vs 실제 위치(real_x/y/z), t=0~2s ===\n');
simOut = sim(mdl);
t = real_x.time(:);
rx = real_x.signals.values(:);
ry = real_y.signals.values(:);
rz = real_z.signals.values(:);
tgt = tgt_pos.signals.values;  % N x (채널수)
fprintf('tgt_pos 채널 수: %d\n', size(tgt,2));

fprintf('\n=== 시간별 스냅샷 (t=0~2s, 0.05s 간격) ===\n');
fprintf('  %6s  %8s %8s %8s   %8s %8s %8s (tgt 채널1,2,3)\n', 't', 'real_x','real_y','real_z','tgt1','tgt2','tgt3');
checkTimes = 0:0.05:2;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    if size(tgt,2) >= 3
        fprintf('  %6.2f  %8.4f %8.4f %8.4f   %8.4f %8.4f %8.4f\n', t(idx), rx(idx), ry(idx), rz(idx), tgt(idx,1), tgt(idx,2), tgt(idx,3));
    else
        fprintf('  %6.2f  %8.4f %8.4f %8.4f   (tgt has %d ch)\n', t(idx), rx(idx), ry(idx), rz(idx), size(tgt,2));
    end
end
