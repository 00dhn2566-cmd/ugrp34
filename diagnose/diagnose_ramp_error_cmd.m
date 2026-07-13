%% t=0부터 목표 위치를 램프로 서서히 접근시키면서, 목표위치-실제위치 오차와
%% cmd_roll/cmd_pitch(자세 명령)을 같이 로깅해서, 오차가 실제로 작게 유지되는지,
%% 그리고 cmd가 포화(±60도) 없이 적절한 범위에 머무는지 확인.

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
T = 5;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
startPoint = [0, 0, 0];
ramp_time = 2.0;  % 2초에 걸쳐 서서히 접근

spline_data = zeros(N,3);
for i = 1:N
    tt = timespot_spl(i);
    frac = min(tt/ramp_time, 1.0);
    frac_smooth = 3*frac^2 - 2*frac^3;
    spline_data(i,:) = startPoint + frac_smooth*(hoverPoint - startPoint);
end
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
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; ...
          'In Bus Element15','tgt_pos'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'};
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

fprintf('=== %.1f초 램프 궤적, 목표-실제 오차 + cmd 로깅 (t=0~%.0fs) ===\n', ramp_time, T);
simOut = sim(mdl);
t = real_x.time(:);
rx = real_x.signals.values(:); ry = real_y.signals.values(:); rz = real_z.signals.values(:);
tgt = tgt_pos.signals.values;
rRoll = rad2deg(real_roll.signals.values(:));
rPitch = rad2deg(real_pitch.signals.values(:));
cRoll = rad2deg(cmd_roll.signals.values(:));
cPitch = rad2deg(cmd_pitch.signals.values(:));

errX = tgt(:,1) - rx;
errY = tgt(:,2) - ry;
errZ = tgt(:,3) - rz;

fprintf('\n=== 시간별 스냅샷: 목표-실제 오차, cmd, real ===\n');
fprintf('  %6s  %8s %8s %8s   %8s %8s   %8s %8s\n', 't', 'errX','errY','errZ','cmdRoll','cmdPitch','realRoll','realPitch');
checkTimes = 0:0.1:3;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  %6.2f  %8.4f %8.4f %8.4f   %8.3f %8.3f   %8.3f %8.3f\n', ...
        t(idx), errX(idx), errY(idx), errZ(idx), cRoll(idx), cPitch(idx), rRoll(idx), rPitch(idx));
end

fprintf('\n=== 전체 구간 요약 ===\n');
fprintf('  errY: min=%.4f max=%.4f\n', min(errY), max(errY));
fprintf('  cmd_roll: min=%.3f max=%.3f (포화=±60)\n', min(cRoll), max(cRoll));
fprintf('  real_roll: min=%.3f max=%.3f last=%.3f\n', min(rRoll), max(rRoll), rRoll(end));
