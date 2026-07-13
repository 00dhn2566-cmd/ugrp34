%% Pos Error saturation을 ±0.01m로 훨씬 타이트하게 걸어서 재확인.
%% (0.01m/0.01s = 1m/s 수준 - 훨씬 현실적인 속도로 낮춤)

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

subBlk = [pc '/Subtract2'];
pidBlk = [pc '/PID Controller'];
subPh = get_param(subBlk, 'PortHandles');
pidPh = get_param(pidBlk, 'PortHandles');

oldLine = get_param(subPh.Outport(1), 'Line');
if oldLine ~= -1
    delete_line(oldLine);
end

satBlk = [pc '/Pos Error Saturate'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'Pos Error Saturate'))
    add_block('simulink/Discontinuities/Saturation', satBlk, 'UpperLimit', '0.01', 'LowerLimit', '-0.01');
end
satPh = get_param(satBlk, 'PortHandles');
add_line(pc, subPh.Outport(1), satPh.Inport(1), 'autorouting', 'on');
add_line(pc, satPh.Outport(1), pidPh.Inport(1), 'autorouting', 'on');

scopeBlk = [pc '/Scope'];
scopePh = get_param(scopeBlk, 'PortHandles');
if get_param(scopePh.Inport(2), 'Line') == -1
    add_line(pc, subPh.Outport(1), scopePh.Inport(2), 'autorouting', 'on');
end

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

fprintf('=== Pos Error에 ±0.01m saturation 적용 후 베이스라인 호버 (IC=0) ===\n');
simOut = sim(mdl);
r = rad2deg(real_roll.signals.values(:));
p = rad2deg(real_pitch.signals.values(:));
y = real_y.signals.values(:);
z = real_z.signals.values(:);
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
fprintf('  y: min=%.3f max=%.3f last=%.3f (목표=0)\n', min(y), max(y), y(end));
fprintf('  z: last=%.4f (목표=1.0)\n', z(end));

fprintf('\n=== 이전(제한 없음) 대비 비교 ===\n');
fprintf('  이전: roll min=-89.986 max=5.518 last=4.175, y max=5.779\n');
fprintf('  지금: roll min=%.3f max=%.3f last=%.3f, y max=%.3f\n', min(r), max(r), r(end), max(y));

cRoll = rad2deg(cmd_roll.signals.values(:));
cPitch = rad2deg(cmd_pitch.signals.values(:));
t = real_roll.time(:);
fprintf('\n=== cmd_roll/cmd_pitch (Pos Error ±0.01m 제한 적용 후) ===\n');
fprintf('  cmd_roll: min=%.3f max=%.3f (포화=±60)\n', min(cRoll), max(cRoll));
fprintf('  cmd_pitch: min=%.3f max=%.3f\n', min(cPitch), max(cPitch));
fprintf('\n=== 시간별 cmd_roll 스냅샷 ===\n');
checkTimes = 0:0.1:2;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  t=%5.2fs: cmd_roll=%8.3f deg, real_roll=%8.3f deg\n', t(idx), cRoll(idx), r(idx));
end
