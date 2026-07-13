%% 발견: 모터 PID 출력 한계(limit_motor=0.25)가 항상 포화 상태라, 어떤 ref를 줘도
%% 모터가 405.32 rad/s에 고정됨 (Kdrag 보정 후 호버 필요속도 634 rad/s에 도달 불가).
%% limit_motor를 올려서 모터가 실제로 634 rad/s에 도달하고 이륙하는지 확인.

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
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element11','prop1_w'};
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

limitVals = [0.25, 1.0, 2.5];
for k = 1:numel(limitVals)
    limit_motor = limitVals(k);
    fprintf('\n=== [%d] limit_motor = %.2f ===\n', k, limit_motor);
    simOut = sim(mdl);
    r = rad2deg(real_roll.signals.values(:));
    p = rad2deg(real_pitch.signals.values(:));
    z = real_z.signals.values(:);
    w = prop1_w.signals.values(:);
    fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));
    fprintf('  Prop1.w: min=%.2f max=%.2f last=%.2f rad/s (호버 필요 ~634)\n', min(w), max(w), w(end));
    fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
    fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
end

fprintf('\n=== 판정 ===\n');
fprintf('  limit_motor를 올렸을 때 Prop1.w가 634 근처로 오르고 z가 1.0에 접근하면\n');
fprintf('  -> "모터 출력 한계 포화"가 이륙 실패의 진짜 원인으로 확정.\n');
