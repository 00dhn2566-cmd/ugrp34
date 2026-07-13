%% Kdrag=4.22(실측 검증값), Kthrust=0.1072(그대로, 이미 검증됨) 적용 후:
%% [1] 현재 보수적 게인으로 호버 테스트 (모터 속도가 현실적 범위로 오는지)
%% [2] pitch IC=10도 외란 테스트 (과소감쇠 문제가 나아지는지)
%% 결과 보고 게인 조정 필요한지 판단.

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

% Kdrag만 실측 검증값으로 수정, Kthrust는 이미 검증된 원래값 유지
propeller.Kdrag = 4.222841;
assignin('base', 'propeller', propeller);
fprintf('=== 적용된 값: Kthrust=%.4f(원래값 유지), Kdrag=%.4f(수정) ===\n', propeller.Kthrust, propeller.Kdrag);

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
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element11','W1'; 'In Bus Element6','T1'};
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

fprintf('\n=== [1] Kdrag 수정 후, 현재 게인으로 호버 테스트 (IC 없음) ===\n');
simOut = sim(mdl);
z1 = real_z.signals.values(:); tz1 = real_z.time(:);
r1 = real_roll.signals.values(:); p1 = real_pitch.signals.values(:);
w1 = W1.signals.values(:); T1v = T1.signals.values(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z1), max(z1), z1(end));
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r1)), rad2deg(max(r1)), rad2deg(r1(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p1)), rad2deg(max(p1)), rad2deg(p1(end)));
fprintf('  Prop1.w: last=%.2f rad/s (%.0f RPM), Prop1.thrust: last=%.3f N\n', ...
    w1(end), w1(end)*60/(2*pi), T1v(end));
n = numel(tz1);
stride = max(1, floor(n/14));
fprintf('  z(t): ');
for idx = 1:stride:n
    fprintf('[%.1fs %.3f] ', tz1(idx), z1(idx));
end
fprintf('[%.1fs %.3f]\n', tz1(end), z1(end));

% pitch IC=10도 외란 테스트
sph = [mdl '/Quadcopter/6 DOF/Joints/Spherical Joint'];
set_param(sph, 'PositionTargetRotationSequenceAngles', '[0,10,0]');
fprintf('\n=== [2] Kdrag 수정 후 pitch IC=10도 외란 테스트 ===\n');
simOut = sim(mdl);
r2 = real_roll.signals.values(:); p2 = real_pitch.signals.values(:);
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r2)), rad2deg(max(r2)), rad2deg(r2(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p2)), rad2deg(max(p2)), rad2deg(p2(end)));
fprintf('  (수정 전에는 pitch가 88.8도까지 튀었음 - 비교용)\n');
set_param(sph, 'PositionTargetRotationSequenceAngles', '[0,0,0]');

% 게인은 그대로 두고, 호버 피드포워드(Bias Chassis)를 새 Kthrust/Kdrag 기준으로
% 재계산해서 재시도. n_hover = sqrt(T_need/(Kthrust*rho*D^4))
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;
n_hover = sqrt(T_need / (propeller.Kthrust * air_rho * propeller.diameter^4));
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
fprintf('\n=== [3] 피드포워드(Bias Chassis) 재계산: 700 -> %.2f, 게인은 원래대로 ===\n', n_hover);
set_param(biasBlk, 'Bias', num2str(n_hover, '%.4f'));
simOut = sim(mdl);
z3 = real_z.signals.values(:); tz3 = real_z.time(:);
r3 = real_roll.signals.values(:); p3 = real_pitch.signals.values(:);
w3 = W1.signals.values(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z3), max(z3), z3(end));
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r3)), rad2deg(max(r3)), rad2deg(r3(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p3)), rad2deg(max(p3)), rad2deg(p3(end)));
fprintf('  Prop1.w: last=%.2f rad/s (%.0f RPM)\n', w3(end), w3(end)*60/(2*pi));
n = numel(tz3);
stride = max(1, floor(n/14));
fprintf('  z(t): ');
for idx = 1:stride:n
    fprintf('[%.1fs %.3f] ', tz3(idx), z3(idx));
end
fprintf('[%.1fs %.3f]\n', tz3(end), z3(end));
