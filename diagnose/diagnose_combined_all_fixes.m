%% 지금까지 확인된 모든 수정을 한 번에 적용한 통합 테스트:
%% 1. 프로펠러 2,3 direction=Negative + Motor Mixer Add5/Add7 부호 반전 (6차 세션 수정,
%%    저장 안 돼 있어서 7~8차 테스트에는 빠져 있었음!)
%% 2. Kdrag=0.597 (8차 세션 재보정: 평형속도 직접 측정으로 검증, 632 vs 예측 634)
%% 3. Bias Chassis=100.98 rev/s (호버 피드포워드 재계산값)
%% 4. 보수적 게인 (kp_attitude=5 등)
%% -> 4개 프로펠러 추력 합이 22.3N에 도달하고 z가 1.0m까지 뜨는지 확인.

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

% --- 수정 2: Kdrag 재보정값 ---
propeller.Kdrag = 0.597;
assignin('base', 'propeller', propeller);

% --- 수정 3: 호버 피드포워드 ---
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;
n_hover = sqrt(T_need / (propeller.Kthrust * air_rho * propeller.diameter^4));
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', num2str(n_hover, '%.4f'));

% --- 수정 1: 프로펠러 2,3 방향 + Motor Mixer 부호 반전 (6차 세션 수정) ---
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
    fprintf('Propeller %d direction: %s\n', p, get_param(blk, 'direction'));
end
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
flipSigns = @(s) strrep(strrep(strrep(s, '+', 'X'), '-', '+'), 'X', '-');
add5_old = get_param([mixer '/Add5'], 'Inputs');
add7_old = get_param([mixer '/Add7'], 'Inputs');
set_param([mixer '/Add5'], 'Inputs', flipSigns(add5_old));
set_param([mixer '/Add7'], 'Inputs', flipSigns(add7_old));
fprintf('Mixer Add5(w2): %s -> %s / Add7(w3): %s -> %s\n', add5_old, get_param([mixer '/Add5'],'Inputs'), add7_old, get_param([mixer '/Add7'],'Inputs'));

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

set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element6','T1'; 'In Bus Element7','T2'; 'In Bus Element8','T3'; 'In Bus Element9','T4'; ...
          'In Bus Element11','W1'; 'In Bus Element10','W2'; 'In Bus Element12','W3'; 'In Bus Element13','W4'};
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

fprintf('\n=== 통합 수정 적용 후 호버 테스트 (Kdrag=0.597, Bias=%.2f, 프롭2,3+믹서 반전) ===\n', n_hover);
simOut = sim(mdl);
z = real_z.signals.values(:);
r = rad2deg(real_roll.signals.values(:));
p = rad2deg(real_pitch.signals.values(:));
t = real_z.time(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
fprintf('  추력(N, last): T1=%.3f T2=%.3f T3=%.3f T4=%.3f (합=%.2f, 필요=22.3)\n', ...
    T1.signals.values(end), T2.signals.values(end), T3.signals.values(end), T4.signals.values(end), ...
    T1.signals.values(end)+T2.signals.values(end)+T3.signals.values(end)+T4.signals.values(end));
fprintf('  속도(rad/s, last): W1=%.1f W2=%.1f W3=%.1f W4=%.1f\n', ...
    W1.signals.values(end), W2.signals.values(end), W3.signals.values(end), W4.signals.values(end));

fprintf('\n=== 시간별 z/roll/pitch 스냅샷 ===\n');
for ct = 0:0.25:5
    [~, idx] = min(abs(t - ct));
    fprintf('  t=%5.2fs: z=%7.4f roll=%8.3f pitch=%8.3f\n', t(idx), z(idx), r(idx), p(idx));
end
