%% 호버 피드포워드(Bias Chassis=700)를 FX450 무게에 맞게 교정.
%% 1) 현재 상태로 호버 테스트 -> 실측 (모터속도 w, 프로펠러 추력 T)에서 추력계수 c=T/w^2 역산
%% 2) 필요 추력(= 총무게/4)에서 필요한 모터속도 w_need = sqrt(T_need/c) 계산
%% 3) Bias Chassis를 그에 맞게 수정하고 재시뮬 -> z가 1.0m 잡는지 검증
%% 모터 물리 한계(~7420 rad/s) 초과 여부도 판정.

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

% 프로펠러 방향 수정 (roll 안정화에 필수)
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end

% 로깅: z(E2), roll(E4), Prop1.w(E11), Prop1.thrust(E6)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element11','prop1_w'; 'In Bus Element6','prop1_T'};
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

biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
fprintf('현재 Bias Chassis = %s\n', get_param(biasBlk, 'Bias'));

% 필요 추력: 총무게(드론+패키지)/4
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;

fprintf('\n=== [1] Bias=700 (기존값) 실측 ===\n');
simOut = sim(mdl);
z1 = real_z.signals.values(:);
w1 = prop1_w.signals.values(:);
T1 = prop1_T.signals.values(:);
fprintf('  z: min=%.4f last=%.4f\n', min(z1), z1(end));
fprintf('  Prop1.w last=%.2f rad/s, Prop1.thrust last=%.4f N\n', w1(end), T1(end));

% 추력계수 역산 (정상상태 실측값): T = c*w^2
c = T1(end) / w1(end)^2;
w_need = sqrt(T_need / c);
fprintf('  역산 추력계수 c=%.4e -> 총무게=%.4fkg, 프로펠러당 필요추력=%.3fN -> 필요속도=%.1f rad/s\n', ...
    c, totalMass, T_need, w_need);
if w_need > 7000
    fprintf('  !! 필요 속도가 모터 물리한계(~7420 rad/s)에 근접/초과 -> max_power 증대 또는 패키지 제거 필요\n');
end

fprintf('\n=== [2] Bias=800 실측 (Bias->속도 민감도 측정, Pause Motor 곱셈 등 모든 스케일 포함) ===\n');
set_param(biasBlk, 'Bias', '800');
simOut = sim(mdl);
w2 = prop1_w.signals.values(:);
z2 = real_z.signals.values(:);
fprintf('  z: min=%.4f last=%.4f, Prop1.w last=%.2f rad/s\n', min(z2), z2(end), w2(end));
slope = (w2(end) - w1(end)) / 100;  % rad/s per Bias unit
fprintf('  민감도 dw/dBias = %.3f (rad/s)/unit\n', slope);

% 필요 Bias 역산: w(bias) 선형 근사
bias_new = 700 + (w_need - w1(end)) / slope;
fprintf('  => 필요 Bias Chassis = %.1f\n', bias_new);

fprintf('\n=== [3] Bias=%.1f 적용 후 검증 ===\n', bias_new);
set_param(biasBlk, 'Bias', num2str(bias_new, '%.4f'));
simOut = sim(mdl);
z3 = real_z.signals.values(:);
tz3 = real_z.time(:);
r3 = real_roll.signals.values(:);
w3 = prop1_w.signals.values(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z3), max(z3), z3(end));
fprintf('  roll: maxabs=%.3f deg last=%.3f deg\n', rad2deg(max(abs(r3))), rad2deg(r3(end)));
fprintf('  Prop1.w: last=%.2f rad/s (한계 7420 대비 여유 %.0f%%)\n', w3(end), 100*(1 - w3(end)/7420));
n = numel(tz3);
stride = max(1, floor(n/14));
fprintf('  z(t): ');
for idx = 1:stride:n
    fprintf('[%.1fs %.3f] ', tz3(idx), z3(idx));
end
fprintf('[%.1fs %.3f]\n', tz3(end), z3(end));

% 모델 저장은 안 하므로 Bias 변경은 이 세션에서만 유효 (영구 적용은 확인 후 별도로)
fprintf('\n(참고: 모델 저장 안 함 - Bias 변경은 세션 한정. 검증되면 영구 반영은 별도 진행.)\n');
