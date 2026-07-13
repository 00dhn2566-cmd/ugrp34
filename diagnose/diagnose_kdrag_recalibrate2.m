%% Kdrag 재보정 2차: limit_motor 스윕에서 "평형 속도에서 프로펠러 토크 = limit_motor"
%% 관계가 확인됨 (405.32@0.25, 512.78@0.40, 정확히 sqrt 비례).
%% 이 직접 측정으로 역산: 호버 속도 634 rad/s에서 실제(APC) 토크 0.0865 N·m가
%% 나오려면 Kdrag ≈ 0.597 이어야 함. (기존 4.2228은 5~7배 과대, 원본 0.01은 60배 과소)
%%
%% 검증 A: Kdrag=0.597 + limit_motor=0.0865 -> 평형속도가 634 rad/s 근처면 보정 성공
%% 검증 B: Kdrag=0.597 + limit_motor=0.25(원래) -> 전체 호버 테스트 (z가 1.0까지 뜨는지)

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

propeller.Kdrag = 0.597;
assignin('base', 'propeller', propeller);
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;
n_hover = sqrt(T_need / (propeller.Kthrust * air_rho * propeller.diameter^4));
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', num2str(n_hover, '%.4f'));
fprintf('Kdrag=0.597, Bias Chassis=%.4f (rev/s), 호버 필요속도=%.1f rad/s\n', n_hover, n_hover*2*pi);

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

set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element11','prop1_w'; 'In Bus Element6','prop1_T'; 'In Bus Element17','mot1_i'};
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

fprintf('\n=== [A] Kdrag=0.597, limit_motor=0.0865 (평형속도가 634 근처면 보정 성공) ===\n');
limit_motor = 0.0865;
try
    simOut = sim(mdl);
    w = prop1_w.signals.values(:);
    z = real_z.signals.values(:);
    fprintf('  Prop1.w: last=%.2f rad/s (예측: 634)\n', w(end));
    fprintf('  z: last=%.4f\n', z(end));
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(200,end)));
end

fprintf('\n=== [B] Kdrag=0.597, limit_motor=0.25 (원래값), 전체 호버 테스트 ===\n');
limit_motor = 0.25;
try
    simOut = sim(mdl);
    w = prop1_w.signals.values(:);
    z = real_z.signals.values(:);
    r = rad2deg(real_roll.signals.values(:));
    p = rad2deg(real_pitch.signals.values(:));
    thr = prop1_T.signals.values(:);
    ii = mot1_i.signals.values(:);
    t = real_z.time(:);
    fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));
    fprintf('  Prop1.w: min=%.2f max=%.2f last=%.2f rad/s\n', min(w), max(w), w(end));
    fprintf('  Prop1.thrust: last=%.3f N (호버 필요 ~5.58/prop)\n', thr(end));
    fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
    fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
    fprintf('  Mot1 전류: max=%.2f last=%.2f A\n', max(ii), ii(end));
    fprintf('\n  === 시간별 z/w 스냅샷 ===\n');
    for ct = 0:0.2:3
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%5.2fs: z=%7.4f w=%8.2f roll=%8.3f\n', t(idx), z(idx), w(idx), r(idx));
    end
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(200,end)));
end
