%% 가설: 프로펠러 2,3의 direction=Negative가 추력 부호를 뒤집어서
%% (2개 위로 + 2개 아래로 = 순추력 0) 모터가 최대로 돌아도 못 뜨는 상태.
%% [1] 현재 상태에서 4개 프로펠러 추력을 각각 실측해서 확인.
%% [2] 물리적으로 올바른 반전 적용: 프로펠러 2,3 Negative + 모터 2,3 회전도 반전
%%     (Motor Mixer의 Add5(w2)/Add7(w3) 부호 전체 반전) -> 반대피치 날개 x 반대회전
%%     = 추력 위, 반작용토크/자이로모멘트는 상쇄. 호버 성공하는지 검증.

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

% 프로펠러 2,3 방향 반전 (기존 [B] 상태)
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end

% 로깅: z(E2), roll(E4), 프로펠러 1~4 추력(E6,E7,E8,E9), 속도(E11,E10,E12,E13)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element6','T1'; 'In Bus Element7','T2'; ...
          'In Bus Element8','T3'; 'In Bus Element9','T4'; ...
          'In Bus Element11','W1'; 'In Bus Element10','W2'; ...
          'In Bus Element12','W3'; 'In Bus Element13','W4'};
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

fprintf('=== [1] 현재 상태(프로펠러 2,3 Negative, 모터 회전 그대로): 프로펠러별 추력 실측 ===\n');
simOut = sim(mdl);
fprintf('  z: last=%.4f\n', real_z.signals.values(end));
fprintf('  추력(N): T1=%.3f T2=%.3f T3=%.3f T4=%.3f (합=%.2f, 필요=22.3)\n', ...
    T1.signals.values(end), T2.signals.values(end), T3.signals.values(end), T4.signals.values(end), ...
    T1.signals.values(end)+T2.signals.values(end)+T3.signals.values(end)+T4.signals.values(end));
fprintf('  속도(rad/s): W1=%.1f W2=%.1f W3=%.1f W4=%.1f\n', ...
    W1.signals.values(end), W2.signals.values(end), W3.signals.values(end), W4.signals.values(end));

fprintf('\n=== [2] 모터 2,3 회전방향도 반전 (Motor Mixer Add5/Add7 부호 전체 반전) ===\n');
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
% w1=Add4, w2=Add5, w3=Add7, w4=Add6 (기존 세션에서 확인된 매핑)
add5_old = get_param([mixer '/Add5'], 'Inputs');
add7_old = get_param([mixer '/Add7'], 'Inputs');
flipSigns = @(s) strrep(strrep(strrep(s, '+', 'X'), '-', '+'), 'X', '-');
set_param([mixer '/Add5'], 'Inputs', flipSigns(add5_old));
set_param([mixer '/Add7'], 'Inputs', flipSigns(add7_old));
fprintf('  Add5(w2): %s -> %s\n', add5_old, get_param([mixer '/Add5'], 'Inputs'));
fprintf('  Add7(w3): %s -> %s\n', add7_old, get_param([mixer '/Add7'], 'Inputs'));

simOut = sim(mdl);
z2 = real_z.signals.values(:);
tz2 = real_z.time(:);
r2 = real_roll.signals.values(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z2), max(z2), z2(end));
fprintf('  roll: maxabs=%.3f deg last=%.3f deg\n', rad2deg(max(abs(r2))), rad2deg(r2(end)));
fprintf('  추력(N): T1=%.3f T2=%.3f T3=%.3f T4=%.3f (합=%.2f, 필요=22.3)\n', ...
    T1.signals.values(end), T2.signals.values(end), T3.signals.values(end), T4.signals.values(end), ...
    T1.signals.values(end)+T2.signals.values(end)+T3.signals.values(end)+T4.signals.values(end));
fprintf('  속도(rad/s): W1=%.1f W2=%.1f W3=%.1f W4=%.1f\n', ...
    W1.signals.values(end), W2.signals.values(end), W3.signals.values(end), W4.signals.values(end));
n = numel(tz2);
stride = max(1, floor(n/14));
fprintf('  z(t): ');
for idx = 1:stride:n
    fprintf('[%.1fs %.3f] ', tz2(idx), z2(idx));
end
fprintf('[%.1fs %.3f]\n', tz2(end), z2(end));

fprintf('\n(참고: 모델 저장 안 함 - Mixer/방향 변경은 세션 한정. 검증되면 영구 반영은 별도 진행.)\n');
