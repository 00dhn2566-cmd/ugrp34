%% propeller.Kthrust(=0.1072, 원래 MathWorks 예제 값 그대로)를 실제 1045 프로펠러
%% 데이터(A2212 1000KV+1045+3S 실측 최대추력 ~800g=7.848N @ 최대속도 1162.4rad/s)
%% 기준으로 재보정. 재보정 후:
%%  [1] 호버에 필요한 모터 속도가 물리적으로 달성 가능한 범위로 내려오는지
%%  [2] pitch에 10도 IC 외란을 줬을 때 발산 폭(과소감쇠)이 줄어드는지
%% 확인.

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
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element6','T1'; 'In Bus Element11','W1'};
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

fprintf('=== [기준] 원래 Kthrust=%.4f 로 호버 테스트 ===\n', propeller.Kthrust);
simOut = sim(mdl);
z0 = real_z.signals.values(:); w0 = W1.signals.values(:);
fprintf('  z: last=%.4f, Prop1.w: last=%.2f rad/s\n', z0(end), w0(end));

% Kthrust 재보정 (실측 800g@3S 데이터 기반, 위 계산의 38.25배)
propeller.Kthrust = 0.1072 * 38.25;
fprintf('\n=== [재보정] Kthrust=%.4f 로 재적용 ===\n', propeller.Kthrust);
assignin('base', 'propeller', propeller);

fprintf('\n=== [1] 재보정 후 호버 테스트 (IC 없음) ===\n');
simOut = sim(mdl);
z1 = real_z.signals.values(:); r1 = real_roll.signals.values(:);
p1 = real_pitch.signals.values(:); w1v = W1.signals.values(:); T1v = T1.signals.values(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z1), max(z1), z1(end));
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r1)), rad2deg(max(r1)), rad2deg(r1(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p1)), rad2deg(max(p1)), rad2deg(p1(end)));
fprintf('  Prop1.w: last=%.2f rad/s (실제 한계 1162.4 대비 %.0f%%), Prop1.thrust last=%.3f N\n', ...
    w1v(end), 100*w1v(end)/1162.4, T1v(end));

% Kthrust가 ~38.25배 커졌으니, T=Kthrust*w^2 관계상 국소 민감도는 대략 sqrt(38.25)~6.2배
% 정도 커진 것으로 보고, 고도/자세 게인을 6배 낮춰서 재시도.
scaleDown = 6.2;
kp_altitude = kp_altitude / scaleDown; ki_altitude = ki_altitude / scaleDown; kd_altitude = kd_altitude / scaleDown;
kp_attitude = kp_attitude / scaleDown; kd_attitude = kd_attitude / scaleDown;
kp_position = kp_position / scaleDown; kd_position = kd_position / scaleDown;
fprintf('\n=== [3] 게인을 %.1f배 낮춘 뒤 재보정 Kthrust로 호버 테스트 (IC 없음) ===\n', scaleDown);
fprintf('  kp_altitude=%.4f kd_altitude=%.4f kp_attitude=%.4f kd_attitude=%.4f\n', ...
    kp_altitude, kd_altitude, kp_attitude, kd_attitude);
simOut = sim(mdl);
z3 = real_z.signals.values(:); r3 = real_roll.signals.values(:); p3 = real_pitch.signals.values(:);
w3 = W1.signals.values(:);
fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z3), max(z3), z3(end));
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r3)), rad2deg(max(r3)), rad2deg(r3(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p3)), rad2deg(max(p3)), rad2deg(p3(end)));
fprintf('  Prop1.w: last=%.2f rad/s (실제 한계 1162.4 대비 %.0f%%)\n', w3(end), 100*w3(end)/1162.4);

% pitch IC 10도 외란 테스트 (낮춘 게인 + 재보정 Kthrust 유지)
sph = [mdl '/Quadcopter/6 DOF/Joints/Spherical Joint'];
set_param(sph, 'PositionTargetRotationSequenceAngles', '[0,10,0]');
fprintf('\n=== [4] 낮춘 게인 + 재보정 Kthrust로 pitch IC=10도 외란 테스트 ===\n');
simOut = sim(mdl);
r4 = real_roll.signals.values(:); p4 = real_pitch.signals.values(:);
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r4)), rad2deg(max(r4)), rad2deg(r4(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p4)), rad2deg(max(p4)), rad2deg(p4(end)));
fprintf('  (원래 Kthrust+게인 조합에서는 pitch가 88.8도까지 튀었음 - 비교용)\n');
