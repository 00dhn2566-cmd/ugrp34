%% 베이스라인(IC 없음) 호버 테스트에서 roll이 -90도로 튀는 첫 방아쇠가
%% 위치 PID(kp_position 등)의 과도한 게인 때문인지 확인.
%% Kdrag=4.222841(실측 검증값), Kthrust=0.1072(그대로) 적용 상태에서,
%% 위치 게인만 훨씬 낮춰서(원래의 1/10) 같은 베이스라인 테스트 재실행.

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

mdl = 'quadcopter_package_delivery';
load_system(mdl);
% Kdrag는 원래(기본, 0.01) 값 그대로 사용 - 이게 실제로 뜨고 roll -90도가
% 나왔던 그 설정이므로, 위치 게인의 영향을 보려면 이 조건에서 봐야 함.

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
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
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

fprintf('=== [A] 원래 위치게인(kp_position=8, kd=3.2)으로 베이스라인 호버 ===\n');
kp_position = 8; ki_position = 0.04; kd_position = 3.2;  % quadcopter_package_parameters.m 원래값
simOut = sim(mdl);
rA = real_roll.signals.values(:); pA = real_pitch.signals.values(:); zA = real_z.signals.values(:);
fprintf('  z: last=%.4f, roll: min=%.3f max=%.3f last=%.3f deg, pitch: min=%.3f max=%.3f last=%.3f deg\n', ...
    zA(end), rad2deg(min(rA)), rad2deg(max(rA)), rad2deg(rA(end)), rad2deg(min(pA)), rad2deg(max(pA)), rad2deg(pA(end)));

fprintf('\n=== [B] 위치게인 1/10로 낮춰서(kp_position=0.8, kd=0.32) 베이스라인 호버 ===\n');
kp_position = 0.8; ki_position = 0.004; kd_position = 0.32;
simOut = sim(mdl);
rB = real_roll.signals.values(:); pB = real_pitch.signals.values(:); zB = real_z.signals.values(:);
fprintf('  z: last=%.4f, roll: min=%.3f max=%.3f last=%.3f deg, pitch: min=%.3f max=%.3f last=%.3f deg\n', ...
    zB(end), rad2deg(min(rB)), rad2deg(max(rB)), rad2deg(rB(end)), rad2deg(min(pB)), rad2deg(max(pB)), rad2deg(pB(end)));

fprintf('\n=== [C] 위치게인 1/100로 더 낮춰서(kp_position=0.08, kd=0.032) 베이스라인 호버 ===\n');
kp_position = 0.08; ki_position = 0.0004; kd_position = 0.032;
simOut = sim(mdl);
rC = real_roll.signals.values(:); pC = real_pitch.signals.values(:); zC = real_z.signals.values(:);
fprintf('  z: last=%.4f, roll: min=%.3f max=%.3f last=%.3f deg, pitch: min=%.3f max=%.3f last=%.3f deg\n', ...
    zC(end), rad2deg(min(rC)), rad2deg(max(rC)), rad2deg(rC(end)), rad2deg(min(pC)), rad2deg(max(pC)), rad2deg(pC(end)));

fprintf('\n=== [D] 위치게인 10배로 올려서(kp_position=80, kd=32) 베이스라인 호버 ===\n');
kp_position = 80; ki_position = 0.4; kd_position = 32;
simOut = sim(mdl);
rD = real_roll.signals.values(:); pD = real_pitch.signals.values(:); zD = real_z.signals.values(:);
fprintf('  z: last=%.4f, roll: min=%.3f max=%.3f last=%.3f deg, pitch: min=%.3f max=%.3f last=%.3f deg\n', ...
    zD(end), rad2deg(min(rD)), rad2deg(max(rD)), rad2deg(rD(end)), rad2deg(min(pD)), rad2deg(max(pD)), rad2deg(pD(end)));

fprintf('\n=== 결론 ===\n');
fprintf('  위치게인을 낮출수록 roll 최대 발산폭이 줄어들고, 올릴수록 커지면 -> 위치 PID가 방아쇠.\n');
fprintf('  게인과 무관하게 비슷하면 -> 위치 PID와 무관한 다른 원인.\n');
