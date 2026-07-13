%% roll/pitch의 min/max/last 세 점 요약만으로는 "끝에서 진짜 수렴했는지"를
%% 판단할 수 없음 (last=4.175deg가 여러 다른 테스트에서 반복 등장 - 의심스러움).
%% 시뮬레이션 뒷부분(꼬리) 구간을 촘촘히 찍어서 실제로 0으로 수렴/진동/
%% 정상상태 오프셋 중 무엇인지 직접 확인.

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
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;  % 원래값(디폴트 Kdrag=0.01)

mdl = 'quadcopter_package_delivery';
load_system(mdl);
% Kdrag는 원래(기본, 0.01) 그대로 - 실제로 뜨고 roll -90도가 나오는 조건.

dt = 0.01;
T = 10;  % 5초 -> 10초로 늘려서 "5초가 부족해서 못 끝난 것"인지도 같이 확인
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
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
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

fprintf('=== 베이스라인 호버, StopTime=%ds로 연장해서 실행 ===\n', T);
simOut = sim(mdl);
t = real_roll.time(:);
roll = rad2deg(real_roll.signals.values(:));
pitch = rad2deg(real_pitch.signals.values(:));
z = real_z.signals.values(:);
x = real_x.signals.values(:);
y = real_y.signals.values(:);

fprintf('\n=== 시간별 x/y/z/roll/pitch 스냅샷 ===\n');
checkTimes = [0.5 1 1.5 2 2.5 3 4 5 6 7 8 9 9.5 9.9 10];
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  t=%5.2fs: x=%7.3f y=%7.3f z=%7.4f roll=%8.3f deg, pitch=%8.3f deg\n', t(idx), x(idx), y(idx), z(idx), roll(idx), pitch(idx));
end
fprintf('\n=== x/y 최대 이탈폭 ===\n');
fprintf('  x: min=%.3f max=%.3f last=%.3f (목표=0)\n', min(x), max(x), x(end));
fprintf('  y: min=%.3f max=%.3f last=%.3f (목표=0)\n', min(y), max(y), y(end));

% 마지막 1초/2초 구간 통계로 "수렴 vs 진동 vs 오프셋" 판정
tailMask1 = t >= (T-1);
tailMask2 = t >= (T-2);
fprintf('\n=== 마지막 1초 구간 (t=%d~%ds) 통계 ===\n', T-1, T);
fprintf('  roll : min=%.4f max=%.4f mean=%.4f std=%.4f\n', min(roll(tailMask1)), max(roll(tailMask1)), mean(roll(tailMask1)), std(roll(tailMask1)));
fprintf('  pitch: min=%.4f max=%.4f mean=%.4f std=%.4f\n', min(pitch(tailMask1)), max(pitch(tailMask1)), mean(pitch(tailMask1)), std(pitch(tailMask1)));

fprintf('\n=== 마지막 2초 구간 (t=%d~%ds) 통계 ===\n', T-2, T);
fprintf('  roll : min=%.4f max=%.4f mean=%.4f std=%.4f\n', min(roll(tailMask2)), max(roll(tailMask2)), mean(roll(tailMask2)), std(roll(tailMask2)));
fprintf('  pitch: min=%.4f max=%.4f mean=%.4f std=%.4f\n', min(pitch(tailMask2)), max(pitch(tailMask2)), mean(pitch(tailMask2)), std(pitch(tailMask2)));

fprintf('\n=== 판정 ===\n');
rollStd1 = std(roll(tailMask1));
pitchStd1 = std(pitch(tailMask1));
if rollStd1 < 0.01 && pitchStd1 < 0.01
    fprintf('  마지막 1초 표준편차가 매우 작음 -> 진동은 아니고, 특정 값에 "정착"한 상태.\n');
    fprintf('  단, 그 값이 0이 아니면 -> 정상상태 오프셋(전형적 P만 있고 I가 부족하거나, 트림이 안 맞는 경우) 문제.\n');
else
    fprintf('  마지막 1초에도 표준편차가 작지 않음 -> 아직도 진동 중 (수렴 안 됨). 5초/10초로는 부족하거나 근본적으로 불안정.\n');
end
