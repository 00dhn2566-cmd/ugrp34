%% "진짜 원본" 상태(프로펠러 direction 전부 기본값 Positive, Motor Mixer 부호도 원본)로
%% z/roll/pitch/yaw와 4개 프로펠러 추력을 전부 동시에 로깅.
%% 지금까지 z를 로깅한 테스트는 전부 프로펠러 방향을 건드린 이후였어서,
%% 이 조합(순수 원본)으로 실제로 뜨는지 자체를 아직 한 번도 확인한 적이 없음.
%% 순추력이 정상(4개 다 양수, 합이 필요 추력 이상)인지도 같이 확인.

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

% 프로펠러 방향/Mixer 부호는 절대 안 건드림 - 로드된 그대로(원본) 사용.
% 확인용으로 현재 값만 출력.
propBlocks = {[mdl '/Quadcopter/Propeller 1/Thrust and Drag/Aerodynamic Propeller'], ...
              [mdl '/Quadcopter/Propeller 2/Thrust and Drag/Aerodynamic Propeller'], ...
              [mdl '/Quadcopter/Propeller 3/Thrust and Drag/Aerodynamic Propeller'], ...
              [mdl '/Quadcopter/Propeller 4/Thrust and Drag/Aerodynamic Propeller']};
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
fprintf('=== 원본 상태 확인 ===\n');
for i = 1:4
    fprintf('  Prop%d direction = %s\n', i, get_param(propBlocks{i}, 'direction'));
end
fprintf('  Add4(w1)=%s Add5(w2)=%s Add6(w4)=%s Add7(w3)=%s\n', ...
    get_param([mixer '/Add4'],'Inputs'), get_param([mixer '/Add5'],'Inputs'), ...
    get_param([mixer '/Add6'],'Inputs'), get_param([mixer '/Add7'],'Inputs'));

% 로깅: z(E2), roll(E4), pitch(E3), yaw(E5), 프로펠러 1~4 추력(E6,E7,E8,E9)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element6','T1'; 'In Bus Element7','T2'; ...
          'In Bus Element8','T3'; 'In Bus Element9','T4'};
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

fprintf('\n=== 호버 테스트 실행 (원본 그대로) ===\n');
simOut = sim(mdl);

z = real_z.signals.values(:);  tz = real_z.time(:);
r = real_roll.signals.values(:);
p = real_pitch.signals.values(:);
y = real_yaw.signals.values(:);

fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z), max(z), z(end));
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r)), rad2deg(max(r)), rad2deg(r(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p)), rad2deg(max(p)), rad2deg(p(end)));
fprintf('  yaw: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(y)), rad2deg(max(y)), rad2deg(y(end)));
fprintf('  추력(N) last: T1=%.3f T2=%.3f T3=%.3f T4=%.3f (합=%.2f)\n', ...
    T1.signals.values(end), T2.signals.values(end), T3.signals.values(end), T4.signals.values(end), ...
    T1.signals.values(end)+T2.signals.values(end)+T3.signals.values(end)+T4.signals.values(end));

fprintf('\n=== z(t), roll(t) 다운샘플 (약 20포인트, 정확한 시간 정렬) ===\n');
n = numel(tz);
stride = max(1, floor(n/20));
for idx = 1:stride:n
    fprintf('  t=%6.3f  z=%7.4f m  roll=%9.4f deg\n', tz(idx), z(idx), rad2deg(r(idx)));
end
fprintf('  t=%6.3f  z=%7.4f m  roll=%9.4f deg (마지막)\n', tz(end), z(end), rad2deg(r(end)));

[minVal, minIdx] = min(r);
fprintf('\n  roll 최소값 지점: t=%.4f, roll=%.4f deg\n', tz(min(minIdx,numel(tz))), rad2deg(minVal));
