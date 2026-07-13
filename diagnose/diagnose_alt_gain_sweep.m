%% 시스템이 드디어 안정됨(프로펠러 방향 수정 + 보수적 게인). 남은 문제는 고도
%% 정상상태 오차(1.0m 목표인데 0.144m에서 유지 - 약한 P게인 + 적분기 없음).
%% 고도 게인 3세트를 한 세션에서 스윕해서 z가 1.0m을 잡는 조합을 찾는다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% 자세/위치는 안정이 확인된 보수적 게인 고정
kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
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

% 프로펠러 방향 수정 적용 (roll 안정화에 필수임이 확인됨)
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end

% 로깅: roll(E4)/pitch(E3)/yaw(E5)/z(E2), StructureWithTime으로 시간 정렬 보장
scope = [mdl '/Scope'];
sigMap = {'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element5','real_yaw';  'In Bus Element2','real_z'};
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

% 고도 게인 후보 (kp, ki, kd)
cands = [0.5, 0.1, 0.3;
         2.0, 0.5, 1.0;
         8.0, 2.0, 4.0];

for c = 1:size(cands,1)
    kp_altitude = cands(c,1);
    ki_altitude = cands(c,2);
    kd_altitude = cands(c,3);
    fprintf('\n=== [%d] kp_alt=%g ki_alt=%g kd_alt=%g ===\n', c, kp_altitude, ki_altitude, kd_altitude);
    try
        simOut = sim(mdl);
        z = real_z.signals.values(:);  tz = real_z.time(:);
        r = real_roll.signals.values(:);
        p_ = real_pitch.signals.values(:);
        yw = real_yaw.signals.values(:);
        fprintf('  z: min=%.4f max=%.4f last=%.4f (목표 1.0m)\n', min(z), max(z), z(end));
        fprintf('  roll: maxabs=%.3f deg last=%.3f deg\n', rad2deg(max(abs(r))), rad2deg(r(end)));
        fprintf('  pitch: maxabs=%.3f deg last=%.3f deg\n', rad2deg(max(abs(p_))), rad2deg(p_(end)));
        fprintf('  yaw: maxabs=%.3f deg last=%.3f deg\n', rad2deg(max(abs(yw))), rad2deg(yw(end)));
        % z 궤적 간략 출력 (0.5s 간격 근사)
        n = numel(tz);
        stride = max(1, floor(n/14));
        fprintf('  z(t): ');
        for idx = 1:stride:n
            fprintf('[%.1fs %.3f] ', tz(idx), z(idx));
        end
        fprintf('[%.1fs %.3f]\n', tz(end), z(end));
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
    end
end
