%% 최종 보정 조합 테스트:
%% - 블록 내부 추력 공식이 표준 공식의 1/91임이 실측으로 확인됨
%%   (Kthrust 38배 -> 추력 정확히 38배 비례 확인, 여러 속도에서 비율 일정)
%% - 따라서 블록용 Kthrust = 0.1072 x 91.3 = 9.79 (실제 634 rad/s에서 5.58N/prop)
%% - Kdrag = 0.597 (평형속도 측정으로 검증: 632 vs 예측 634)
%% - Bias Chassis = 100.98 rev/s (호버 피드포워드)
%% - 프로펠러 2,3 Negative + Motor Mixer Add5/Add7 반전 (6차 수정)
%% - 보수적 게인
%% => 현실적인 모터 속도(634 rad/s)에서 물리적으로 일관된 호버가 되는지 확인.

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

% --- 최종 보정값 ---
propeller.Kthrust = 9.79;   % 블록 공식 보정 (0.1072 x 91.3)
propeller.Kdrag   = 0.597;  % 평형속도 실측 검증값
assignin('base', 'propeller', propeller);

% 호버 피드포워드: 실제 필요 속도 634 rad/s = 100.98 rev/s
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', '100.98');

% --- 프로펠러 2,3 방향 + Motor Mixer 부호 반전 (6차 수정) ---
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
flipSigns = @(s) strrep(strrep(strrep(s, '+', 'X'), '-', '+'), 'X', '-');
set_param([mixer '/Add5'], 'Inputs', flipSigns(get_param([mixer '/Add5'], 'Inputs')));
set_param([mixer '/Add7'], 'Inputs', flipSigns(get_param([mixer '/Add7'], 'Inputs')));

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
          'In Bus Element11','W1'; 'In Bus Element17','mot1_i'};
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

fprintf('=== 최종 보정 조합 호버 테스트 (Kthrust=9.79, Kdrag=0.597, Bias=100.98, 프롭/믹서 수정) ===\n');
try
    simOut = sim(mdl);
    z = real_z.signals.values(:);
    r = rad2deg(real_roll.signals.values(:));
    p = rad2deg(real_pitch.signals.values(:));
    w = W1.signals.values(:);
    ii = mot1_i.signals.values(:);
    t = real_z.time(:);
    fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));
    fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
    fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
    fprintf('  W1: last=%.1f rad/s (예상 ~634)\n', w(end));
    fprintf('  추력(N, last): T1=%.3f T2=%.3f T3=%.3f T4=%.3f (합=%.2f, 필요=22.3)\n', ...
        T1.signals.values(end), T2.signals.values(end), T3.signals.values(end), T4.signals.values(end), ...
        T1.signals.values(end)+T2.signals.values(end)+T3.signals.values(end)+T4.signals.values(end));
    fprintf('  Mot1 전류: max=%.2f last=%.2f A\n', max(ii), ii(end));
    fprintf('\n=== 시간별 z/roll/pitch/W1 스냅샷 ===\n');
    for ct = 0:0.25:5
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%5.2fs: z=%7.4f roll=%8.3f pitch=%8.3f w=%8.1f\n', t(idx), z(idx), r(idx), p(idx), w(idx));
    end
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(300,end)));
end
