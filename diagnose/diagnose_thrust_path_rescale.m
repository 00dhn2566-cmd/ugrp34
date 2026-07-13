%% 추력 경로 전체를 새(현실) 추력맵에 맞게 재스케일:
%% 모터 ref = 2pi x (고도PID + Bias Chassis + (load+1) x BiasLoadGain x 패키지질량)
%% - Bias Chassis: 700 -> 56.5 (기체분: 101 rev/s x 1.2726/2.2726)
%% - Bias Load 상수: 260 -> 44.4 (패키지분: 101 rev/s x 1/2.2726 / 1kg)
%% - 고도 PID 출력(cmd)에 ±30 rev/s 클램프 삽입 (992까지 폭주하던 것 차단)
%% + 기존 확정 수정: Kthrust=9.79, Kdrag=0.597, 프롭2,3+믹서 반전, x,y sat ±0.15m

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

% --- 프로펠러 물리 보정 ---
propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

% --- 추력 경로 재스케일 ---
sub = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem'];
set_param([sub '/Bias Chassis'], 'Bias', '56.5');
set_param([sub '/Bias Load'], 'Gain', '44.4*pkgSize(1)^3*pkgDensity');

% 고도 PID 출력(cmd 인포트 신호)에 ±30 클램프 삽입
cmdBlk = [sub '/cmd'];
bcBlk = [sub '/Bias Chassis'];
cph = get_param(cmdBlk, 'PortHandles');
bph = get_param(bcBlk, 'PortHandles');
oldLine = get_param(cph.Outport(1), 'Line');
if oldLine ~= -1; delete_line(oldLine); end
satBlk = [sub '/Alt Cmd Sat'];
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Alt Cmd Sat'))
    add_block('simulink/Discontinuities/Saturation', satBlk, 'UpperLimit', '30', 'LowerLimit', '-30');
end
sph = get_param(satBlk, 'PortHandles');
add_line(sub, cph.Outport(1), sph.Inport(1), 'autorouting', 'on');
add_line(sub, sph.Outport(1), bph.Inport(1), 'autorouting', 'on');

% --- 프로펠러 2,3 + 믹서 반전 ---
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
flipSigns = @(s) strrep(strrep(strrep(s, '+', 'X'), '-', '+'), 'X', '-');
set_param([mixer '/Add5'], 'Inputs', flipSigns(get_param([mixer '/Add5'], 'Inputs')));
set_param([mixer '/Add7'], 'Inputs', flipSigns(get_param([mixer '/Add7'], 'Inputs')));

% --- x,y만 ±0.15m saturation ---
pc = [mdl '/Maneuver Controller/Position Control'];
subBlk = [pc '/Subtract2'];
pidBlk = [pc '/PID Controller'];
subPh = get_param(subBlk, 'PortHandles');
pidPh = get_param(pidBlk, 'PortHandles');
oldLine = get_param(subPh.Outport(1), 'Line');
if oldLine ~= -1; delete_line(oldLine); end
demuxBlk = [pc '/PosErr Demux'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Demux'))
    add_block('simulink/Signal Routing/Demux', demuxBlk, 'Outputs', '3');
end
satXBlk = [pc '/PosErr Sat X'];
satYBlk = [pc '/PosErr Sat Y'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat X'))
    add_block('simulink/Discontinuities/Saturation', satXBlk, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
end
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat Y'))
    add_block('simulink/Discontinuities/Saturation', satYBlk, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
end
muxBlk = [pc '/PosErr Mux'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Mux'))
    add_block('simulink/Signal Routing/Mux', muxBlk, 'Inputs', '3');
end
demuxPh = get_param(demuxBlk, 'PortHandles');
satXPh = get_param(satXBlk, 'PortHandles');
satYPh = get_param(satYBlk, 'PortHandles');
muxPh = get_param(muxBlk, 'PortHandles');
add_line(pc, subPh.Outport(1), demuxPh.Inport(1), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(1), satXPh.Inport(1), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(2), satYPh.Inport(1), 'autorouting', 'on');
add_line(pc, satXPh.Outport(1), muxPh.Inport(1), 'autorouting', 'on');
add_line(pc, satYPh.Outport(1), muxPh.Inport(2), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(3), muxPh.Inport(3), 'autorouting', 'on');
add_line(pc, muxPh.Outport(1), pidPh.Inport(1), 'autorouting', 'on');
% Scope 분기 재연결
scopeBlk = [pc '/Scope'];
scopePh = get_param(scopeBlk, 'PortHandles');
if get_param(scopePh.Inport(2), 'Line') == -1
    add_line(pc, subPh.Outport(1), scopePh.Inport(2), 'autorouting', 'on');
end

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
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'; 'In Bus Element11','W1'};
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

fprintf('=== 추력경로 재스케일 호버 테스트 (Bias 56.5 + BiasLoad 44.4 + AltPID ±30 클램프) ===\n');
try
    simOut = sim(mdl);
    t = real_z.time(:);
    x = real_x.signals.values(:); y = real_y.signals.values(:); z = real_z.signals.values(:);
    r = rad2deg(real_roll.signals.values(:)); p = rad2deg(real_pitch.signals.values(:)); yw = rad2deg(real_yaw.signals.values(:));
    cR = rad2deg(cmd_roll.signals.values(:)); cP = rad2deg(cmd_pitch.signals.values(:));
    w = W1.signals.values(:);
    fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));
    fprintf('  x: min=%.3f max=%.3f last=%.3f / y: min=%.3f max=%.3f last=%.3f\n', min(x), max(x), x(end), min(y), max(y), y(end));
    fprintf('  roll: min=%.2f max=%.2f last=%.2f / pitch: min=%.2f max=%.2f last=%.2f deg\n', min(r), max(r), r(end), min(p), max(p), p(end));
    fprintf('  yaw: min=%.2f max=%.2f last=%.2f deg\n', min(yw), max(yw), yw(end));
    fprintf('  W1: min=%.1f max=%.1f last=%.1f rad/s (호버 목표 ~634)\n', min(w), max(w), w(end));
    fprintf('\n=== 시간별 스냅샷 ===\n');
    for ct = 0:0.25:5
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%5.2f: x=%7.3f y=%7.3f z=%7.4f | roll=%8.2f pitch=%8.2f yaw=%8.2f | w=%7.1f\n', ...
            t(idx), x(idx), y(idx), z(idx), r(idx), p(idx), yw(idx), w(idx));
    end
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(300,end)));
end
