%% 모터 2,3이 실제로 음의 방향으로 도는지 확인 (반대회전 실현 여부 검증):
%% 믹서 반전으로 모터 2,3은 음수 ref를 받아야 하는데, combined 테스트에서
%% W1~W4가 전부 +1076으로 나왔음 -> 반대회전이 물리적으로 안 되고 있을 의심.
%% W1~W4(부호 포함)와 믹서 Add5/Add7 출력(ref)을 같이 로깅.
%% 기반 조건은 thrust_path_rescale과 동일:
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

% --- B'보정: 회전된 FX450 플레이트 전제 + 앵커 보정 Transform 삽입 ---
% 전제: CAD/Geometry의 plate_top/bottom이 Rx(+90) 회전본이어야 함 (rotate_step.py 적용본).
% 보정값은 diagnose_plate_offset_fix.m가 산출: [-29.89 +29.25 +0.76] mm (잔차 1.16mm 검증됨)
bb = [mdl '/Quadcopter/Body/Body'];
p = bb;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
pbBlk = [bb '/plate_bottom'];
pbPh = get_param(pbBlk, 'PortHandles');
pbConn = [pbPh.LConn pbPh.RConn];
anchorPort = -1; bNodePort = -1; anchorLine = -1;
for ci = 1:numel(pbConn)
    cp2 = pbConn(ci);
    l = get_param(cp2, 'Line');
    if l == -1; continue; end
    cands = [get_param(l,'SrcPortHandle'), get_param(l,'DstPortHandle')];
    othPort = -1;
    for c2 = cands
        if c2 > 0 && c2 ~= cp2; othPort = c2; end
    end
    if othPort == -1; continue; end
    othName = strtrim(regexprep(get_param(get_param(othPort,'Parent'), 'Name'), '\s+', ' '));
    if strcmp(othName, 'B')
        anchorPort = cp2; bNodePort = othPort; anchorLine = l;
    end
end
if anchorPort == -1
    error('plate_bottom <-> B 라인을 못 찾음 - B보정 적용 불가, 실행 무효');
end
compBlk = [bb '/Plate Anchor Comp'];
if isempty(find_system(bb, 'SearchDepth', 1, 'Name', 'Plate Anchor Comp'))
    add_block('sm_lib/Frames and Transforms/Rigid Transform', compBlk);
end
set_param(compBlk, 'Orientation', 'right');
pbPos = get_param(pbBlk, 'Position');
set_param(compBlk, 'Position', pbPos + [-100 80 -100 80]);
set_param(compBlk, 'TranslationMethod', 'Cartesian');
set_param(compBlk, 'TranslationCartesianOffsetUnits', 'mm');
set_param(compBlk, 'TranslationCartesianOffset', '[-30.7741 30.1152 0.78248]');
delete_line(anchorLine);
cph2 = get_param(compBlk, 'PortHandles');
if numel(cph2.RConn) >= 1
    pB = cph2.LConn(1); pF = cph2.RConn(1);
else
    pB = cph2.LConn(1); pF = cph2.LConn(2);
end
add_line(bb, bNodePort, pB, 'autorouting', 'on');
add_line(bb, pF, anchorPort, 'autorouting', 'on');
fprintf('B보정 삽입 완료: Plate Anchor Comp = [-30.7741 30.1152 0.78248] mm (잔차 0.04mm 검증)\n');

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
% 프로펠러 direction 오버라이드 제거 - 공장 상태(전부 Positive) 그대로 사용.
% 공장 설계: 모터 2,3 하류 내장반전(음의 회전) + Thrust Direction 게인(-1)이
% 이미 반대회전+추력방향을 완결적으로 처리하고 있음.
for p = 1:4
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    fprintf('Propeller %d direction(공장값 유지) = %s\n', p, get_param(blk, 'direction'));
end
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
% 믹서 반전 제거: 원래(디스크) 부호 그대로 사용.
% 근거: 모터가 ref의 절대값을 추종함이 확인됨(부호 무시) -> 원래 부호면
% 차등 성분들이 올바른 기하학적 부호로 크기에 반영됨.
fprintf('Mixer 원래 부호 유지: Add4=%s Add5=%s Add6=%s Add7=%s\n', ...
    get_param([mixer '/Add4'],'Inputs'), get_param([mixer '/Add5'],'Inputs'), ...
    get_param([mixer '/Add6'],'Inputs'), get_param([mixer '/Add7'],'Inputs'));

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
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'; ...
          'In Bus Element11','W1'; 'In Bus Element10','W2'; 'In Bus Element12','W3'; 'In Bus Element13','W4'; ...
          'In Bus Element6','T1'; 'In Bus Element7','T2'; 'In Bus Element8','T3'; 'In Bus Element9','T4'};
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

% 믹서 Add4~Add7 출력(모터별 ref) 태핑
for aPair = {{'Add5','ref_w2'},{'Add7','ref_w3'},{'Add4','ref_w1'},{'Add6','ref_w4'}}
    aName = aPair{1}{1}; vName = aPair{1}{2};
    aph = get_param([mixer '/' aName], 'PortHandles');
    twName = ['To Workspace ' vName];
    if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', twName))
        twBlk2 = [mixer '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk2, 'VariableName', vName, 'SaveFormat', 'StructureWithTime');
        twPh2 = get_param(twBlk2, 'PortHandles');
        add_line(mixer, aph.Outport(1), twPh2.Inport(1), 'autorouting', 'on');
    end
end

fprintf('=== 완전 공장구성 + 보정값 호버 테스트 (direction 오버라이드 제거) ===\n');
try
    simOut = sim(mdl);
    t = real_z.time(:);
    z = real_z.signals.values(:);
    r = rad2deg(real_roll.signals.values(:));
    w1 = W1.signals.values(:); w2 = W2.signals.values(:); w3 = W3.signals.values(:); w4 = W4.signals.values(:);
    r1 = ref_w1.signals.values(:); r2 = ref_w2.signals.values(:); r3 = ref_w3.signals.values(:); r4 = ref_w4.signals.values(:);
    fprintf('  ref(last):  w1=%.1f w2=%.1f w3=%.1f w4=%.1f\n', r1(end), r2(end), r3(end), r4(end));
    fprintf('  meas(last): W1=%.1f W2=%.1f W3=%.1f W4=%.1f rad/s\n', w1(end), w2(end), w3(end), w4(end));
    fprintf('  추력(last): T1=%.3f T2=%.3f T3=%.3f T4=%.3f N\n', T1.signals.values(end), T2.signals.values(end), T3.signals.values(end), T4.signals.values(end));
    fprintf('\n=== 시간별: ref vs meas (부호가 관건) ===\n');
    for ct = [0.1 0.25 0.5 1.0 2.0 3.0 4.0 5.0]
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%4.2f: ref2=%9.1f W2=%8.1f | ref3=%9.1f W3=%8.1f | z=%6.3f roll=%7.2f\n', ...
            t(idx), r2(idx), w2(idx), r3(idx), w3(idx), z(idx), r(idx));
    end

    cR = rad2deg(cmd_roll.signals.values(:));
    cP = rad2deg(cmd_pitch.signals.values(:));
    pch = rad2deg(real_pitch.signals.values(:));
    fprintf('\n=== 시간별: cmd 자세 vs 실제 자세 ===\n');
    fprintf('  %5s | %8s %8s | %8s %8s | %6s\n', 't', 'cmdR', 'realR', 'cmdP', 'realP', 'z');
    for ct = 0:0.25:5
        [~, idx] = min(abs(t - ct));
        fprintf('  %5.2f | %8.2f %8.2f | %8.2f %8.2f | %6.3f\n', ...
            t(idx), cR(idx), r(idx), cP(idx), pch(idx), z(idx));
    end
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(300,end)));
end
