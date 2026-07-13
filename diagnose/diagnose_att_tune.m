%% 자세 게인 스윕: B'구성 + 위치루프 절단 + roll +5도 스텝, 게인 6세트 연속 평가
%% 각 세트의 최종값/상승시간/오버슈트/정착시간/타축 커플링/z 를 표로 출력.
%% 전제: CAD/Geometry 플레이트가 회전본. 규칙: 대상 미발견 시 error() 즉사.

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

% --- B'보정: 앵커 보정 Transform ---
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
fprintf('B보정 삽입 완료\n');

% --- 프로펠러 물리 보정 ---
propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

% --- 추력 경로 재스케일 + 고도 클램프 ---
sub = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem'];
set_param([sub '/Bias Chassis'], 'Bias', '56.5');
set_param([sub '/Bias Load'], 'Gain', '44.4*pkgSize(1)^3*pkgDensity');
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

% --- x,y만 ±0.15m saturation (위치루프는 어차피 절단하지만 배선 일관성 유지) ---
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
scopeBlk = [pc '/Scope'];
scopePh = get_param(scopeBlk, 'PortHandles');
if get_param(scopePh.Inport(2), 'Line') == -1
    add_line(pc, subPh.Outport(1), scopePh.Inport(2), 'autorouting', 'on');
end

% --- 위치 루프 절단 + roll 스텝 주입 ---
mc = [mdl '/Maneuver Controller'];
pcB = [mc '/Position Control'];
pph = get_param(pcB, 'PortHandles');
consts = {'5*pi/180', '0'};
for oi = 1:2
    l = get_param(pph.Outport(oi), 'Line');
    if l == -1
        error('PC Out%d 라인 없음 - 실행 무효', oi);
    end
    dsts = get_param(l, 'DstPortHandle');
    delete_line(l);
    cbName = sprintf('Step Cmd %d', oi);
    cb = [mc '/' cbName];
    if isempty(find_system(mc, 'SearchDepth', 1, 'Name', cbName))
        add_block('simulink/Sources/Constant', cb, 'Value', consts{oi});
    end
    cph3 = get_param(cb, 'PortHandles');
    for dd = dsts(:)'
        add_line(mc, cph3.Outport(1), dd, 'autorouting', 'on');
    end
end
fprintf('위치루프 절단 + 스텝 주입 완료 (Out1=5도, Out2=0)\n');

% --- 궤적/워크스페이스 ---
dt = 0.01;
T = 2;
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

% --- 로깅 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'};
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

% --- 게인 스윕 ---
% pidtune(wc=8: Kp=-351/Kd=-265, 링잉 큼 / wc=4: Kp=-62/Kd=-134) 주변 정밀 스윕
gainSets = [ ...
   -62    0  -134;   % pidtune wc=4
  -100    0  -150;
  -150    0  -200;
  -175    0  -130;   % wc=8의 절반
  -250    0  -280;
  -351    0  -265];  % pidtune wc=8 (링잉 기준)
fprintf('\n===================== 자세 게인 스윕 (roll +5도 스텝) =====================\n');
summ = {};
for gi = 1:size(gainSets, 1)
    kp_attitude = gainSets(gi,1);
    ki_attitude = gainSets(gi,2);
    kd_attitude = gainSets(gi,3);
    fprintf('\n--- 세트 %d: kp=%g ki=%g kd=%g ---\n', gi, kp_attitude, ki_attitude, kd_attitude);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message(1:min(150,end)));
        continue;
    end
    t = real_roll.time(:);
    r = rad2deg(real_roll.signals.values(:));
    pch = rad2deg(real_pitch.signals.values(:));
    cR = rad2deg(cmd_roll.signals.values(:));
    cP = rad2deg(cmd_pitch.signals.values(:));
    zv = real_z.signals.values(:);
    % 스텝 채널/목표 판별 (cmd 로그가 상수를 못 비추면 5도 가정)
    if mean(abs(cR)) > 0.5 || mean(abs(cP)) > 0.5
        if mean(abs(cR)) > mean(abs(cP))
            y = r; yc = mean(cR(t > 0.5)); oth = pch; othName = 'pitch';
        else
            y = pch; yc = mean(cP(t > 0.5)); oth = r; othName = 'roll';
        end
    else
        if max(abs(r)) > max(abs(pch))
            y = r; yc = 5; oth = pch; othName = 'pitch';
        else
            y = pch; yc = 5; oth = r; othName = 'roll';
        end
    end
    yfin = mean(y(t > 1.7));
    s = sign(yc); if s == 0; s = 1; end
    yn = s * y; tgt = abs(yc);
    i90 = find(yn >= 0.9 * tgt, 1);
    tr = NaN; if ~isempty(i90); tr = t(i90); end
    ovs = (max(yn) - tgt) / max(tgt, 0.1) * 100;
    okmask = abs(y - yc) < 0.5;
    ts = NaN;
    for ii = 1:numel(t)
        if all(okmask(ii:end)); ts = t(ii); break; end
    end
    fprintf('  최종 %+.2f도 (cmd %+.2f) | 상승 %.2fs | 오버슈트 %.0f%% | 정착 %.2fs | 커플링(%s) max %.2f | z구간 [%.2f %.2f]\n', ...
        yfin, yc, tr, ovs, ts, othName, max(abs(oth)), min(zv), max(zv));
    summ{end+1} = sprintf('kp=%-3g ki=%-2g kd=%-3g | 최종 %+6.2f | 상승 %5.2f | OS %4.0f%% | 정착 %5.2f | z최저 %5.2f', ...
        gainSets(gi,1), gainSets(gi,2), gainSets(gi,3), yfin, tr, ovs, ts, min(zv)); %#ok<AGROW>
end
fprintf('\n===================== 요약 =====================\n');
for i = 1:numel(summ)
    fprintf('  %s\n', summ{i});
end
