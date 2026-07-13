%% 검증된 구성을 모델 파일에 영구 저장 (save_system) + 무수정 검증 비행
%% 굽는 내용 (11차 세션 검증 완료):
%%   1) Plate Anchor Comp [-30.7741 30.1152 0.78248]mm (B <-> plate_bottom)
%%   2) Bias Chassis 56.5 / Bias Load 44.4*pkgSize^3*pkgDensity
%%   3) 고도 PID 출력 ±30 rev/s 클램프 (Alt Cmd Sat)
%%   4) 위치오차 x,y ±0.15m saturation (PosErr Demux/Sat/Mux)
%% 게인/Kthrust/Kdrag는 quadcopter_package_parameters.m에 이미 반영됨.
%% 저장 후: 모델을 닫고 다시 열어 아무 수정 없이 호버 시뮬 -> 파일 자체 검증.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- (1) 앵커 보정 ---
bb = [mdl '/Quadcopter/Body/Body'];
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
    error('plate_bottom <-> B 라인을 못 찾음 - 굽기 중단');
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
fprintf('[1/4] 앵커 보정 삽입\n');

% --- (2) 추력 경로 재스케일 ---
sub = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem'];
set_param([sub '/Bias Chassis'], 'Bias', '56.5');
set_param([sub '/Bias Load'], 'Gain', '44.4*pkgSize(1)^3*pkgDensity');
fprintf('[2/4] Bias 재스케일\n');

% --- (3) 고도 클램프 ---
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
fprintf('[3/4] 고도 클램프\n');

% --- (4) x,y saturation ---
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
fprintf('[4/4] x,y saturation\n');

% --- 저장 ---
save_system(mdl);
fprintf('=== save_system 완료: %s ===\n', which([mdl '.slx']));
close_system(mdl, 0);

% --- 무수정 검증: 다시 열어 그대로 호버 ---
fprintf('\n=== 무수정 검증: 파일 재로드 후 호버 8초 ===\n');
load_system(mdl);
dt = 0.01; T = 8; N = round(T/dt) + 1;
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
% 로깅
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
sim(mdl);
t = real_roll.time(:);
r = rad2deg(real_roll.signals.values(:));
pch = rad2deg(real_pitch.signals.values(:));
zv = real_z.signals.values(:);
fprintf('  %5s | %7s %7s | %6s\n', 't', 'roll', 'pitch', 'z');
for ct = 0:0.5:T
    [~, idx] = min(abs(t - ct));
    fprintf('  %5.1f | %7.2f %7.2f | %6.3f\n', t(idx), r(idx), pch(idx), zv(idx));
end
mask = t > 1;
fprintf('\n>> 자세 RMS %.2f도 / 최대|R| %.1f / 최대|P| %.1f / z [%.2f %.2f]\n', ...
    sqrt(mean(r(mask).^2 + pch(mask).^2)), max(abs(r)), max(abs(pch)), min(zv), max(zv));
fprintf('>> 위 결과가 호버 검증(RMS 0.56도급)과 같으면 파일 굽기 성공 - 이 .slx는 자립적으로 난다\n');
