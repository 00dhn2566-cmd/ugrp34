%% 자세 플랜트 식별 + pidtune 게인 산출 + 검증 (MATLAB 도구 기반 튜닝)
%% 1) B'구성 + 위치루프 절단, pitch 채널에 사각파(±5도) 가진
%%    u = Control Pitch 출력(모멘트 명령), y = 실제 pitch 를 로깅
%% 2) 회귀로 플랜트 추정: y'' = b*u + c*y'  ->  P(s) = b/(s^2 - c*s)  (b의 부호가 플랜트 부호)
%% 3) C = pidtune(P, 'PDF', wc) -> kp/ki/kd 산출 (부호 자동 포함)
%% 4) 산출 게인으로 스텝 검증 시뮬
%% 규칙: 대상 미발견 시 error() 즉사.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% 식별 런 게인: 유계 응답이 확인된 세트 (스윕에서 -20/-12가 가장 억제력 좋았음)
kp_attitude = -20;  ki_attitude = 0;    kd_attitude = -12;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.5;  ki_altitude = 0.1;  kd_altitude = 0.3;
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- B'보정 ---
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
    error('plate_bottom <-> B 라인을 못 찾음 - 실행 무효');
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

% --- 물리 보정 + 추력 재스케일 + 클램프 ---
propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);
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

% --- 위치루프 절단 + 가진 주입 (Out1: 사각파 ±5도, Out2: 0) ---
mc = [mdl '/Maneuver Controller'];
pcB = [mc '/Position Control'];
pph = get_param(pcB, 'PortHandles');
for oi = 1:2
    l = get_param(pph.Outport(oi), 'Line');
    if l == -1
        error('PC Out%d 라인 없음 - 실행 무효', oi);
    end
    dsts = get_param(l, 'DstPortHandle');
    delete_line(l);
    if oi == 1
        srcName = 'Excite Pulse';
        srcB = [mc '/' srcName];
        if isempty(find_system(mc, 'SearchDepth', 1, 'Name', srcName))
            add_block('simulink/Sources/Pulse Generator', srcB, ...
                'Amplitude', '10*pi/180', 'Period', '1.5', 'PulseWidth', '50', 'PhaseDelay', '0.3');
        end
        % 사각파 0..10도 -> -5..+5도로 시프트
        biasB = [mc '/Excite Bias'];
        if isempty(find_system(mc, 'SearchDepth', 1, 'Name', 'Excite Bias'))
            add_block('simulink/Math Operations/Bias', biasB, 'Bias', '-5*pi/180');
        end
        sph2 = get_param(srcB, 'PortHandles');
        bph2 = get_param(biasB, 'PortHandles');
        if get_param(bph2.Inport(1), 'Line') == -1
            add_line(mc, sph2.Outport(1), bph2.Inport(1), 'autorouting', 'on');
        end
        outPh = bph2.Outport(1);
    else
        srcName = 'Zero Cmd';
        srcB = [mc '/' srcName];
        if isempty(find_system(mc, 'SearchDepth', 1, 'Name', srcName))
            add_block('simulink/Sources/Constant', srcB, 'Value', '0');
        end
        z2 = get_param(srcB, 'PortHandles');
        outPh = z2.Outport(1);
    end
    for dd = dsts(:)'
        add_line(mc, outPh, dd, 'autorouting', 'on');
    end
end
fprintf('가진 주입 완료 (Out1: ±5도 사각파 1.5s 주기, Out2: 0)\n');

% --- Control Pitch 출력(u) 태핑 ---
yprB = [mc '/Altitude and  YPR Control'];
allY = find_system(mc, 'LookUnderMasks','all', 'FollowLinks','on');
cpB = '';
yprNorm = regexprep(yprB, '\s+', ' ');
for i = 1:numel(allY)
    try
        nm1 = strtrim(regexprep(get_param(allY{i}, 'Name'), '\s+', ' '));
    catch
        continue;
    end
    if strcmp(nm1, 'Control Pitch')
        parNorm = regexprep(get_param(allY{i}, 'Parent'), '\s+', ' ');
        if strcmp(parNorm, yprNorm)
            cpB = allY{i};
        end
    end
end
if isempty(cpB)
    fprintf('YPR 직계에서 Control Pitch 못 찾음 - 전체 후보:\n');
    for i = 1:numel(allY)
        try
            nm1 = strtrim(regexprep(get_param(allY{i}, 'Name'), '\s+', ' '));
            if ~isempty(regexp(nm1, '^Control', 'once'))
                fprintf('  %s\n', strrep(allY{i}, newline, '|'));
            end
        catch
        end
    end
    error('Control Pitch 블록 못 찾음 - 실행 무효');
end
fprintf('Control Pitch: %s\n', strrep(cpB, newline, '|'));
yprReal = get_param(cpB, 'Parent');   % 개행 포함 실제 경로 사용
cpPh = get_param(cpB, 'PortHandles');
twU = [yprReal '/To Workspace u_pitch'];
if isempty(find_system(yprReal, 'SearchDepth', 1, 'Name', 'To Workspace u_pitch'))
    add_block('simulink/Sinks/To Workspace', twU, 'VariableName', 'u_pitch', 'SaveFormat', 'StructureWithTime');
    twUph = get_param(twU, 'PortHandles');
    add_line(yprReal, cpPh.Outport(1), twUph.Inport(1), 'autorouting', 'on');
end

% --- 궤적/워크스페이스 ---
dt = 0.01;
T = 6;
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

%% ================= 식별 런 =================
fprintf('\n=== 식별 런 (T=%gs, 게인 kp=%g kd=%g) ===\n', T, kp_attitude, kd_attitude);
sim(mdl);
tu = u_pitch.time(:);   uu = u_pitch.signals.values(:);
ty = real_pitch.time(:); yy = real_pitch.signals.values(:);   % rad

% 균일 리샘플
dts = 0.005;
tg = (0.5:dts:T-0.1)';   % 초기 과도 제외
u_rs = interp1(tu, uu, tg, 'linear');
y_rs = interp1(ty, yy, tg, 'linear');
% 부드러운 미분
yd  = gradient(y_rs, dts);
ydd = gradient(yd, dts);
% 저역 필터(간단 이동평균)로 잡음 억제
w = 9;
k1 = ones(w,1)/w;
ydd_f = conv(ydd, k1, 'same');
u_f   = conv(u_rs, k1, 'same');
yd_f  = conv(yd,  k1, 'same');
% 회귀: ydd = b*u + c*yd
A = [u_f, yd_f];
theta = A \ ydd_f;
b_hat = theta(1); c_hat = theta(2);
fitres = 1 - norm(A*theta - ydd_f)/norm(ydd_f - mean(ydd_f));
fprintf('플랜트 회귀: ydd = %+.4g * u %+.4g * yd   (적합도 %.2f)\n', b_hat, c_hat, fitres);
fprintf('  -> 플랜트 부호 b = %+.4g (음수면 자세 PID 게인도 음수가 정답)\n', b_hat);

% 플랜트 & pidtune
s = tf('s');
P = b_hat / (s^2 - c_hat*s);
wc_list = [4 8 12];
fprintf('\n=== pidtune 결과 ===\n');
best = [];
for wc = wc_list
    try
        C = pidtune(P, 'PDF', wc);
        [gm, pm] = margin(C*P);
        fprintf('  wc=%4.1f rad/s: Kp=%+.3f Ki=%+.3f Kd=%+.3f (Tf=%.4f) | PM=%.0f도\n', ...
            wc, C.Kp, C.Ki, C.Kd, C.Tf, pm);
        if wc == 8; best = C; end
    catch e
        fprintf('  wc=%4.1f 실패: %s\n', wc, e.message(1:min(100,end)));
    end
end
if isempty(best)
    error('pidtune 실패 - 위 메시지 확인');
end

%% ================= 검증 런 (pidtune 게인) =================
kp_attitude = best.Kp;
ki_attitude = best.Ki;
kd_attitude = best.Kd;
fprintf('\n=== 검증 런: pidtune 게인 적용 kp=%.3f ki=%.3f kd=%.3f ===\n', kp_attitude, ki_attitude, kd_attitude);
sim(mdl);
t = real_pitch.time(:);
pch = rad2deg(real_pitch.signals.values(:));
r = rad2deg(real_roll.signals.values(:));
zv = real_z.signals.values(:);
fprintf('  %5s | %8s | %8s | %6s\n', 't', 'pitch', 'roll', 'z');
for ct = 0:0.3:T
    [~, idx] = min(abs(t - ct));
    fprintf('  %5.2f | %8.2f | %8.2f | %6.3f\n', t(idx), pch(idx), r(idx), zv(idx));
end
fprintf('\n(판정) ±5도 사각파를 pitch가 추종하면 성공 - 다음: roll 검증 + 위치루프 복구 호버\n');
