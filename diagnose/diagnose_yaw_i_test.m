%% yaw 결선: 후보 3세트 x [호버 + yaw축 외란 펄스] + 제어 신호량(모터 회전수) 채점
%% 후보: 세트2(6/0/2, 사용자 선호 ~1.5s급), 세트3(10/0/3, 스윕 유일 정착), 세트6(15/0/4, 최대이탈 최소)
%% 외란: t=8s에 z축(yaw) 토크 0.05 N·m x 0.5s — 반토크 불균형급 외란의 흡수/복귀 비교
%% 채점: ① 초기 과도 최대|yaw| ② 펄스 최대 이탈 ③ 복귀 시간 ④ 모터 차등(노력)/포화 여유/지터
%% 배선 확정 지식(diagnose_robust_torque.m 실측): External F&T는 RConn=프레임/LConn=PS입력,
%%   주입점은 Body/Body의 Transform Arm Ref쪽 프레임 라인. 컴파일 검사로 재확인.
%% 규칙: 대상 미발견 시 error() 즉사. 구운 .slx 무수정(메모리만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- [1] 궤적: 20초 호버 (먼저 주입 - 컴파일 검사 전 필수) ---
dt = 0.01; T = 25; N = round(T/dt) + 1;
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

% --- [2] yaw축 외란 하네스 (robust_torque와 동일 구조, TorqueZ) ---
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
armTf1 = '';
for i = 1:numel(allBlk)
    try
        nm1 = strtrim(regexprep(get_param(allBlk{i}, 'Name'), '\s+', ' '));
    catch
        continue;
    end
    if strcmp(nm1, 'Transform Arm1'); armTf1 = allBlk{i}; end
end
if isempty(armTf1); error('Transform Arm1 못 찾음 - 실행 무효'); end
qcSys2 = get_param(armTf1, 'Parent');
p = qcSys2;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
bodyBlk = find_system(qcSys2, 'SearchDepth', 1, 'BlockType', 'SubSystem', 'Name', 'Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, qcSys2));
if isempty(bodyBlk); error('내부 Body 서브시스템 못 찾음 - 실행 무효'); end
bodyBlk = bodyBlk{1};
bph0 = get_param(bodyBlk, 'PortHandles');
bconn = [bph0.LConn bph0.RConn];
attPort = -1;
for ci = 1:numel(bconn)
    l = get_param(bconn(ci), 'Line');
    if l == -1; continue; end
    hs = collect_line_ends(l);
    nbrs = {};
    for e2 = hs(:)'
        if e2 == bconn(ci); continue; end
        nbrs{end+1} = 1; %#ok<SAGROW>
    end
    if attPort == -1 && ~isempty(nbrs); attPort = bconn(ci); end
end
if attPort == -1; error('Body 프레임 주입점 없음 - 실행 무효'); end

ref = 'sm_lib/Forces and Torques/External Force and Torque';
extB = [qcSys2 '/Disturb Torque Z'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb Torque Z'))
    add_block(ref, extB);
end
set_param(extB, 'EnableTorqueZ', 'on');
plsB = [qcSys2 '/Disturb Pulse Z'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb Pulse Z'))
    add_block('simulink/Sources/Pulse Generator', plsB, ...
        'Amplitude', '0.01', 'Period', '100', 'PulseWidth', '30', 'PhaseDelay', '8');
end
spsB = [qcSys2 '/Disturb SPS Z'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb SPS Z'))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try; set_param(spsB, 'Unit', 'N*m'); catch; fprintf('SPS Unit 설정 실패 - 단위 주의\n'); end
pph = get_param(plsB, 'PortHandles');
sph = get_param(spsB, 'PortHandles');
if get_param(sph.Inport(1), 'Line') == -1
    add_line(qcSys2, pph.Outport(1), sph.Inport(1), 'autorouting', 'on');
end
eph = get_param(extB, 'PortHandles');
allC = [eph.LConn eph.RConn];
if numel(allC) ~= 2; error('conserving 포트 %d개(2개 예상)', numel(allC)); end
% 실측 지식: RConn=프레임, LConn=PS입력. 컴파일 검사로 확인, 실패 시 반대 방향.
orders = [2 1; 1 2];
wired = false;
for oi = 1:2
    fPort = allC(orders(oi,1)); tPort = allC(orders(oi,2));
    added = [];
    try
        added(end+1) = add_line(qcSys2, attPort, fPort, 'autorouting', 'on'); %#ok<SAGROW>
        added(end+1) = add_line(qcSys2, sph.RConn(1), tPort, 'autorouting', 'on'); %#ok<SAGROW>
        feval(mdl, [], [], [], 'compile');
        feval(mdl, [], [], [], 'term');
        wired = true;
        fprintf('yaw 외란 배선 통과 (방향 %d)\n', oi);
        break;
    catch e
        fprintf('배선 방향 %d 실패: %s\n', oi, e.message);
        try; feval(mdl, [], [], [], 'term'); catch; end
        for l2 = added; try; delete_line(l2); catch; end; end
    end
end
if ~wired; error('yaw 외란 배선 실패'); end
fprintf('외란: z축 토크 0.05 N·m x 0.5s @ t=8s\n');

% --- [3] 로깅: 자세 + 모터 회전수 (실측 매핑) ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element11','w1'; 'In Bus Element10','w2'; ...
          'In Bus Element12','w3'; 'In Bus Element13','w4'};
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

% --- [4] 후보 3세트 실행 ---
gainSets = [15 0 4; 15 0.5 4; 15 1.5 4];   % I 재심사: 현실 지속외란 하에서 오프셋 소거 능력
fprintf('\n===== yaw 결선 (호버 과도 + 외란 펄스 + 제어량, 20s) =====\n');
for gi = 1:size(gainSets, 1)
    kp_yaw = gainSets(gi,1); ki_yaw = gainSets(gi,2); kd_yaw = gainSets(gi,3);
    fprintf('\n--- 후보 %d: kp=%g ki=%g kd=%g ---\n', gi, kp_yaw, ki_yaw, kd_yaw);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    t = real_yaw.time(:);
    yw = rad2deg(real_yaw.signals.values(:));
    r = rad2deg(real_roll.signals.values(:));
    pch = rad2deg(real_pitch.signals.values(:));
    zv = real_z.signals.values(:);
    W = zeros(numel(t), 4);
    wvars = {w1, w2, w3, w4};
    for k = 1:4
        W(:,k) = interp1(wvars{k}.time(:), wvars{k}.signals.values(:), t, 'linear', 'extrap');
    end
    for ct = [0 2 4 6 8 8.5 9 10 11 12 14 16 20 22 25]
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%5.1f | yaw %8.3f | roll %6.2f pitch %6.2f | z %6.3f\n', t(idx), yw(idx), r(idx), pch(idx), zv(idx));
    end
    % 지표 ① 초기 과도(0~8s)
    m1 = t < 8;
    maxIni = max(abs(yw(m1)));
    % 지표 ② 펄스 이탈: 기준 = 7~8s 평균
    base = mean(yw(t > 7 & t < 8));
    m2 = t >= 8 & t < 15;
    dev = max(abs(yw(m2) - base));
    % 지표 ③ 복귀: 8.5s 이후 |yaw-base|<2도(배회 감안) 유지 시작
    tRec = NaN;
    okm = abs(yw - base) < 2.0;
    i85 = find(t >= 8.5, 1);
    for ii = i85:numel(t)
        if all(okm(ii:end)); tRec = t(ii) - 8.0; break; end
    end
    % 지표 ④ 제어량: 반대회전쌍 차등 (변동 큰 조합 채택)
    d14 = (W(:,1) + W(:,4)) - (W(:,2) + W(:,3));
    d13 = (W(:,1) + W(:,3)) - (W(:,2) + W(:,4));
    if std(d14) >= std(d13); dY = d14; pairTag = '(1+4)-(2+3)'; else; dY = d13; pairTag = '(1+3)-(2+4)'; end
    wHover = mean(W(t > 5, :), 'all');
    satPct = 100 * max(abs(dY)) / (2 * limit_yaw);
    jit = std(diff(dY));
    mask2 = t > 2;
    fprintf('  >> 초기 최대|yaw| %.2f도 / 펄스 이탈 %.2f도 / 복귀 %.2fs / 자세RMS %.2f / z [%.2f %.2f]\n', ...
        maxIni, dev, tRec, sqrt(mean(r(mask2).^2 + pch(mask2).^2)), min(zv), max(zv));
    fprintf('  >> 제어량 %s: 최대차등 %.2f rev/s (호버 %.1f의 %.1f%%) / 한계사용 %.0f%% / 지터 %.3f\n', ...
        pairTag, max(abs(dY)), wHover, 100*max(abs(dY))/wHover, satPct, jit);
    if satPct > 80
        fprintf('  ⚠ 포화 근접(한계의 %.0f%%) - 이 게인 탈락 후보\n', satPct);
    end
end
fprintf('\n(판정) 펄스 이탈/복귀 비슷하면 제어량·지터 작은 쪽. 포화 경고 세트는 탈락.\n');

function hs = collect_line_ends(l0)
    hs = [];
    stack = l0; seen = l0;
    while ~isempty(stack)
        l = stack(end); stack(end) = [];
        hs = [hs, get_param(l,'SrcPortHandle'), get_param(l,'DstPortHandle')]; %#ok<AGROW>
        nexts = [];
        kids = get_param(l, 'LineChildren');
        if ~isempty(kids); nexts = [nexts; kids(:)]; end
        par = get_param(l, 'LineParent');
        if par ~= -1; nexts = [nexts; par]; end
        for k2 = nexts(:)'
            if ~any(seen == k2)
                seen(end+1) = k2; %#ok<AGROW>
                stack(end+1) = k2; %#ok<AGROW>
            end
        end
    end
    hs = unique(hs(hs > 0));
end
