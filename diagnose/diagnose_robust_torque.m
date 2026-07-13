%% 강건성 ①: roll축 외란 토크 펄스 0.3 N·m x 0.3s @ t=4s
%% 전제: 구운 .slx (앵커보정/Bias/클램프/게인 모두 파일에 반영됨) - 모델 무수정, 외란 주입만.
%% 지난 실패 원인 추정: External Force and Torque의 프레임 포트/토크입력 포트 혼동 배선.
%% 이번 대책: (1) 기존 항력용 External F&T 배선을 먼저 조사해 출력,
%%           (2) 주입 지점은 팔(회전 프레임)이 아닌 몸체 중앙 프레임 노드,
%%           (3) 두 배선 방향을 컴파일 검사로 시도, 맞는 쪽 채택. 실패 시 전체 오류 출력 후 즉사.
%% 규칙: 대상 미발견 시 error() 즉사. save_system 금지 (구운 파일 보호).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;   % 최종 게인/Kthrust/Kdrag 반영본
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- [1] 기존 External Force and Torque 조사 (포트 방향 참고용) ---
ref = 'sm_lib/Forces and Torques/External Force and Torque';
exts = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'ReferenceBlock', ref);
fprintf('=== 기존 External Force and Torque: %d개 ===\n', numel(exts));
for k = 1:numel(exts)
    fprintf(' [%d] %s\n', k, regexprep(exts{k}, '\s+', ' '));
    phk = get_param(exts{k}, 'PortHandles');
    cps = [phk.LConn phk.RConn];
    nL = numel(phk.LConn);
    for ci = 1:numel(cps)
        if ci <= nL; tag = sprintf('LConn%d', ci); else; tag = sprintf('RConn%d', ci-nL); end
        l = get_param(cps(ci), 'Line');
        if l == -1; fprintf('    %s: (미연결)\n', tag); continue; end
        hs = collect_line_ends(l);
        nbrs = {};
        for e2 = hs(:)'
            if e2 ~= cps(ci)
                nbrs{end+1} = strtrim(regexprep(get_param(get_param(e2,'Parent'),'Name'), '\s+', ' ')); %#ok<SAGROW>
            end
        end
        fprintf('    %s -> %s\n', tag, strjoin(unique(nbrs), ', '));
    end
end

% --- [2] 주입 지점: 몸체 중앙 프레임 노드 (Transform Arm들이 모이는 쪽) ---
allBlk2 = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
armTf1 = '';
for i = 1:numel(allBlk2)
    try
        nm1 = strtrim(regexprep(get_param(allBlk2{i}, 'Name'), '\s+', ' '));
    catch
        continue;
    end
    if strcmp(nm1, 'Transform Arm1'); armTf1 = allBlk2{i}; end
end
if isempty(armTf1); error('Transform Arm1 못 찾음 - 실행 무효'); end
qcSys2 = get_param(armTf1, 'Parent');
fprintf('\n주입 서브시스템: %s\n', regexprep(qcSys2, '\s+', ' '));

% 라이브러리 링크 비활성화 (블록 추가 위해)
p = qcSys2;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
            fprintf('링크 비활성: %s\n', regexprep(p, '\s+', ' '));
        end
    catch
    end
    p = get_param(p, 'Parent');
end

% 주입 지점: 몸체 'Body' 서브시스템(섀시 기준 프레임)의 conserving 포트 라인에 분기
bodyBlk = find_system(qcSys2, 'SearchDepth', 1, 'BlockType', 'SubSystem', 'Name', 'Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, qcSys2));   % 주입 시스템 자신(같은 이름 'Body') 제외
if isempty(bodyBlk); error('%s 안에서 내부 Body 서브시스템 못 찾음 - 실행 무효', qcSys2); end
bodyBlk = bodyBlk{1};
fprintf('내부 Body 블록: %s\n', regexprep(bodyBlk, '\s+', ' '));
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
        nbrs{end+1} = strtrim(regexprep(get_param(get_param(e2,'Parent'),'Name'), '\s+', ' ')); %#ok<SAGROW>
    end
    fprintf('Body 프레임 포트 %d: 이웃 = %s\n', ci, strjoin(unique(nbrs), ', '));
    % 이웃이 실재하는(허공 라인이 아닌) 포트만 주입점 후보로 인정
    if attPort == -1 && ~isempty(nbrs); attPort = bconn(ci); end
end
if attPort == -1; error('Body 서브시스템의 연결된 conserving 포트 없음 - 실행 무효'); end
fprintf('주입 노드: Body 프레임 포트 (몸체 기준 프레임 - TorqueX = 몸체 X축)\n');

% --- [3] 외란 블록 추가 ---
extB = [qcSys2 '/Disturb Torque'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb Torque'))
    add_block(ref, extB);
end
try
    set_param(extB, 'EnableTorqueX', 'on');
catch e
    dp = get_param(extB, 'DialogParameters');
    error('EnableTorqueX 설정 실패: %s\n파라미터 목록: %s', e.message, strjoin(fieldnames(dp)', ', '));
end
plsB = [qcSys2 '/Disturb Pulse'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb Pulse'))
    add_block('simulink/Sources/Pulse Generator', plsB, ...
        'Amplitude', '0.3', 'Period', '100', 'PulseWidth', '0.3', 'PhaseDelay', '4');
end
spsB = [qcSys2 '/Disturb SPS'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb SPS'))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try
    set_param(spsB, 'Unit', 'N*m');
catch
    fprintf('SPS Unit 설정 실패 - 기본 단위 사용(값 해석 주의)\n');
end
pph2 = get_param(plsB, 'PortHandles');
sph3 = get_param(spsB, 'PortHandles');
if get_param(sph3.Inport(1), 'Line') == -1
    add_line(qcSys2, pph2.Outport(1), sph3.Inport(1), 'autorouting', 'on');
end

% --- [4] 궤적/워크스페이스 선행 설정 (컴파일 검사가 waypoints 등을 필요로 함) ---
dt = 0.01; T = 10; N = round(T/dt) + 1;
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

% --- [5] 배선: 두 방향 시도 + 컴파일 검사 ---
eph = get_param(extB, 'PortHandles');
allC = [eph.LConn eph.RConn];
fprintf('\nDisturb Torque conserving 포트: LConn %d / RConn %d\n', numel(eph.LConn), numel(eph.RConn));
if numel(allC) ~= 2
    error('conserving 포트 %d개 (2개 예상) - 포트 구성 재확인 필요', numel(allC));
end
% 기존 블록 조사 결과: RConn1=프레임, LConn=PS입력 -> RConn 프레임을 1순위로 시도
orders = [2 1; 1 2];
wired = false;
for oi = 1:2
    fPort = allC(orders(oi,1));   % 프레임 후보
    tPort = allC(orders(oi,2));   % 토크입력 후보
    added = [];
    try
        added(end+1) = add_line(qcSys2, attPort, fPort, 'autorouting', 'on'); %#ok<SAGROW>
        added(end+1) = add_line(qcSys2, sph3.RConn(1), tPort, 'autorouting', 'on'); %#ok<SAGROW>
        feval(mdl, [], [], [], 'compile');
        feval(mdl, [], [], [], 'term');
        wired = true;
        fprintf('배선 방향 %d 컴파일 통과 (프레임=%s쪽)\n', oi, ternary_side(orders(oi,1), numel(eph.LConn)));
        break;
    catch e
        fprintf('--- 배선 방향 %d 실패 ---\n%s\n', oi, getReport(e, 'extended', 'hyperlinks', 'off'));
        try; feval(mdl, [], [], [], 'term'); catch; end
        for l2 = added
            try; delete_line(l2); catch; end
        end
    end
end
if ~wired; error('양방향 배선 모두 컴파일 실패 - 위 전체 오류 참조'); end
fprintf('외란 배선 완료: TorqueX 0.3 N·m x 0.3s @ t=4s (몸체 중앙 노드)\n');

% --- [6] 로깅 ---
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

% --- [7] 실행 + 평가 ---
fprintf('\n===== 외란 토크 강건성 (0.3 N·m x 0.3s @ 4s, 최종 게인) =====\n');
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
icr = find(zv < 0.3, 1);
tcr = Inf; if ~isempty(icr); tcr = t(icr); end
maskPre = t > 2 & t < 4;
maskPost = t >= 4 & t < 8;
rollBase = mean(r(maskPre)); pchBase = mean(pch(maskPre));
devR = max(abs(r(maskPost) - rollBase));
devP = max(abs(pch(maskPost) - pchBase));
% 회복: 펄스 종료(4.3s) 이후 |자세-기준|<1도 유지 시작 시각
tRec = NaN;
idx43 = find(t >= 4.3, 1);
okm = abs(r - rollBase) < 1.0 & abs(pch - pchBase) < 1.0;
for ii = idx43:numel(t)
    if all(okm(ii:end)); tRec = t(ii) - 4.0; break; end
end
mask = t > 1 & t < min(tcr, T);
rmsA = sqrt(mean(r(mask).^2 + pch(mask).^2));
fprintf('\n>> 생존 %.1fs / 최대이탈 roll %.2f도 pitch %.2f도 / 회복 %.2fs(펄스시작 기준) / RMS %.2f / z [%.2f %.2f]\n', ...
    min(tcr, T), devR, devP, tRec, rmsA, min(zv), max(zv));
if min(tcr,T) >= T && max(devR,devP) < 15 && ~isnan(tRec) && tRec < 3
    fprintf('>> 합격: 외란 토크 펄스 흡수 후 복귀. 다음: CG 오프셋 ±5mm\n');
else
    fprintf('>> 불합격 또는 경계: 수치 검토 필요\n');
end

function hs = collect_line_ends(l0)
    % 분기 라인 전체(부모/자식 세그먼트)를 방문 목록 기반 반복 탐색 - 순환/중복 안전
    hs = [];
    stack = l0;
    seen = l0;
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

function s = ternary_side(idx1, nL)
    if idx1 <= nL; s = 'L'; else; s = 'R'; end
end
