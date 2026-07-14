%% 강건성: x/y 병진(횡방향) 외란 — 돌풍 펄스 1N x 1s @ t=5s, 축별 2케이스
%% 지금까지 토크(회전) 외란만 시험했으므로 병진 계열 추가 (사용자 요청).
%% 1N ~= 풍속 7m/s급 돌풍이 기체 전면적(Cd~0.35, A~0.09m2)에 주는 힘. 지속 성분은
%% CG 테스트(상수 외란 = 위치 I 소거 증명)와 다음 세션 바람 테스트가 커버 -> 오늘은 펄스만.
%% 채점: 위치 이탈/복귀 (px/py = In Bus Element/Element1 직접 로깅), 자세 과도, z 유지.
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

% --- 궤적: 15초 호버 (컴파일 검사 전 선행 주입) ---
dt = 0.01; T = 15; N = round(T/dt) + 1;
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

% --- 주입점: 몸체 프레임 (검증된 방법: robust_torque/yaw_verify와 동일) ---
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
    nOth = sum(hs ~= bconn(ci));
    if attPort == -1 && nOth >= 1; attPort = bconn(ci); end
end
if attPort == -1; error('Body 프레임 주입점 없음 - 실행 무효'); end

% --- 외란 블록 2개 (ForceX용 / ForceY용), 각각 프레임 분기 + 컴파일 검사 배선 ---
ref = 'sm_lib/Forces and Torques/External Force and Torque';
axDefs = {'X', 'EnableForceX'; 'Y', 'EnableForceY'; 'Z', 'EnableForceZ'};
plsHandles = cell(3,1);
for ai = 1:3
    axTag = axDefs{ai,1};
    extB = sprintf('%s/Disturb Force %s', qcSys2, axTag);
    if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', sprintf('Disturb Force %s', axTag)))
        add_block(ref, extB);
    end
    set_param(extB, axDefs{ai,2}, 'on');
    plsB = sprintf('%s/Gust Pulse %s', qcSys2, axTag);
    if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', sprintf('Gust Pulse %s', axTag)))
        add_block('simulink/Sources/Pulse Generator', plsB, ...
            'Amplitude', '0', 'Period', '100', 'PulseWidth', '1', 'PhaseDelay', '5');
    end
    spsB = sprintf('%s/Gust SPS %s', qcSys2, axTag);
    if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', sprintf('Gust SPS %s', axTag)))
        add_block('nesl_utility/Simulink-PS Converter', spsB);
    end
    try; set_param(spsB, 'Unit', 'N'); catch; fprintf('SPS 단위 설정 실패(%s) - 단위 주의\n', axTag); end
    pph = get_param(plsB, 'PortHandles');
    sph = get_param(spsB, 'PortHandles');
    if get_param(sph.Inport(1), 'Line') == -1
        add_line(qcSys2, pph.Outport(1), sph.Inport(1), 'autorouting', 'on');
    end
    eph = get_param(extB, 'PortHandles');
    allC = [eph.LConn eph.RConn];
    if numel(allC) ~= 2; error('%s: conserving 포트 %d개(2개 예상)', axTag, numel(allC)); end
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
            break;
        catch
            try; feval(mdl, [], [], [], 'term'); catch; end
            for l2 = added; try; delete_line(l2); catch; end; end
        end
    end
    if ~wired; error('%s축 외란 배선 실패', axTag); end
    plsHandles{ai} = plsB;
    fprintf('%s축 돌풍 배선 완료\n', axTag);
end

% --- 로깅: 위치 + 자세 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element1','py'; 'In Bus Element2','real_z'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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

% --- 6케이스: x/y 돌풍 1N, z 돌풍 ±2N(1s), z 지속바람 ±1N(t=5~끝) ---
% z ±2N = 중량(19.3N)의 ~10% 다운/업드래프트. 지속 ±1N(~5%) = 고도 I(ki=0.1)의 상수외란 소거 검증.
% 열: {태그, Fx, Fy, Fz, z펄스폭%% (1=1s 펄스, 10=t=5~15s 지속)}
cases = {'x축',            1, 0,  0,  1; ...
         'y축',            0, 1,  0,  1; ...
         'z 하강돌풍',      0, 0, -2,  1; ...
         'z 상승돌풍',      0, 0,  2,  1; ...
         'z 지속하강바람',   0, 0, -1, 10; ...
         'z 지속상승바람',   0, 0,  1, 10};
fprintf('\n===== x/y/z 병진 외란 강건성 (%gs 호버, 외란 @ t=5s) =====\n', T);
for ci = 1:size(cases,1)
    tag = cases{ci,1};
    set_param(plsHandles{1}, 'Amplitude', num2str(cases{ci,2}));
    set_param(plsHandles{2}, 'Amplitude', num2str(cases{ci,3}));
    set_param(plsHandles{3}, 'Amplitude', num2str(cases{ci,4}));
    set_param(plsHandles{3}, 'PulseWidth', num2str(cases{ci,5}));
    fprintf('\n--- 케이스 %s ---\n', tag);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    t = px.time(:);
    x = px.signals.values(:);
    y = interp1(py.time(:), py.signals.values(:), t, 'linear', 'extrap');
    zv = interp1(real_z.time(:), real_z.signals.values(:), t, 'linear', 'extrap');
    r = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), t, 'linear', 'extrap'));
    pch = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear', 'extrap'));
    for ct = [0 3 5 5.5 6 6.5 7 8 9 10 12 15]
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%5.1f | x %7.3f y %7.3f | roll %6.2f pitch %6.2f | z %6.3f\n', ...
            t(idx), x(idx), y(idx), r(idx), pch(idx), zv(idx));
    end
    % 채점: 외란 축 위치 이탈, 복귀(±5cm 재진입 유지), 자세 과도, z
    if cases{ci,2} ~= 0
        dpos = x;
    elseif cases{ci,3} ~= 0
        dpos = y;
    else
        dpos = zv;   % z 케이스: 고도 자체가 외란 축
    end
    base = mean(dpos(t > 3 & t < 5));
    mPost = t >= 5;
    devMax = max(abs(dpos(mPost) - base));
    tRec = NaN;
    okm = abs(dpos - base) < 0.05;
    i6 = find(t >= 6, 1);
    for ii = i6:numel(t)
        if all(okm(ii:end)); tRec = t(ii) - 5.0; break; end
    end
    mask2 = t > 2;
    fprintf('  >> 외란축 이탈 %.3fm / 복귀(±5cm) %.2fs / 최대|roll| %.1f |pitch| %.1f / z [%.3f %.3f]\n', ...
        devMax, tRec, max(abs(r(mask2))), max(abs(pch(mask2))), min(zv), max(zv));
    if devMax < 0.5 && ~isnan(tRec) && min(zv) > 0.7
        fprintf('  >> 합격: 외란 흡수 후 복귀\n');
    else
        fprintf('  >> 불합격/경계: 수치 검토 필요\n');
    end
end

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
