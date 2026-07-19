%% agile 프로파일 관문 (18차): 외란 토크 + 질량 스윕 — 채택 보류 해제 판정
%% agile(24/10.8)은 이동 성능(1.3cm)만 검증된 상태(r6). 프로파일 결정 D의 조건부:
%% "외란/질량 관문 대기". 이 스크립트가 그 관문 — 합격 시 agile 보류 딱지 제거.
%% 구성: ① 1kg 호버 + 토크 펄스 0.3N·m x 0.3s @4s (diagnose_robust_torque와 동일 규격)
%%       ② 1kg 이동 (r6 기준선 재현 확인) ③ 2kg 이동 ④ 0.5kg 이동
%% 질량별 자세/고도 게인은 채택된 1차식(sA=0.75+0.25m, sZ=0.56+0.44m) 연동.
%% 합격: ① 생존+최대이탈<15도+회복<3s ②~④ 무발산 + 추종이 precision 대비 우세 유지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
ctrl_profile = 'agile';            % 위치 24/10.8 + posErrSat=0.05 자동
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 투하 비활성 ---
dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(dropBlocks{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(dropBlocks)
    set_param(dropBlocks{i}, 'Value', '-1');
end

% --- 외란 배선 (diagnose_robust_torque.m 검증본 축약) ---
ref = 'sm_lib/Forces and Torques/External Force and Torque';
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
if isempty(armTf1); error('Transform Arm1 못 찾음'); end
qcSys2 = get_param(armTf1, 'Parent');
p = qcSys2;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
bodyBlk = find_system(qcSys2, 'SearchDepth', 1, 'BlockType', 'SubSystem', 'Name', 'Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, qcSys2));
if isempty(bodyBlk); error('내부 Body 서브시스템 못 찾음'); end
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
if attPort == -1; error('Body conserving 포트 없음'); end

extB = [qcSys2 '/Disturb Torque'];
if isempty(find_system(qcSys2, 'SearchDepth', 1, 'Name', 'Disturb Torque'))
    add_block(ref, extB);
end
set_param(extB, 'EnableTorqueX', 'on');
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
end
pph2 = get_param(plsB, 'PortHandles');
sph3 = get_param(spsB, 'PortHandles');
if get_param(sph3.Inport(1), 'Line') == -1
    add_line(qcSys2, pph2.Outport(1), sph3.Inport(1), 'autorouting', 'on');
end

% 궤적 선행 주입 (컴파일 검사용 임시 호버)
dt = 0.01; T = 14; N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', [hoverPoint; hoverPoint + [0 0 2]]');
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis([hoverPoint; hoverPoint + [0 0 2]]'));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', repmat(hoverPoint, N, 1));
mws.assignin('spline_yaw', zeros(N, 1));
set_param(mdl, 'StopTime', num2str(T));

eph = get_param(extB, 'PortHandles');
allC = [eph.LConn eph.RConn];
if numel(allC) ~= 2; error('conserving 포트 %d개 (2개 예상)', numel(allC)); end
orders = [2 1; 1 2];
wired = false;
for oi = 1:2
    fPort = allC(orders(oi,1));
    tPort = allC(orders(oi,2));
    added = [];
    try
        added(end+1) = add_line(qcSys2, attPort, fPort, 'autorouting', 'on'); %#ok<SAGROW>
        added(end+1) = add_line(qcSys2, sph3.RConn(1), tPort, 'autorouting', 'on'); %#ok<SAGROW>
        feval(mdl, [], [], [], 'compile');
        feval(mdl, [], [], [], 'term');
        wired = true;
        break;
    catch
        try; feval(mdl, [], [], [], 'term'); catch; end
        for l2 = added
            try; delete_line(l2); catch; end
        end
    end
end
if ~wired; error('외란 배선 컴파일 실패'); end
fprintf('외란 배선 완료 (TorqueX, 몸체 중앙 노드)\n');

% --- 신호 태핑 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
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

% --- 이동 궤적 준비 (refine 정본과 동일: 1m 젠틀무브 0.9s) ---
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0; tStep = 3; A = 1.0;
tau = min(max((tt-tStep)/0.9,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smMove = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
smHover = repmat(hoverPoint, N, 1);
wpMove = [0 0 1; A 0 1]';

% 구성: {라벨, 질량kg, 이동?, 펄스암} — 게인은 1차식 연동
cfgs = { ...
    '외란 1kg 호버', 1.0, false, '0.3'; ...
    '이동 1kg     ', 1.0, true,  '0';   ...
    '이동 2kg     ', 2.0, true,  '0';   ...
    '이동 0.5kg   ', 0.5, true,  '0';   ...
};

nC = size(cfgs,1);
rows = nan(nC, 10);
fprintf('===== agile 관문: 외란 + 질량 (위치 24/10.8, 1차식 게인 연동) =====\n');
for c = 1:nC
    m_pkg = cfgs{c,2};
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));
    sA = 0.75 + 0.25 * min(m_pkg, 2);
    sZ = 0.56 + 0.44 * min(m_pkg, 2);
    kp_attitude = -85 * sA;   ki_attitude = -10 * sA;  kd_attitude = -127.5 * sA;
    kp_altitude = 0.5 * sZ;   ki_altitude = 0.1 * sZ;  kd_altitude = 0.15 * sZ;
    set_param(plsB, 'Amplitude', cfgs{c,4});
    if cfgs{c,3}
        mws.assignin('spline_data', smMove);
        mws.assignin('waypoints', wpMove);
        mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(wpMove));
    else
        mws.assignin('spline_data', smHover);
        mws.assignin('waypoints', [hoverPoint; hoverPoint + [0 0 2]]');
        mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis([hoverPoint; hoverPoint + [0 0 2]]'));
    end

    try
        sim(mdl);
    catch e
        fprintf('%-14s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz); pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    if cfgs{c,3}
        % 이동 지표 (refine 정본)
        xr = interp1(tt, smMove(:,1), tu);
        hovp  = rmsf(xg(seg(1,3)))*100;
        mv    = sqrt(mean((xg(seg(3,7))-xr(seg(3,7))).^2))*100;
        ov    = max(0, max(xg) - A)*100;
        tailv = rmsf(pg(seg(8,14)));
        zpk   = max(abs(zg(seg(1,14)) - 1))*100;
        apk   = max(max(abs(pg)), max(abs(rg)));
        rows(c,:) = [m_pkg, sA, sZ, hovp, mv, ov, tailv, zpk, apk, 0];
        fprintf('%-14s | 호버 %5.2f 추종 %6.2f 오버 %6.1f 꼬리 %5.2f z피크 %5.1f 자세피크 %5.1f\n', ...
            cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
    else
        % 외란 지표 (robust_torque 정본: 기준 2~4s, 이탈/회복)
        maskPre  = seg(2,4);
        maskPost = seg(4,8);
        rollBase = mean(rg(maskPre)); pchBase = mean(pg(maskPre));
        devR = max(abs(rg(maskPost) - rollBase));
        devP = max(abs(pg(maskPost) - pchBase));
        tRec = NaN;
        idx43 = find(tu >= 4.3, 1);
        okm = abs(rg - rollBase) < 1.0 & abs(pg - pchBase) < 1.0;
        for ii = idx43:numel(tu)
            if all(okm(ii:end)); tRec = tu(ii) - 4.0; break; end
        end
        zpk = max(abs(zg - 1))*100;
        surv = T; icr = find(zg < 0.3, 1);
        if ~isempty(icr); surv = tu(icr); end
        rows(c,:) = [m_pkg, sA, sZ, surv, max(devR,devP), tRec, zpk, 0, 0, 1];
        fprintf('%-14s | 생존 %4.1fs 최대이탈 %5.2f도 회복 %5.2fs z피크 %5.1fcm\n', ...
            cfgs{c,1}, surv, max(devR,devP), tRec, zpk);
    end
end
fprintf(['(합격선: 외란 = 생존 14s+이탈<15도+회복<3s / 이동 = 무발산 + 1kg 추종 r6급(1.3cm) 재현 +\n' ...
         ' 2kg/0.5kg 추종이 precision 동질량(4.0/4.2cm)보다 우세 유지)\n']);

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','sA','sZ','met1','met2','met3','met4','met5','met6','is_disturb'});
writetable(Tb, fullfile(csvDir, 'verify_agile_gates.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'verify_agile_gates.csv'));

function hs = collect_line_ends(l0)
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
