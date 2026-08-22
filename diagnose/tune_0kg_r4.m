%% 0 kg 재튜닝 4라운드: limit_att 임계 탐색 (호버·이동·외란 3종) (성능 지표 세션, 2026-08-18)
%% 1라운드(tune_0kg.m: sA x kd/kp x kp_pos)의 승자를 기준점(BASE)으로 두고, 사용자 지정 튜닝 항목
%% (자세/위치/고도/yaw 루프의 게인·필터·포화·트림 — 모터 내부 루프 제외)을 한 축씩 3점 스윕한다.
%% 각 점: 6 s 호버 → 지터 ≤ 2° 면 1 m 이동(10 s). 점수 = 호버 지터/드리프트/새그 + 이동 추종/오버/z피크/자세피크 가중합.
%% .slx 하드코딩 상수 4종은 메모리 수술로 변수화 (save 금지): Alt Cmd Sat(±30), Pitch/Roll Limit(±60°),
%%   Filter pz(0.01), pitch/roll 측정 필터(현재 altitude_filtM 참조 → filtM_att_meas 로 분리).
%% 출력: diagnose/results/tune_0kg_r2.csv (축, 값, 지표, 점수)
%% 사용: matlab -batch "cd(fullfile(pwd,'diagnose')); tune_0kg_r2" > out.txt 2>&1
%%      AXES 환경변수(쉼표 구분 축 이름)로 부분 실행 가능.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
cacheDir = fullfile(getenv('LOCALAPPDATA'), 'ugrp_drone', 'slprj_perf');
if ~exist(cacheDir, 'dir'); mkdir(cacheDir); end
Simulink.fileGenControl('set', 'CacheFolder', cacheDir, 'CodeGenFolder', cacheDir, 'createDir', true);
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);
t0all = tic;

dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
unlink_chain(dropBlocks{1}, mdl);
for i = 1:numel(dropBlocks); set_param(dropBlocks{i}, 'Value', '-1'); end

% --- 외란 토크 배선 (verify_agile_gates.m 검증본 그대로) ---
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
        'Amplitude', '0', 'Period', '100', 'PulseWidth', '0.3', 'PhaseDelay', '4');
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
fprintf('외란 배선 완료 (TorqueX, 몸체 중앙 노드, 진폭 0 = 무외란)\n');

% --- 하드코딩 상수 변수화 (메모리) ---
blkAltSat = find_one(mdl, 'Alt Cmd Sat');
unlink_chain(blkAltSat, mdl); set_param(blkAltSat, 'UpperLimit', 'altCmdSat', 'LowerLimit', '-altCmdSat');
blkPL = find_one(mdl, 'Pitch Limit'); unlink_chain(blkPL, mdl); set_param(blkPL, 'UpperLimit', 'tiltLimit', 'LowerLimit', '-tiltLimit');
blkRL = find_one(mdl, 'Roll Limit');  unlink_chain(blkRL, mdl); set_param(blkRL, 'UpperLimit', 'tiltLimit', 'LowerLimit', '-tiltLimit');
blkFz = find_one(mdl, 'Filter pz');   unlink_chain(blkFz, mdl); set_param(blkFz, 'Denominator', '[filtPz 1]');
blkFp = find_one(mdl, 'Filter Pitch'); unlink_chain(blkFp, mdl); set_param(blkFp, 'Denominator', '[filtM_att_meas 1]');
blkFr = find_one(mdl, 'Filter Roll');  unlink_chain(blkFr, mdl); set_param(blkFr, 'Denominator', '[filtM_att_meas 1]');
blkBias = find_one(mdl, 'Bias Chassis'); unlink_chain(blkBias, mdl); set_param(blkBias, 'Bias', 'biasChassis');
qc_zsplit_apply(mdl);
fprintf('하드코딩 상수 5종 변수화 완료 (altCmdSat/tiltLimit/filtPz/filtM_att_meas/biasChassis)\n');

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element1','py'; 'In Bus Element2','pz'; ...
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

% --- 0 kg 물성 ---
pkgSize = [1 1 1] * 0.14;
pkgDensity = 1e-6 / (pkgSize(1)*pkgSize(2)*pkgSize(3));
m_pkg_now = 0; wind_speed = 0;

% --- 궤적 ---
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0; dt = 0.01;
s5 = @(tau) (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
Th = 6;  tth = (0:round(Th/dt))' * dt; smH = repmat([0 0 1], numel(tth), 1);   % 한계사이클은 2 s 내 발현 (사용자 지적: 짧게)
Tm = 10; ttm = (0:round(Tm/dt))' * dt;                                            % 이동 3~5 s, 꼬리 창 6~10 s
smM = traj_smoother(ttm, [s5(min(max((ttm-3)/0.9,0),1)), zeros(numel(ttm),1), ones(numel(ttm),1)], VMAX, AMAX, JMAX);
mws = get_param(mdl, 'ModelWorkspace');
setTraj = @(tt, sm) cellfun(@(k,v) mws.assignin(k, v), ...
    {'timespot_spl','spline_data','spline_yaw','waypoints','wayp_path_vis'}, ...
    {tt, sm, zeros(numel(tt),1), [sm(1,:); sm(1,:)+[1 0 0]]', quadcopter_waypoints_to_path_vis([sm(1,:); sm(1,:)+[1 0 0]]')});

% --- 기준점 BASE (1라운드 승자 — 실행 전 갱신) ---
BASE = struct( ...
    'sA', 0.75, 'r_att', 1.5, 'ki_att', -10, 'filtD_att', 2500, 'filtM_att_meas', 0.05, 'limit_att', 800, ...
    'kp_pos', 8, 'kd_pos', 3.2, 'ki_pos', 0.04, 'pos2att', 2.4, 'posErrSat', 1.2/8, 'posErrSatZ', 1.2/8, ...
    'filtM_pos', 0.005, 'filtD_pos', 100, 'tiltLimit', pi/3, ...
    'sZ', 0.56, 'kp_alt', 0.5, 'ki_alt', 0.1, 'kd_alt', 0.15, 'filtM_alt', 0.05, 'filtD_alt', 1000, 'filtPz', 0.01, ...
    'limit_alt', 10, 'altCmdSat', 30, 'biasChassis', 56.5, ...
    'kp_yaw', 15, 'ki_yaw', 1.5, 'kd_yaw', 4, 'filtM_yaw', 0.01, 'filtD_yaw', 100, 'limit_yaw', 20);
baseFile = getenv('BASE_JSON');
if ~isempty(baseFile) && exist(baseFile, 'file')
    ov = jsondecode(fileread(baseFile)); fn = fieldnames(ov);
    for i = 1:numel(fn); BASE.(fn{i}) = ov.(fn{i}); end
    fprintf('BASE 갱신: %s\n', baseFile);
end

% --- limit_att 임계 탐색 (사용자: 포화는 작을수록 호버는 좋아지지만 권한 임계가 있다) ---
% 승자 구성(BASE_JSON 로 주입) 위에서 limit_att 만 스윕, 각 점: 호버(6 s) + 1 m 이동(10 s) + 외란 펄스 0.3 N·m(10 s).
LA = [20 30 40 50 75 100 150 300];
csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
rows = {};
fprintf('===== 0 kg r4 limit_att 임계: %d 점 =====\n', numel(LA));
fprintf('%6s | %8s %8s %8s | %8s %8s %8s %8s | %8s %8s %8s\n', 'limAtt', '호버RMS','새그','드리프트','추종','오버','z피크','자세피크','외란피크','회복s','외란z');
for k = 1:numel(LA)
    cfg = BASE; cfg.limit_att = LA(k);
    set_param(plsB, 'Amplitude', '0');
    [res, ~] = eval_cfg(cfg, mdl, mws, setTraj, tth, smH, ttm, smM, Th, Tm, sT, sQ);
    % 외란: 호버 궤적 + 펄스 @4 s, 10 s
    set_param(plsB, 'Amplitude', '0.3');
    Td = 10; ttd = (0:round(Td/dt))' * dt; smD = repmat([0 0 1], numel(ttd), 1);
    setTraj(ttd, smD); set_param(mdl, 'StopTime', num2str(Td));
    dpk = NaN; trec = NaN; dz = NaN;
    try
        out = sim(mdl, 'SrcWorkspace', 'base', 'ReturnWorkspaceOutputs', 'on');
        tu = (0:0.005:Td)'; gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
        pg = rad2deg(gi(out.get('real_pitch'))); rg = rad2deg(gi(out.get('real_roll'))); zg = gi(out.get('pz'));
        pre = tu>=2 & tu<4; post = tu>=4;
        dev = hypot(rg - mean(rg(pre)), pg - mean(pg(pre)));
        dpk = max(dev(post));
        ok = dev < 1.0; i43 = find(tu >= 4.3, 1);
        for ii = i43:numel(tu); if all(ok(ii:end)); trec = tu(ii) - 4; break; end; end
        dz = max(abs(zg(post) - 1)) * 100;
    catch e
        fprintf('  (외란 실패: %s)\n', e.message);
    end
    set_param(plsB, 'Amplitude', '0');
    rows(end+1,:) = [{LA(k)}, num2cell(res), {dpk, trec, dz}]; %#ok<AGROW>
    fprintf('%6d | %8.3f %8.1f %8.2f | %8.2f %8.1f %8.1f %8.1f | %8.2f %8.2f %8.1f\n', LA(k), res, dpk, trec, dz);
    Tb = cell2table(rows, 'VariableNames', {'limit_att','hover_att_rms_deg','sag_cm','drift_cm','track_rms_cm','overshoot_cm','z_peak_cm','att_peak_deg','dist_peak_deg','dist_recover_s','dist_z_cm'});
    writetable(Tb, fullfile(csvDir, 'tune_0kg_r4_limit.csv'));
end
fprintf('완료: %.0fs\n', toc(t0all));
close_system(mdl, 0);

% ================================================================ 함수
function [res, score] = eval_cfg(c, mdl, mws, setTraj, tth, smH, ttm, smM, Th, Tm, sT, sQ)
    % 게인 배선 (parameters.m 규약)
    assignin('base', 'sA_mass', c.sA);
    assignin('base', 'kp_attitude', -85 * sT * c.sA);
    assignin('base', 'ki_attitude', c.ki_att * sT * c.sA);
    assignin('base', 'kd_attitude', -85 * c.r_att * sT * c.sA);
    assignin('base', 'filtD_attitude', c.filtD_att);
    assignin('base', 'filtM_att_meas', c.filtM_att_meas);
    assignin('base', 'limit_attitude', c.limit_att);
    assignin('base', 'kp_position', c.kp_pos); assignin('base', 'kd_position', c.kd_pos); assignin('base', 'ki_position', c.ki_pos);
    assignin('base', 'pos2attitude', c.pos2att);
    assignin('base', 'posErrSat', c.posErrSat); assignin('base', 'posErrSatZ', c.posErrSatZ);
    assignin('base', 'filtM_position', c.filtM_pos); assignin('base', 'filtD_position', c.filtD_pos);
    assignin('base', 'tiltLimit', c.tiltLimit);
    assignin('base', 'sZ_mass', c.sZ);
    assignin('base', 'kp_altitude', c.kp_alt * sT * c.sZ); assignin('base', 'ki_altitude', c.ki_alt * sT * c.sZ); assignin('base', 'kd_altitude', c.kd_alt * sT * c.sZ);
    assignin('base', 'filtM_altitude', c.filtM_alt); assignin('base', 'filtD_altitude', c.filtD_alt); assignin('base', 'filtPz', c.filtPz);
    assignin('base', 'limit_altitude', c.limit_alt); assignin('base', 'altCmdSat', c.altCmdSat); assignin('base', 'biasChassis', c.biasChassis);
    assignin('base', 'kp_yaw', c.kp_yaw * sQ); assignin('base', 'ki_yaw', c.ki_yaw * sQ); assignin('base', 'kd_yaw', c.kd_yaw * sQ);
    assignin('base', 'filtM_yaw', c.filtM_yaw); assignin('base', 'filtD_yaw', c.filtD_yaw); assignin('base', 'limit_yaw', c.limit_yaw);
    res = nan(1,7); score = NaN;
    % ① 호버
    setTraj(tth, smH); set_param(mdl, 'StopTime', num2str(Th));
    try
        out = sim(mdl, 'SrcWorkspace', 'base', 'ReturnWorkspaceOutputs', 'on');
    catch e
        fprintf('  (호버 실패: %s)\n', e.message); return;
    end
    px = out.get('px'); py = out.get('py'); pz = out.get('pz');
    rp = out.get('real_pitch'); rr = out.get('real_roll');
    tu = (0:0.005:Th)';
    gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi(px); yg = gi(py); zg = gi(pz); pg = rad2deg(gi(rp)); rg = rad2deg(gi(rr));
    w = tu >= 2;
    hov = sqrt(mean([pg(w)-mean(pg(w)); rg(w)-mean(rg(w))].^2));
    sag = (1 - min(zg(tu < 2))) * 100;
    drift = max(hypot(xg(w)-mean(xg(w)), yg(w)-mean(yg(w)))) * 100;
    if ~isfinite(hov) || max(abs([xg; yg])) > 5 || min(zg) < 0.3; hov = 99; end   % 발산 표식
    res(1:3) = [hov sag drift];
    if hov > 2.0
        score = 100 + hov + drift;    % 호버 탈락 — 큰 점수 (작을수록 좋음)
        return;
    end
    % ② 1 m 이동
    setTraj(ttm, smM); set_param(mdl, 'StopTime', num2str(Tm));
    try
        out = sim(mdl, 'SrcWorkspace', 'base', 'ReturnWorkspaceOutputs', 'on');
    catch e
        fprintf('  (이동 실패: %s)\n', e.message); score = 90; return;
    end
    px = out.get('px'); pz = out.get('pz'); rp = out.get('real_pitch'); rr = out.get('real_roll');
    tu2 = (0:0.005:Tm)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu2, 'linear', 'extrap');
    xg2 = gi2(px); zg2 = gi2(pz); pg2 = rad2deg(gi2(rp)); rg2 = rad2deg(gi2(rr));
    xr = interp1(ttm, smM(:,1), tu2);
    seg = @(a,b) (tu2>=a & tu2<b);
    mv = sqrt(mean((xg2(seg(3,7)) - xr(seg(3,7))).^2)) * 100;
    ov = max(0, max(xg2) - 1) * 100;
    zpk = max(abs(zg2(tu2 >= 2) - 1)) * 100;
    apk = max(max(abs(pg2)), max(abs(rg2)));
    if max(abs(xg2)) > 5; mv = 999; ov = 999; end
    res(4:7) = [mv ov zpk apk];
    % 점수: 스펙 정규화 가중합 (호버 지터/0.25, 드리프트/5, 새그/5, 추종/10, 오버/10, z피크/10, 자세피크/20)
    score = hov/0.25 + drift/5 + sag/5 + mv/10 + ov/10 + zpk/10 + apk/20;
end

function b = find_one(mdl, name)
    hits = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', name);
    if isempty(hits); error('블록 못 찾음: %s (실행 무효)', name); end
    if numel(hits) > 1
        fprintf('경고: "%s" %d개 — 첫 번째 사용: %s\n', name, numel(hits), regexprep(hits{1}, '\s+', ' '));
    end
    b = hits{1};
end

function unlink_chain(blk, mdl)
    p = get_param(blk, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
                set_param(p, 'LinkStatus', 'none');
            end
        catch
        end
        p = get_param(p, 'Parent');
    end
end

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
