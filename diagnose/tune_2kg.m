%% 2 kg(최대 적재) 앵커 튜닝: 좌표하강 (성능 지표 세션, 2026-08-18) — tune_0kg_r2.m 복제, 질량만 PKG(기본 2.0)
%% 기준점 BASE = parameters.m 08-18 스케줄이 1 kg 이상에서 외삽하는 값(sA 1+0.25(m-1), sZ 0.56+0.44m, kd/kp 1.5, limit 800,
%% kp_pos 8, filtPz 0.01, FF √질량 = 100.9·√((1.2726+m)/2.2726)). 2 kg 이상은 앵커 실측이 없었으므로 이 스윕이 첫 실측.
%% 관심 축: 추력 여유(2 kg 호버 ≈ 121 rev/s vs 대역 ~131 rev/s = 825 rad/s → 여유 8 %) 하의 고도/자세 권한:
%%   altCmdSat, limit_att, sA, sZ, kp/kd_alt, biasChassis(FF), kp/kd_pos, posErrSat, filtPz, tiltLimit.
%% 출력: diagnose/results/tune_2kg_r1.csv   사용: PKG=2 matlab -batch "cd(fullfile(pwd,'diagnose')); tune_2kg" > out.txt 2>&1
%%      AXES 환경변수(쉼표 구분)로 부분 실행, BASE_JSON 으로 기준점 덮어쓰기.

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

% --- 적재 질량 (PKG env, 기본 2.0 kg) ---
m_pkg_now = str2double(getenv('PKG')); if isnan(m_pkg_now); m_pkg_now = 2.0; end
pkgSize = [1 1 1] * 0.14;
pkgDensity = max(m_pkg_now, 1e-6) / (pkgSize(1)*pkgSize(2)*pkgSize(3));
wind_speed = 0;
qc_ff_trim_apply(mdl);   % Bias Load 게인 0, Bias Chassis -> bias_hover_rps (eval_cfg 가 biasChassis 로 덮어씀)
set_param(find_one(mdl, 'Bias Chassis'), 'Bias', 'biasChassis');
ff_sqrt = 100.9 * sqrt((1.2726 + m_pkg_now) / 2.2726);
fprintf('PKG=%.2f kg, FF √질량 = %.1f rev/s (선형 = %.1f)\n', m_pkg_now, ff_sqrt, 56.5 + 44.4*m_pkg_now);

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

% --- 기준점 BASE = parameters.m 스케줄 외삽값 (m>=1: sA 1+0.25(m-1), sZ 0.56+0.44m, 1 kg 앵커 나머지) ---
mc = min(m_pkg_now, 2);
BASE = struct( ...
    'sA', 1 + 0.25*(mc-1), 'r_att', 1.5, 'ki_att', -10, 'filtD_att', 2500, 'filtM_att_meas', 0.05, 'limit_att', 800, ...
    'kp_pos', 8, 'kd_pos', 3.2, 'ki_pos', 0.04, 'pos2att', 2.4, 'posErrSat', 1.2/8, 'posErrSatZ', 1.2/8, ...
    'filtM_pos', 0.005, 'filtD_pos', 100, 'tiltLimit', pi/3, ...
    'sZ', 0.56 + 0.44*mc, 'kp_alt', 0.5, 'ki_alt', 0.1, 'kd_alt', 0.15, 'filtM_alt', 0.05, 'filtD_alt', 1000, 'filtPz', 0.01, ...
    'limit_alt', 10, 'altCmdSat', 30, 'biasChassis', ff_sqrt, ...
    'kp_yaw', 15, 'ki_yaw', 1.5, 'kd_yaw', 4, 'filtM_yaw', 0.01, 'filtD_yaw', 100, 'limit_yaw', 20);
baseFile = getenv('BASE_JSON');
ternary_tag = 'r1'; if ~isempty(baseFile); ternary_tag = 'r2'; end   % BASE_JSON 주어지면 r2 결과 파일
if ~isempty(baseFile) && exist(baseFile, 'file')
    ov = jsondecode(fileread(baseFile)); fn = fieldnames(ov);
    for i = 1:numel(fn); BASE.(fn{i}) = ov.(fn{i}); end
    fprintf('BASE 갱신: %s\n', baseFile);
end

% --- 스윕 축 (사용자 지정 튜닝 항목 1~21, 모터 루프 제외) ---
AX = { ...
 'sA',            [1.0 1.5 1.75]; ...             % 스케줄 외삽 1.25 (2 kg) 의 양옆 — 관성 1.7배(짐 진자)에서 권한 vs 지터
 'r_att',         [1.2 1.8 2.2]; ...
 'limit_att',     [400 1200 2000]; ...
 'sZ',            [0.8 1.2 1.5]; ...              % 고도 루프: 2 kg 추력 여유 8 % 에서 새그/오버 맞교환
 'altCmdSat',     [20 45 60]; ...                 % 고도 명령 클램프 ±30 rev/s: 2 kg 호버 121 + 30 = 151 > 대역 131 → 상단 포화 여부
 'biasChassis',   [56.5+44.4*m_pkg_now, ff_sqrt*0.97, ff_sqrt*1.03]; ...   % 선형 FF(145 @2kg) vs √ 앵커 ±3 %
 'kp_alt',        [0.35 0.7 1.0]; ...
 'kd_alt',        [0.08 0.3 0.5]; ...
 'ki_alt',        [0.05 0.2]; ...
 'kp_pos',        [6 10 12]; ...
 'kd_pos',        [2.4 4.5 6]; ...
 'posErrSat',     [0.08 0.25 0.4]; ...
 'tiltLimit',     [pi/9 pi/6 pi/4]; ...          % 2 kg 에서 큰 기울기 = 수직 추력 손실 → 새그; 작게 하면 권한 손실
 'filtPz',        [0.005 0.02]; ...
 'filtM_pos',     [0.02 0.05]; ...
 'ki_att',        [0 -20]; ...
 'filtM_att_meas',[0.03 0.08]; ...
 'pos2att',       [1.6 3.2]; ...
};
only = getenv('AXES');
if ~isempty(only)
    keep = strtrim(strsplit(only, ','));
    AX = AX(ismember(AX(:,1), keep), :);
end

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
rows = {};
% 기준점 먼저 1회
fprintf('===== 0 kg r2 좌표하강: %d 축 =====\n', size(AX,1));
fprintf('%-15s %8s | %8s %8s %8s | %8s %8s %8s %8s | %7s\n', '축','값','호버RMS','새그','드리프트','추종','오버','z피크','자세피크','점수');
[res, sc] = eval_cfg(BASE, mdl, mws, setTraj, tth, smH, ttm, smM, Th, Tm, sT, sQ);
rows(end+1,:) = [{'BASE', NaN}, num2cell(res), {sc}]; %#ok<AGROW>
fprintf('%-15s %8s | %8.3f %8.1f %8.2f | %8.2f %8.1f %8.1f %8.1f | %7.2f  <기준\n', 'BASE', '-', res, sc);
for a = 1:size(AX,1)
    ax = AX{a,1}; vals = AX{a,2};
    for v = vals
        cfg = BASE; cfg.(ax) = v;
        [res, sc] = eval_cfg(cfg, mdl, mws, setTraj, tth, smH, ttm, smM, Th, Tm, sT, sQ);
        rows(end+1,:) = [{ax, v}, num2cell(res), {sc}]; %#ok<AGROW>
        fprintf('%-15s %8.4g | %8.3f %8.1f %8.2f | %8.2f %8.1f %8.1f %8.1f | %7.2f\n', ax, v, res, sc);
        Tb = cell2table(rows, 'VariableNames', {'axis','value','hover_att_rms_deg','sag_cm','drift_cm','track_rms_cm','overshoot_cm','z_peak_cm','att_peak_deg','score'});
        writetable(Tb, fullfile(csvDir, sprintf('tune_%gkg_%s.csv', m_pkg_now, ternary_tag)));
    end
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
