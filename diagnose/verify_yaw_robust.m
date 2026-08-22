%% yaw 강건화 검증 — ±pi 랩 + 이탈량 비례 감속 (사용자 지시 2026-08-22)
%% 대상 조건: 3 m / 8 s 이동 중 t=8~11s 에 z축 0.10 N·m 돌풍
%%            (verify_yawdist_move m010_fast_base 에서 28 m 발산한 바로 그 조건)
%% 케이스:
%%   base     : 수술 없음                                   <- 발산 기준선
%%   wrap     : yaw 오차 ±pi 랩만 (qc_yawwrap_apply)          <- 랩만으로 얼마나 사는가
%%   wrap_gov : 랩 + 조속기 (이탈량 선형 비례, s_min=0)        <- 최종안
%%   nd_wrap_gov : 외란 0 + 랩 + 조속기                        <- 평시 손해 측정 (감속 대가)
%% 판정: 3D 최대이탈 / 종점오차 / yaw 최대이탈 / 평시 s 손실
%% 규칙: save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
% 짐 질량 선택 (기본 1 kg). -batch 에서 `VERIFY_PKG_KG=0; verify_yaw_robust` 로 0 kg 실행.
if ~exist('VERIFY_PKG_KG','var'); VERIFY_PKG_KG = 1.0; end
qc_pkg_kg = VERIFY_PKG_KG;
if qc_pkg_kg > 0
    qc_pkg_mass_set;                 % 1 kg 이상: parameters.m 질량 1차식 그대로
else
    % 0 kg 은 08-18 채택 구성을 쓴다 (parameters.m 미동기 → tune_0kg_r5.m 에서 복원).
    % 게인 배선은 load_system 후에 해야 하므로 여기서는 표시만, 루프 안에서 적용.
    fprintf('[0kg] 08-18 채택 구성 사용 (qc_0kg_tuned_apply)\n');
end
if qc_pkg_kg <= 0; PKGTAG = '0kg'; else; PKGTAG = sprintf('%gkg', qc_pkg_kg); end
qc_clock_gov_defaults;
mdl = 'quadcopter_package_delivery';

DX = 3.0; Z0 = 1.0; T0 = 5; TMOVE = 8; T_END = 30;
TG0 = 8; TGD = 3;

%          tag                외란   랩?    AW?    조속기?
cases = { 'base',            0.10,  false, false, false; ...
          'wrap',            0.10,  true,  false, false; ...
          'aw',              0.10,  false, true,  false; ...
          'wrap_aw',         0.10,  true,  true,  false; ...
          'wrap_aw_gov',     0.10,  true,  true,  true;  ...
          'nd_wrap_aw_gov',  0.00,  true,  true,  true;  ...
          'p30_base',        0.30,  false, false, false; ...
          'p30_wrap_aw',     0.30,  true,  true,  false };
R = struct();

for ci = 1:size(cases,1)
    tag = cases{ci,1}; TAU = cases{ci,2};
    doWrap = cases{ci,3}; doAW = cases{ci,4}; doGov = cases{ci,5};
    fprintf('\n########## %s (외란 %g, 랩 %d, AW %d, 조속기 %d) ##########\n', ...
            tag, TAU, doWrap, doAW, doGov);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);

    disable_drop(mdl);          % ★ 투하 로직 무력화 — 안 하면 마지막 웨이포인트 도달 후
                                %   짐을 떨구며 고도가 0.14 m 까지 주저앉는다. 이걸 빼먹어서
                                %   앞선 이동 시험들의 '고도이탈 85.7 cm' 와 이동 후 구간이
                                %   전부 오염됐다 (diagnose_yaw_final 등은 원래 이걸 한다).
    move_setup(mdl, T_END, DX, Z0, T0, TMOVE);
    if TAU > 0
        yaw_pulse_wire(mdl, TAU, TG0, TGD);
    else
        fprintf('외란 없음\n');
    end
    log_signals(mdl);

    if qc_pkg_kg <= 0; qc_0kg_tuned_apply(mdl); end   % 0 kg 채택 구성 (게인 + 모델 상수 + 비선형 자세게인)

    if doWrap; qc_yawwrap_apply(mdl); end
    if doAW;   qc_antiwindup_apply(mdl, 'clamping', {'Control Yaw'}); end
    if doGov
        assignin('base', 'gov_on', 1);
        qc_clock_gov_apply(mdl);
        log_gov(mdl);
    else
        assignin('base', 'gov_on', 0);
    end

    tic; sim(mdl); el = toc;
    t  = real_x.time(:);
    x  = real_x.signals.values(:);
    y  = interp1(real_y.time(:), real_y.signals.values(:), t, 'linear','extrap');
    z  = interp1(real_z.time(:), real_z.signals.values(:), t, 'linear','extrap');
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), t, 'linear','extrap'));
    ywr = mod(yw + 180, 360) - 180;                       % ±180 랩

    % 경로 이탈 = 직선 경로(y=0, z=Z0)로부터의 수직 거리. 진행축(x) 지연과 분리해서 본다.
    % (초판은 '종점까지의 거리'를 재서 출발 시점의 3 m 가 항상 최대로 잡히는 무의미한 지표였다)
    devAll = max(sqrt(y.^2 + (z - Z0).^2));
    mE = t > T_END - 3;
    endErr = norm([mean(x(mE))-DX, mean(y(mE)), mean(z(mE))-Z0]);

    rec = struct('t',t,'x',x,'y',y,'z',z,'yw',yw,'ywr',ywr,'sec',el, ...
                 'devAll',devAll, 'endErr',endErr, 'yawPk',max(abs(yw - yw(1))), ...
                 'yawWrapPk',max(abs(ywr)), 'yawEnd',mean(ywr(mE)), ...
                 'xEnd',x(end),'yEnd',y(end),'zEnd',z(end), 'sMin',NaN, 'tauEnd',NaN);
    if doGov && exist('gov_s','var')
        gs = interp1(gov_s.time(:), gov_s.signals.values(:), t, 'linear','extrap');
        gt = interp1(gov_tau.time(:), gov_tau.signals.values(:), t, 'linear','extrap');
        rec.s = gs; rec.tau = gt; rec.sMin = min(gs); rec.tauEnd = gt(end);
        rec.sPre = mean(gs(t > 2 & t < TG0));             % 돌풍 전 평균 s = 평시 손해
    end
    R.(tag) = rec;

    fprintf(['>> %s | 경로이탈 %6.1f cm | 종점오차 %8.1f cm | 끝 (%.2f, %.2f, %.2f) ' ...
             '| yaw 최대 %7.1f도 (랩 %6.1f) 끝 %7.1f도 | s_min %6.4f | %.0fs\n'], ...
            tag, 100*devAll, 100*endErr, rec.xEnd, rec.yEnd, rec.zEnd, ...
            rec.yawPk, rec.yawWrapPk, rec.yawEnd, rec.sMin, el);
end

%% --- 판정 ---
fprintf('\n===== 판정 (짐 %s) =====\n', PKGTAG);
fprintf('  %-15s %12s %12s %11s %10s %8s\n', '케이스','경로이탈','종점오차','yaw최대','yaw끝','s_min');
tags = {'base','wrap','aw','wrap_aw','wrap_aw_gov','nd_wrap_aw_gov','p30_base','p30_wrap_aw'};
for k = 1:numel(tags)
    r = R.(tags{k});
    fprintf('  %-15s %10.1fcm %10.1fcm %9.1f도 %8.1f도 %8.4f\n', ...
            tags{k}, 100*r.devAll, 100*r.endErr, r.yawPk, r.yawEnd, r.sMin);
end
fprintf('\n  [0.10 N·m] 항목별\n');
sub = {'base','wrap','aw','wrap_aw','wrap_aw_gov'};
for k = 1:numel(sub)
    fprintf('    %-13s yaw끝 %8.2f도 | 종점오차 %6.2f cm | 경로이탈 %6.1f cm\n', ...
            sub{k}, R.(sub{k}).yawEnd, 100*R.(sub{k}).endErr, 100*R.(sub{k}).devAll);
end
fprintf('  [0.30 N·m] p30_base yaw끝 %8.2f도 -> p30_wrap_aw %8.2f도\n', ...
        R.p30_base.yawEnd, R.p30_wrap_aw.yawEnd);
if isfield(R.nd_wrap_aw_gov, 'sPre')
    fprintf('  평시 손해 : 돌풍 전 평균 s = %.4f  (순항 속도 %.1f%% 손실)\n', ...
            R.nd_wrap_aw_gov.sPre, 100*(1 - R.nd_wrap_aw_gov.sPre));
end

outMat = fullfile(modelDir,'diagnose','results', ['verify_yaw_robust_' PKGTAG '.mat']);
save(outMat, '-struct', 'R');
fprintf('\n결과 저장: %s\n', outMat);

%% ================= 로컬 함수 =================
function disable_drop(mdl)
% 짐 투하 로직 무력화 (probe_prop_and_mixer / diagnose_yaw_final 과 동일 처리)
dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(dropBlocks{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved', 'inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(dropBlocks)
    set_param(dropBlocks{i}, 'Value', '-1');
end
end

function move_setup(mdl, T, dx, z0, t0, tmove)
dt = 0.01; N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
u = min(max((timespot_spl - t0) / tmove, 0), 1);
xs = dx * 0.5 * (1 - cos(pi * u));
spline_data = [xs, zeros(N,1), z0*ones(N,1)];
spline_yaw = zeros(N, 1);
waypoints = [0 0 z0; dx 0 z0]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));
fprintf('이동 궤적: %g m / %g s (피크 %.3f m/s), 관측 %g s\n', dx, tmove, dx*pi/(2*tmove), T);
end

function log_gov(mdl)
r = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'QC S');
if numel(r) ~= 1; error('log_gov: QC S %d개', numel(r)); end
ref = get_param(r{1}, 'Parent');
pairs = {'QC S','gov_s'; 'QC Clock Int','gov_tau'};
for i = 1:size(pairs,1)
    src = [ref '/' pairs{i,1}];
    twName = ['To Workspace ' pairs{i,2}];
    old = find_system(ref, 'SearchDepth',1, 'Name', twName);
    if ~isempty(old); delete_block(old{1}); end
    tw = [ref '/' twName];
    add_block('simulink/Sinks/To Workspace', tw, ...
        'VariableName', pairs{i,2}, 'SaveFormat','StructureWithTime');
    add_line(ref, get_param(src,'PortHandles').Outport(1), ...
             get_param(tw,'PortHandles').Inport(1), 'autorouting','on');
end
end

function yaw_pulse_wire(mdl, amp, tOn, dur)
PER = 1000;
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
armTf1 = '';
for i = 1:numel(allBlk)
    try
        if strcmp(strtrim(regexprep(get_param(allBlk{i},'Name'), '\s+', ' ')), 'Transform Arm1')
            armTf1 = allBlk{i};
        end
    catch
    end
end
if isempty(armTf1); error('Transform Arm1 못 찾음'); end
qcSys2 = get_param(armTf1, 'Parent');
p = qcSys2;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
bodyBlk = find_system(qcSys2, 'SearchDepth', 1, 'BlockType','SubSystem', 'Name','Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, qcSys2));
if isempty(bodyBlk); error('내부 Body 서브시스템 못 찾음'); end
bph0 = get_param(bodyBlk{1}, 'PortHandles');
bconn = [bph0.LConn bph0.RConn];
attPort = -1;
for ci = 1:numel(bconn)
    if get_param(bconn(ci), 'Line') ~= -1 && attPort == -1
        attPort = bconn(ci);
    end
end
if attPort == -1; error('Body 프레임 주입점 없음'); end
extB = [qcSys2 '/Disturb Torque Z'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb Torque Z'))
    add_block('sm_lib/Forces and Torques/External Force and Torque', extB);
end
set_param(extB, 'EnableTorqueZ', 'on');
plsB = [qcSys2 '/Disturb Pulse Z'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb Pulse Z'))
    add_block('simulink/Sources/Pulse Generator', plsB, ...
        'Amplitude', num2str(amp), 'Period', num2str(PER), ...
        'PulseWidth', num2str(100*dur/PER), 'PhaseDelay', num2str(tOn));
end
spsB = [qcSys2 '/Disturb SPS Z'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb SPS Z'))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try
    set_param(spsB, 'Unit', 'N*m');
catch
end
pph = get_param(plsB,'PortHandles'); sph = get_param(spsB,'PortHandles');
if get_param(sph.Inport(1),'Line') == -1
    add_line(qcSys2, pph.Outport(1), sph.Inport(1), 'autorouting','on');
end
eph = get_param(extB,'PortHandles');
allC = [eph.LConn eph.RConn];
if numel(allC) ~= 2; error('conserving 포트 %d개(2 예상)', numel(allC)); end
orders = [2 1; 1 2]; wired = false;
for oi = 1:2
    added = [];
    try
        added(end+1) = add_line(qcSys2, attPort, allC(orders(oi,1)), 'autorouting','on'); %#ok<AGROW>
        added(end+1) = add_line(qcSys2, sph.RConn(1), allC(orders(oi,2)), 'autorouting','on'); %#ok<AGROW>
        feval(mdl, [], [], [], 'compile');
        feval(mdl, [], [], [], 'term');
        wired = true;
        fprintf('외란 배선 통과 (방향 %d): %g N·m, t=%g~%gs\n', oi, amp, tOn, tOn+dur);
        break;
    catch e
        fprintf('배선 방향 %d 실패: %s\n', oi, e.message);
        try
            feval(mdl, [], [], [], 'term');
        catch
        end
        for l2 = added
            try
                delete_line(l2);
            catch
            end
        end
    end
end
if ~wired; error('yaw 외란 배선 실패'); end
end

function log_signals(mdl)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, ...
        'VariableName', sigMap{i,2}, 'SaveFormat','StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    add_line(scope, srcPh.Outport(1), get_param(twBlk,'PortHandles').Inport(1), 'autorouting','on');
end
end
