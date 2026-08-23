%% 지연 경계 정밀 스윕 + 외란 강건성 (2026-08-22 외란 강건화 세션 A상)
%%
%% 배경: 08-18 배터리는 자세 10 ms 통과 / 20 ms 실패, 위치 0 통과 / 50 실패 까지만 알고
%%       그 사이가 비어 있다 (chain g 중단). 경계를 모르면 보상 법칙을 세울 수 없다.
%%
%% 이 스크립트가 재는 것 — 각 지연에서
%%   ① 호버 자세 지터 RMS/피크   (안정성)
%%   ② 외란 펄스 최대 이탈        (강건성 - 크기)
%%   ③ 외란 복귀 시간             (강건성 - 회복)  ★ 사용자 요구의 핵심
%%
%% 지연은 측정 경로에 Transport Delay 로 주입 (qc_delay_apply, 08-18 세션 자산).
%% 규칙: save_system 금지, 투하 로직 무력화.
%%
%% env:
%%   SWEEP_AXIS = 'att' (기본) | 'pos'      어느 경로에 지연을 넣을지
%%   SWEEP_LIST = '0 8 12 16 20 24'         [ms] 공백 구분

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';

axisSel = getenv('SWEEP_AXIS'); if isempty(axisSel); axisSel = 'att'; end
lstStr  = getenv('SWEEP_LIST');
if isempty(lstStr)
    if strcmp(axisSel, 'att'); lstStr = '0 8 12 16 20 24'; else; lstStr = '0 10 20 30 40'; end
end
MS = sscanf(lstStr, '%f')';

TAU_D = 0.3;      % N·m, 외란 펄스 크기 (능력 카드 R1 과 동일)
T_ON  = 6.0;      % s, 인가 시각
T_DUR = 0.3;      % s, 펄스 폭 (R1 규약)
T_END = 20.0;

fprintf('\n===== 지연 스윕 (%s 경로) : %s ms =====\n', axisSel, num2str(MS));
fprintf('%8s %12s %12s %11s %11s %10s\n', '지연[ms]', '호버RMS[deg]', '호버피크', ...
        '외란피크[deg]', '복귀[s]', '판정');
R = struct('ms', [], 'hov_rms', [], 'hov_pk', [], 'dist_pk', [], 'recover', [], 'sec', []);

for k = 1:numel(MS)
    ms = MS(k);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);
    qc_disable_drop(mdl);

    % 지연 변수 (qc_delay_apply 가 참조)
    if strcmp(axisSel, 'att')
        assignin('base', 'dly_att_s', ms * 1e-3);
        assignin('base', 'dly_pos_s', 1e-6);
    else
        assignin('base', 'dly_att_s', 1e-6);
        assignin('base', 'dly_pos_s', ms * 1e-3);
    end

    qc_hover_setup(mdl, T_END);
    qc_yaw_wire_none(mdl);                 % 로깅만
    qc_torque_pulse_wire(mdl, TAU_D, T_ON, T_DUR);
    qc_delay_apply(mdl);                   % ★ 지연 주입 (0 도 블록은 삽입 — A0 행 주의)
    qc_yawwrap_apply(mdl);                 % 08-22 채택분은 켜고 잰다
    qc_antiwindup_apply(mdl, 'clamping', {'Control Yaw'});

    tic; sim(mdl); el = toc;
    t  = real_roll.time(:);
    rr = rad2deg(real_roll.signals.values(:));
    pp = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear','extrap'));
    att = sqrt(rr.^2 + pp.^2);

    mH = t > 2.0 & t < T_ON;                       % 외란 전 호버 구간
    hov_rms = sqrt(mean(att(mH).^2));
    hov_pk  = max(att(mH));
    mD = t >= T_ON;
    dist_pk = max(att(mD));
    % 복귀 = 펄스 종료 후 |att| < max(1도, 호버피크x3) 를 처음으로 계속 유지
    thr = max(1.0, 3.0 * hov_pk);
    rec = NaN; i0 = find(t > T_ON + T_DUR, 1);
    ok = att < thr;
    for ii = i0:numel(t)
        if all(ok(ii:end)); rec = t(ii) - (T_ON + T_DUR); break; end
    end
    pass = (hov_rms < 0.25) && ~isnan(rec);
    R.ms(end+1)=ms; R.hov_rms(end+1)=hov_rms; R.hov_pk(end+1)=hov_pk;
    R.dist_pk(end+1)=dist_pk; R.recover(end+1)=rec; R.sec(end+1)=el;
    fprintf('%8.0f %12.4f %12.4f %11.2f %11s %10s   (%.0fs)\n', ms, hov_rms, hov_pk, ...
            dist_pk, num2str(rec,'%.2f'), qc_tf(pass), el);
end

fprintf('\n===== 경계 =====\n');
okIdx = find(arrayfun(@(i) R.hov_rms(i) < 0.25 && ~isnan(R.recover(i)), 1:numel(R.ms)));
if isempty(okIdx)
    fprintf('  전부 실패\n');
else
    fprintf('  마지막 통과 %g ms', R.ms(okIdx(end)));
    bad = setdiff(1:numel(R.ms), okIdx);
    if ~isempty(bad); fprintf(' / 첫 실패 %g ms', R.ms(bad(1))); end
    fprintf('\n');
end
outp = fullfile(modelDir,'diagnose','results', sprintf('sweep_delay_%s.mat', axisSel));
save(outp, '-struct', 'R');
fprintf('결과 저장: %s\n', outp);

%% ================= 로컬 함수 =================
function s = qc_tf(b)
if b; s = 'OK'; else; s = 'FAIL'; end
end

function qc_disable_drop(mdl)
b = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
      [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(b{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(b); set_param(b{i}, 'Value', '-1'); end
end

function qc_hover_setup(mdl, T)
dt = 0.01; N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hp = [0, 0, 1.0];
spline_data = repmat(hp, N, 1);
spline_yaw = zeros(N, 1);
waypoints = [hp; hp + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));
end

function qc_yaw_wire_none(mdl)
scope = [mdl '/Scope'];
sig = {'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
       'In Bus Element5','real_yaw';  'In Bus Element2','real_z'};
for i = 1:size(sig,1)
    nm = ['To Workspace ' sig{i,2}];
    old = find_system(scope, 'SearchDepth',1, 'Name', nm);
    if ~isempty(old); delete_block(old{1}); end
    tw = [scope '/' nm];
    add_block('simulink/Sinks/To Workspace', tw, 'VariableName', sig{i,2}, ...
              'SaveFormat','StructureWithTime');
    add_line(scope, get_param([scope '/' sig{i,1}],'PortHandles').Outport(1), ...
             get_param(tw,'PortHandles').Inport(1), 'autorouting','on');
end
end

function qc_torque_pulse_wire(mdl, amp, tOn, dur)
% roll 축(x) 외란 — 자세 루프 강건성 시험 (능력 카드 R1 규약)
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
bodyBlk = find_system(qcSys2, 'SearchDepth',1, 'BlockType','SubSystem', 'Name','Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, qcSys2));
if isempty(bodyBlk); error('내부 Body 서브시스템 못 찾음'); end
bph = get_param(bodyBlk{1}, 'PortHandles');
bconn = [bph.LConn bph.RConn];
attPort = -1;
for ci = 1:numel(bconn)
    if get_param(bconn(ci),'Line') ~= -1 && attPort == -1; attPort = bconn(ci); end
end
if attPort == -1; error('Body 프레임 주입점 없음'); end
extB = [qcSys2 '/Disturb Torque X'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb Torque X'))
    add_block('sm_lib/Forces and Torques/External Force and Torque', extB);
end
set_param(extB, 'EnableTorqueX', 'on');
plsB = [qcSys2 '/Disturb Pulse X'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb Pulse X'))
    add_block('simulink/Sources/Pulse Generator', plsB, 'Amplitude', num2str(amp), ...
        'Period', num2str(PER), 'PulseWidth', num2str(100*dur/PER), 'PhaseDelay', num2str(tOn));
end
spsB = [qcSys2 '/Disturb SPS X'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb SPS X'))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try; set_param(spsB, 'Unit', 'N*m'); catch; end
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
        feval(mdl, [], [], [], 'compile'); feval(mdl, [], [], [], 'term');
        wired = true; break;
    catch
        try; feval(mdl, [], [], [], 'term'); catch; end
        for l2 = added; try; delete_line(l2); catch; end; end
    end
end
if ~wired; error('외란 배선 실패'); end
end
