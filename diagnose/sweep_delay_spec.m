%% B상 — 지연 × 스펙배율: "지연이 있어도 외란 강건성이 살아있는 최대 스펙" 찾기
%%
%% 2026-08-23. 사용자 요구:
%%   "시간 지연이 있어도 disturbance 에 대한 강건성은 계속 유지해야 한다.
%%    예상 지연 시간을 계속 업데이트하면서 상위 계획기에 보고할 spec 을 조정한다."
%%   방침(사용자 정정): **게인은 손대지 않는다.** 지연에 맞춰 스펙을 여유 있게 깎고
%%   path_time 이 그 안에서 궤적을 만들게 한다.
%%
%% ※ 탐색은 **내림차순 전수**여야 한다. 이분 탐색을 쓰면 안 된다 —
%%   2026-08-23 실측에서 복귀 시간이 배율에 단조롭지 않았다 (tau=60 ms 에서
%%   s 1.00/0.75/0.55/0.40/0.28 -> 복귀 6.17/12.19/11.06/7.58/1.98 s).
%%   중간 배율이 어떤 진동 모드를 더 잘 여기하는 듯하고, 이분하면 통과 구간을 건너뛴다.
%%
%% 그래서 재는 것 — 각 지연 tau 에서 스펙 배율 s 를 1.0 부터 내려가며
%%   1) 목표 도달 오차 (사용자 요구: 수 cm 이내)
%%   2) 이동 중 외란 펄스(0.3 N*m x 0.3 s, roll 축)의 최대 이탈과 복귀
%% 둘 다 만족하는 **첫(=가장 큰) s** 가 그 지연의 허용 스펙이다.
%%
%% 스펙 배율의 물리적 의미 — 시간축만 늘린다 (capability.py 와 같은 대수):
%%   이동시간 TM = TM0 / s  =>  v ~ s, a ~ s^2, j ~ s^3, snap ~ s^4
%% 경로 기하는 그대로라 상위는 "s 배로 느리게"만 알면 된다.
%%
%% 규칙: save_system 금지. 투하 로직 무력화 필수 (qctest.disable_drop 주석 참조).
%%
%% env:
%%   SPEC_TAU_LIST = '0 20 40 60 80'   [ms] 위치(VIO) 경로 지연 — 지배적인 쪽
%%   SPEC_ATT_MS   = '5'               [ms] 자세(IMU) 경로 지연, 전 조건 고정
%%   SPEC_S_LIST   = '1.0 0.75 0.55 0.40 0.28 0.20'   내림차순 스펙 배율
%%   SPEC_PKG      = '1' (기본) | '0'   짐 질량 [kg]. 0 이면 08-18 채택 0 kg 튜닝을 얹는다

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(fullfile(modelDir, 'diagnose'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';

TAU    = getenv_num('SPEC_TAU_LIST', [0 20 40 60 80]);
ATT_MS = getenv_num('SPEC_ATT_MS', 5);
SLIST  = getenv_num('SPEC_S_LIST', [1.00 0.75 0.55 0.40 0.28 0.20]);
PKG    = getenv_num('SPEC_PKG', 1);

% 짐 0 kg — 질량과 함께 게인도 그 앵커로 바꿔야 한다. 게인 스케줄은 비행 중이 아니라
% **이륙 전에 한 번** 정해지는 물건이라, 시험도 그렇게 구성한다.
if PKG == 0
    % parameters.m 을 다시 부르면 안 된다 — 그 파일이 pkgSize/pkgDensity 를 1 kg 으로
    % 되돌린다. tune_0kg_r5 와 같은 순서: parameters.m 뒤에 물성만 덮어쓴다.
    pkgSize = [1 1 1] * 0.14;                                  %#ok<NASGU>
    pkgDensity = 1e-6 / (pkgSize(1)*pkgSize(2)*pkgSize(3));    %#ok<NASGU>
    m_pkg_now = 0;  wind_speed = 0;                            %#ok<NASGU>
    V_REF = 1.2;  A_REF = 1.0;      % 0 kg 앵커 (capability._ANCHORS)
else
    V_REF = 1.6;  A_REF = 1.6;      % 1 kg 앵커
end

DX = 3.0; Z0 = 1.0; T0 = 3.0;
TM0   = DX * pi / (2 * V_REF);   % s=1 일 때 피크 속도가 V_REF 가 되는 이동시간
TAU_D = 0.3; T_DUR = 0.3;        % 외란 펄스 (능력 카드 R1 규약)

GATE_END_CM = 5.0;               % 목표 도달 오차 상한 — 사용자 요구 "수 cm"
GATE_DEV_M  = 1.0;               % 이 이상 옆으로 밀리면 발산 취급
% 복귀 시간 상한. '복귀가 존재하는가'만 보면 안 된다 — 2026-08-23 실측에서
% tau=40 ms 가 종단오차 1.4 cm 로 통과했는데 복귀에 12.9 s 가 걸렸다 (무지연 1.7 s).
% 사실상 진동이 안 잦아드는 것이라, 강건성 유지라는 목표에 비추면 실패다.
%
% 기본 3.0 s 는 1 kg 무지연 기준(1.73 s)의 약 2배다. 질량이 바뀌면 기준도 바뀐다 —
% 0 kg 은 같은 0.3 N·m 에 훨씬 크게 밀리므로(08-18: 밀림 0.26 m, 재진입 5.3 s)
% 절대값 3 s 를 그대로 쓰면 무지연에서조차 탈락한다. 그 질량의 tau=0 복귀를 먼저 재고
% 그 2배로 잡을 것 (SPEC_GATE_REC 로 넘긴다).
GATE_REC_S = str2double(getenv_str('SPEC_GATE_REC', '3.0'));

% 진행 상황 파일 — -batch 의 stdout 은 리다이렉트되면 블록 버퍼링돼 끝까지 안 보인다.
% 한 줄마다 열고 닫아 즉시 디스크에 남긴다 (긴 스윕을 밖에서 지켜볼 수 있게).
PROG = fullfile(modelDir, 'diagnose', 'results', 'sweep_delay_spec_progress.txt');
prog = @(fmt, varargin) logrow(PROG, fmt, varargin{:});

hdr = sprintf('%7s %6s %7s %9s %9s %9s %9s %8s', ...
    'tau[ms]', 's', 'v[m/s]', '종단[cm]', '오버[cm]', '외란y[cm]', '복귀[s]', '판정');
fprintf('\n===== B상 지연x스펙 (짐 %g kg, 자세 %g ms 고정, 위치 지연 스윕) =====\n', PKG, ATT_MS);
fprintf('기저: %g m 이동, s=1 -> v %.2f m/s (TM %.2f s), 외란 %.1f N*m x %.1f s @ 이동 중간\n\n', ...
        DX, V_REF, TM0, TAU_D, T_DUR);
fprintf('%s\n', hdr);
prog('==== 시작: 짐 %g kg, 자세 %g ms, tau %s ms ====\n%s\n', PKG, ATT_MS, num2str(TAU), hdr);

ROWS = [];
for ti = 1:numel(TAU)
    tau = TAU(ti);
    smax = NaN;
    for si = 1:numel(SLIST)
        s = SLIST(si);
        r = run_case(mdl, tau, ATT_MS, s, DX, Z0, T0, TM0, TAU_D, T_DUR, PKG);
        pass = (r.end_cm <= GATE_END_CM) && ~isnan(r.rec_s) && ...
               (r.rec_s <= GATE_REC_S) && (r.devy_m < GATE_DEV_M);
        row = sprintf('%7g %6.2f %7.3f %9.2f %9.2f %9.2f %9s %8s  (%.0fs)', ...
                      tau, s, r.vpk, r.end_cm, r.over_cm, 100*r.devy_m, ...
                      num2str(r.rec_s, '%.2f'), tf(pass), r.sec);
        fprintf('%s\n', row);
        prog('%s\n', row);
        r.tau = tau; r.s = s; r.pass = pass;
        ROWS = [ROWS; r]; %#ok<AGROW>
        if pass; smax = s; break; end
    end
    if isnan(smax)
        msg = sprintf('  -> tau=%g ms : 목록 내 통과 없음 (운용 불가 또는 s<%.2f 필요)', tau, min(SLIST));
    else
        msg = sprintf('  -> tau=%g ms : 허용 스펙 배율 s_max = %.2f  (v %.3f m/s)', tau, smax, V_REF * smax);
    end
    fprintf('%s\n\n', msg);
    prog('%s\n', msg);
end

fprintf('\n===== limits(tau) 표 =====\n');
fprintf('%7s %8s %9s %9s %9s %10s\n', 'tau[ms]', 's_max', 'v', 'a', 'j', 'snap');
prog('==== limits(tau) ====\n');
for ti = 1:numel(TAU)
    sel = ROWS([ROWS.tau] == TAU(ti) & [ROWS.pass]);
    if isempty(sel)
        line = sprintf('%7g %8s  (운용 불가)', TAU(ti), '-');
    else
        s = max([sel.s]);
        line = sprintf('%7g %8.2f %9.3f %9.3f %9.3f %10.2f', TAU(ti), s, ...
                       V_REF*s, A_REF*s^2, 8.0*s^3, 64.0*s^4);
    end
    fprintf('%s\n', line);
    prog('%s\n', line);
end
outp = fullfile(modelDir, 'diagnose', 'results', sprintf('sweep_delay_spec_%gkg.mat', PKG));
save(outp, 'ROWS', 'TAU', 'SLIST', 'ATT_MS', 'V_REF', 'A_REF', 'TM0', 'PKG');
fprintf('\n결과 저장: %s\n', outp);
prog('결과 저장: %s\n', outp);

%% ================= 로컬 =================
function logrow(path, fmt, varargin)
f = fopen(path, 'a');
if f > 0; fprintf(f, fmt, varargin{:}); fclose(f); end
end

function v = getenv_str(name, dflt)
v = getenv(name);
if isempty(v); v = dflt; end
end

function v = getenv_num(name, dflt)
s = getenv(name);
if isempty(s); v = dflt; else; v = sscanf(s, '%f')'; end
end

function s = tf(b)
if b; s = 'OK'; else; s = 'FAIL'; end
end

function r = run_case(mdl, tau_pos_ms, tau_att_ms, s, DX, Z0, T0, TM0, amp, dur, pkg)
TM = TM0 / s;
T_END = T0 + TM + 12.0;
tPulse = T0 + TM / 2;             % 이동 한복판 — 순항 속도에서 맞는 것이 최악

if bdIsLoaded(mdl); close_system(mdl, 0); end
load_system(mdl);
qctest.disable_drop(mdl);
if pkg == 0
    qc_0kg_tuned_apply(mdl);      % 08-18 채택 0 kg 구성 (parameters.m 미동기 대체분)
end
assignin('base', 'dly_att_s', max(tau_att_ms, 0) * 1e-3);
assignin('base', 'dly_pos_s', max(tau_pos_ms, 0) * 1e-3);
[tr, xr, vpk] = qctest.raised_cos_move(mdl, T_END, DX, Z0, T0, TM);
qctest.log_signals(mdl, 'pos');
qctest.torque_pulse(mdl, amp, tPulse, dur);
if tau_att_ms > 0 || tau_pos_ms > 0
    qc_delay_apply(mdl);          % 0/0 이면 블록 자체를 넣지 않는다 (항등 기준선)
end
qc_yawwrap_apply(mdl);            % 08-22 채택분은 켜고 잰다
qc_antiwindup_apply(mdl, 'clamping', {'Control Yaw'});

tic; sim(mdl); el = toc;
t = real_x.time(:);
x = real_x.signals.values(:);
y = interp1(real_y.time(:), real_y.signals.values(:), t, 'linear', 'extrap');

r.vpk     = vpk;
r.end_cm  = 100 * abs(x(end) - DX);                % 종단 도달 오차
r.over_cm = 100 * max(0, max(x) - DX);             % 폐루프 오버슈트
xrq = interp1(tr, xr, t, 'linear', 'extrap');
r.trk_cm  = 100 * max(abs(x - xrq));               % 기준 대비 최대 이탈 (참고)
mD = t >= tPulse;
r.devy_m  = max(abs(y(mD)));                       % 외란이 민 옆방향 최대
% 복귀 = 펄스 종료 후 |y| < 2 cm 를 처음으로 '끝까지' 유지
r.rec_s = NaN;
i0 = find(t > tPulse + dur, 1);
ok = abs(y) < 0.02;
for ii = i0:numel(t)
    if all(ok(ii:end)); r.rec_s = t(ii) - (tPulse + dur); break; end
end
r.sec = el;
end
