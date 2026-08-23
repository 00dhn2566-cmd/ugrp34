%% 질량 선형 보간 검증 (2026-08-23)
%%
%% 사용자 지시: "무게에 따라서 중요 튜닝 값들 선형 보간해서 넣어보고 검증해봐"
%%
%% 무엇을 대조하나 — 같은 질량에서 두 구성:
%%   A 현행  : parameters.m 스케줄 그대로 (sA = 0.75 + 0.25m, 07-19 18차 앵커)
%%   B 보간  : qc_mass_lerp_apply (08-18 채택 0 kg 앵커 <-> 1 kg 을 잇는 1차식)
%%
%% 왜 다른가 — 08-18 성능 세션이 0 kg 을 다시 튜닝해 sA 0.35 를 채택했는데
%% 그 결과가 이 머신의 parameters.m 에 동기되지 않았다. 그래서 지금은 0 kg 만
%% 별도 이산 구성(`qc_0kg_tuned_apply`)이고 그 사이 질량은 어느 쪽도 아니다.
%% 1 kg 에서는 두 구성이 같아야 한다(회귀 확인점).
%%
%% 재는 것 (질량마다):
%%   ① 호버 자세 지터 RMS      — 08-18 이 0 kg 에서 문제 삼은 축
%%   ② 1 m 이동 추종/오버슈트/종단
%%   ③ 이동 중 외란 0.3 N*m 펄스의 횡이탈·복귀
%%
%% 규칙: save_system 금지. 투하 로직 무력화.
%%
%% env:
%%   LERP_MASSES = '0 0.25 0.5 0.75 1.0'   [kg]
%%   LERP_CFGS   = 'sched lerp'            둘 중 골라 돌릴 수 있다

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(fullfile(modelDir, 'diagnose'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
mdl = 'quadcopter_package_delivery';

MASSES = getnum('LERP_MASSES', [0 0.25 0.5 0.75 1.0]);
cfgStr = getenv('LERP_CFGS'); if isempty(cfgStr); cfgStr = 'sched lerp'; end
CFGS = strsplit(strtrim(cfgStr));

DX = 1.0; Z0 = 1.0; T0 = 3.0; TM = 2.0; T_END = T0 + TM + 12.0;
AMP = 0.3; DUR = 0.3; tPulse = T0 + TM / 2;

PROG = fullfile(modelDir, 'diagnose', 'results', 'verify_mass_lerp_progress.txt');
prog = @(fmt, varargin) logrow(PROG, fmt, varargin{:});
hdr = sprintf('%6s %7s %11s %10s %10s %10s %10s', ...
    '질량', '구성', '호버RMS[°]', '종단[cm]', '추종[cm]', '외란y[cm]', '복귀[s]');
fprintf('\n===== 질량 선형 보간 대조 =====\n%s\n', hdr);
prog('==== 시작 %s ====\n%s\n', datestr(now, 'HH:MM:SS'), hdr);

R = [];
for mi = 1:numel(MASSES)
    for ci = 1:numel(CFGS)
        r = run_case(mdl, MASSES(mi), CFGS{ci}, DX, Z0, T0, TM, T_END, AMP, DUR, tPulse);
        row = sprintf('%6.2f %7s %11.4f %10.2f %10.2f %10.2f %10s  (%.0fs)', ...
                      MASSES(mi), CFGS{ci}, r.hov_rms, r.end_cm, r.trk_cm, ...
                      r.devy_cm, num2str(r.rec_s, '%.2f'), r.sec);
        fprintf('%s\n', row);
        prog('%s\n', row);
        r.m = MASSES(mi); r.cfg = CFGS{ci};
        R = [R; r]; %#ok<AGROW>
    end
end

fprintf('\n===== 대조 (보간 - 현행) =====\n');
fprintf('%6s %13s %11s %11s %11s\n', '질량', '호버RMS 변화', '종단', '추종', '외란y');
prog('==== 대조 ====\n');
for mi = 1:numel(MASSES)
    a = R([R.m] == MASSES(mi) & strcmp({R.cfg}, 'sched'));
    b = R([R.m] == MASSES(mi) & strcmp({R.cfg}, 'lerp'));
    if isempty(a) || isempty(b); continue; end
    line = sprintf('%6.2f %8.4f->%.4f %+10.2f %+10.2f %+10.2f', MASSES(mi), ...
                   a.hov_rms, b.hov_rms, b.end_cm - a.end_cm, ...
                   b.trk_cm - a.trk_cm, b.devy_cm - a.devy_cm);
    fprintf('%s\n', line);
    prog('%s\n', line);
end
outp = fullfile(modelDir, 'diagnose', 'results', 'verify_mass_lerp.mat');
save(outp, 'R', 'MASSES', 'CFGS');
fprintf('\n결과 저장: %s\n', outp);
prog('결과 저장: %s\n', outp);

%% ================= 로컬 =================
function logrow(path, fmt, varargin)
f = fopen(path, 'a');
if f > 0; fprintf(f, fmt, varargin{:}); fclose(f); end
end

function v = getnum(name, dflt)
s = getenv(name);
if isempty(s); v = dflt; else; v = sscanf(s, '%f')'; end
end

function r = run_case(mdl, m_pkg, cfg, DX, Z0, T0, TM, T_END, amp, dur, tPulse)
if bdIsLoaded(mdl); close_system(mdl, 0); end

% 질량 물성은 parameters.m **전에** 정해야 게인 스케줄이 그 질량으로 계산된다.
%
% ★ parameters.m 은 **반드시 base 에서** 돌려야 한다. 함수 안에서 그냥 부르면
%   스크립트가 이 함수의 워크스페이스에서 실행돼 kp_attitude 등이 전부 지역 변수가
%   되고, 모델은 base 만 보므로 컴파일이 깨진다. 그런데 그 실패가 나중에 외란 배선
%   시점에서야 드러나서 원인이 엉뚱해 보인다 (2026-08-23 실측 — 이걸로 두 번 헛돌았다).
pkgSize = [1 1 1] * 0.14;
vol = prod(pkgSize);
if m_pkg <= 0
    pkgDensity = 1e-6 / vol;
else
    pkgDensity = m_pkg / vol;
end
assignin('base', 'pkgSize', pkgSize);
assignin('base', 'pkgDensity', pkgDensity);
evalin('base', 'quadcopter_package_parameters;');
% parameters.m 이 pkgSize/pkgDensity 를 1 kg 값으로 덮어쓰므로 되돌린다.
assignin('base', 'pkgSize', pkgSize);
assignin('base', 'pkgDensity', pkgDensity);
assignin('base', 'm_pkg_now', max(m_pkg, 0));

load_system(mdl);
qctest.disable_drop(mdl);

dt = 0.01; N = round(T_END/dt) + 1;
t = (0:N-1)' * dt;
u = min(max((t - T0) / TM, 0), 1);
xr = DX * 0.5 * (1 - cos(pi * u));
qctest.set_path(mdl, T_END, [xr, zeros(N,1), Z0*ones(N,1)], [], ...
                [0 0 Z0; DX 0 Z0]', dt);
qctest.log_signals(mdl, 'all');
qctest.torque_pulse(mdl, amp, tPulse, dur);   % 물리 배선은 먼저 (compile 검사를 포함한다)

if strcmp(cfg, 'lerp')
    qc_mass_lerp_apply(mdl, m_pkg);
else
    % 현행: parameters.m 이 이미 m_pkg_now 로 sA_mass/sZ_mass 를 계산해 둔 상태 그대로.
    % `qc_mass_sched_apply` 는 부르지 않는다 — 그 스크립트는 08-18 판 parameters.m 의
    % 변수(filtPz_mass/nl_gmax/bias_hover_rps)를 참조하는데 이 머신엔 없어서
    % 블록이 미정의 변수를 가리키게 되고, 그러면 컴파일이 나중에(외란 배선 시점에)
    % 깨져 원인이 엉뚱한 곳으로 보인다 (2026-08-23 실측).
    qc_zsplit_apply(mdl);
end
qc_yawwrap_apply(mdl);
qc_antiwindup_apply(mdl, 'clamping', {'Control Yaw'});

tic; sim(mdl); r.sec = toc;
tt = real_x.time(:);
x  = real_x.signals.values(:);
y  = interp1(real_y.time(:), real_y.signals.values(:), tt, 'linear', 'extrap');
rr = rad2deg(interp1(real_roll.time(:),  real_roll.signals.values(:),  tt, 'linear','extrap'));
pp = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tt, 'linear','extrap'));
att = sqrt(rr.^2 + pp.^2);

mH = tt > 1.0 & tt < T0;                 % 이동 전 호버 구간
r.hov_rms = sqrt(mean(att(mH).^2));
r.end_cm  = 100 * abs(x(end) - DX);
xrq = interp1(t, xr, tt, 'linear', 'extrap');
r.trk_cm  = 100 * max(abs(x - xrq));
mD = tt >= tPulse;
r.devy_cm = 100 * max(abs(y(mD)));
r.rec_s = NaN;
i0 = find(tt > tPulse + dur, 1);
ok = abs(y) < 0.02;
for ii = i0:numel(tt)
    if all(ok(ii:end)); r.rec_s = tt(ii) - (tPulse + dur); break; end
end
end
