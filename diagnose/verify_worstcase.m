%% 실제 있을 법한 극한 상황 배터리 + 사용 전력량 (2026-08-26)
%%
%% sweep_delay_spec 은 "각 지연에서 최대 배율이 얼마인가"를 훑는 탐색이다. 이건 다르다:
%% **임무에서 실제로 마주칠 법한 최악의 조합**을 잡고, 스펙을 안 깎았을 때와 표대로
%% 깎았을 때가 어떻게 갈리는지를 시계열로 남긴다. 그림용 재료다.
%%
%% 최악을 이렇게 잡은 근거 (전부 저장소 실측/결정에서 온 값):
%%   - 위치(VIO) 경로 지연 60 ms — 30 Hz VIO + 처리. capability._LAT_POS_ANCHORS 가
%%     이미 배율 0.75 를 요구하는 지점. 80 ms 는 0.37 로 절벽이 시작된다.
%%   - 자세(IMU) 경로 지연 12 ms — LAT_ATT_CLEAN_S. 관문(16 ms) 바로 아래다.
%%   - 외란 0.3 N*m x 0.3 s 를 **이동 한복판**에 — 순항 속도에서 맞는 것이 최악
%%     (능력 카드 R1 규약, sweep_delay_spec 과 동일 자극).
%%   - 짐 1 kg, precision, 3 m 직선 이동.
%%
%% 증명하려는 것: "지연이 있는데 스펙을 안 깎으면 외란에서 못 돌아오고, 표대로 깎으면
%% 산다." 이게 08-23 지연->스펙 전략의 존재 이유다. 반증되면 전략을 고쳐야 한다.
%%
%% ── 사용 전력량 (사용자 요구 08-26) ──────────────────────────────────
%% "먼저 사용 전력 추정치 -> 실제 전력 사용량 확인 -> 피드백"
%% 그래서 케이스마다 **둘 다** 낸다:
%%   est  : 운동량 이론 추정치. 궤적만 있으면 **비행 전에도** 같은 식으로 계산된다
%%          (control_seoungjin/energy.py 가 상위 계획기용 짝). 상위가 임무를 고를 때 쓴다.
%%   act  : 이 시뮬이 실제로 쓴 양. Simscape 배터리(v*i)가 잡히면 그것이 진실이고,
%%          안 잡히면 모델이 실제로 낸 추력으로 되짚는다. 어느 쪽을 썼는지 기록한다.
%% 둘의 비(act/est)가 피드백이다 — energy.py 의 효율 상수(FM*eta)를 이 비로 교정한다.
%% ⚠ 실측이 안 잡히면 act 는 NaN 으로 둔다. 추정치를 실측인 척 쓰지 않는다.
%%
%% 규칙: save_system 금지. 투하 로직 무력화. 한 번에 한 시뮬 (RAM 16 GB).
%%
%% env:
%%   WC_ONLY = '' (전부) | 'A,C,E'   케이스 라벨로 골라 돌린다
%%
%% 산출: diagnose/results/worstcase/
%%   summary.csv        케이스별 지표 한 줄씩 (전력 포함)
%%   ts_<라벨>.csv      시계열 (t, x, y, z, xref, roll, pitch, P_est_W) — 그림용
%%   progress.txt       한 줄씩 즉시 기록 (긴 실행을 밖에서 지켜보려고)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(fullfile(modelDir, 'diagnose'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';

outDir = fullfile(modelDir, 'diagnose', 'results', 'worstcase');
if ~exist(outDir, 'dir'); mkdir(outDir); end
PROG = fullfile(outDir, 'progress.txt');   % 태그는 아래 TAG 확정 후 붙인다
% (TAG 는 질량 스위치 뒤에 정해지므로 progress 는 그때 다시 잡는다)

% 짐 질량 스위치 (sweep_delay_spec 과 같은 패턴).
% 0 kg 은 **게인도 그 앵커로** 바꿔야 한다 — 게인 스케줄은 비행 중이 아니라 이륙 전에
% 정해지는 물건이라 시험도 그렇게 구성한다. parameters.m 을 다시 부르면 안 된다
% (그 파일이 pkgSize/pkgDensity 를 1 kg 으로 되돌린다) — 뒤에 물성만 덮어쓴다.
PKG = str2double(getenv_or_num('WC_PKG', '1'));
if PKG == 0
    pkgSize = [1 1 1] * 0.14;                                  %#ok<NASGU>
    pkgDensity = 1e-6 / (pkgSize(1)*pkgSize(2)*pkgSize(3));    %#ok<NASGU>
    m_pkg_now = 0;  wind_speed = 0;                            %#ok<NASGU>
    V_REF = 1.2;                  % 0 kg 앵커 (capability._ANCHORS)
    TAG = '0kg_';
else
    V_REF = 1.6;                  % 1 kg 앵커
    TAG = '';
end
% 예측기를 켜면 출력이 기준선을 덮지 않게 태그를 나눈다 (같은 케이스 라벨을 쓰므로).
if str2double(getenv_or_num('WC_PRED', '0')) > 0
    TAG = [TAG 'pred_'];
end
if ~isempty(getenv('WC_KIPOS'))
    TAG = [TAG sprintf('ki%s_', strrep(getenv('WC_KIPOS'), '.', 'p'))];
end

DX = 3.0; Z0 = 1.0; T0 = 3.0;
TM0   = DX * pi / (2 * V_REF);    % s=1 일 때 피크 속도 = V_REF
AMP = 0.3; DUR = 0.3;             % 능력 카드 R1

PROG = fullfile(outDir, [TAG 'progress.txt']);
if exist(PROG, 'file'); delete(PROG); end

%  라벨  설명                             tau_pos tau_att   s   토크x    힘[Fx Fy Fz]
%  토크는 roll 축(x) 하나, 힘은 3축 벡터. 같은 창(tPulse ~ +DUR)에 함께 걸린다.
C = {
 'A', '기준선 - 무지연 + 외란',                 0,   5, 1.00, [AMP 0 0], [0 0 0]
 'B', '최악지연 - 스펙 안 깎음 + 외란',        60,  12, 1.00, [AMP 0 0], [0 0 0]
 'C', '최악지연 - 표대로 0.75 + 외란',         60,  12, 0.75, [AMP 0 0], [0 0 0]
 'D', '최악지연 - 더 깎음 0.55 + 외란',        60,  12, 0.55, [AMP 0 0], [0 0 0]
 'E', '절벽지연 80ms - 안 깎음 + 외란',        80,  12, 1.00, [AMP 0 0], [0 0 0]
 'F', '절벽지연 80ms - 표대로 0.37 + 외란',    80,  12, 0.37, [AMP 0 0], [0 0 0]
 'G', '자세 관문 16ms - 표대로 0.75 + 외란',   60,  16, 0.75, [AMP 0 0], [0 0 0]
 'H', '최악지연 - 안 깎음 - 외란 없음',         60,  12, 1.00, [0 0 0],   [0 0 0]
 'I', '최악지연 - 표대로 0.75 - 외란 없음',     60,  12, 0.75, [0 0 0],   [0 0 0]
 % --- 현실 대역 (사용자 지적 08-26: 60~80 ms 는 과한 가정) ---
 %     위치 지연만 변수, 자세는 5 ms 고정. A 가 0 ms 짝이다.
 'J', '현실대역 10ms + 외란',                  10,   5, 1.00, [AMP 0 0], [0 0 0]
 'K', '현실대역 20ms + 외란',                  20,   5, 1.00, [AMP 0 0], [0 0 0]
 % --- 힘 외란 / 복합 (사용자 요청 08-26) ---
 %   토크는 기체를 돌려서 간접적으로 밀지만, 바람은 직접 민다. 응답 모양이 다르다.
 %   크기 감각: 정면 0.2 m^2, Cd~1 이면 3 m/s 돌풍 = 1.1 N, 5 m/s = 3.1 N.
 'L', '20ms + 힘 y 2N (돌풍급)',              20,   5, 1.00, [0 0 0],   [0 2 0]
 'M', '20ms + 힘 y 5N (강한 돌풍)',           20,   5, 1.00, [0 0 0],   [0 5 0]
 'N', '20ms + 복합(토크x + 힘 y,z)',          20,   5, 1.00, [AMP 0 0], [0 2 -2]

 % --- 배분기가 외란 복귀에 손댈 수 있나 (사용자 질문 08-28) ---
 %   PID 는 이미 스윕 채택안이다 (0 kg: tune_0kg_r5). 남은 레버가 배분기인지 본다.
 %   위치 지연 20 ms 고정, 배율 s 만 내린다. s 가 작으면 외란 맞는 순항 속도가
 %   낮아지므로(v_peak = V_REF*s), 배분기가 손댈 수 있는 실패라면 복귀가 줄어야 한다.
 'O', '20ms + 외란 - s 0.75',                 20,   5, 0.75, [AMP 0 0], [0 0 0]
 'P', '20ms + 외란 - s 0.55',                 20,   5, 0.55, [AMP 0 0], [0 0 0]
 'Q', '20ms + 외란 - s 0.37',                 20,   5, 0.37, [AMP 0 0], [0 0 0]
 % --- 예측기 시험용: 위치 지연만 크게 (자세는 5 ms 고정) ---
 'R', '60ms + 외란 (예측기 시험)',            60,   5, 1.00, [AMP 0 0], [0 0 0]
 % --- 순수 외란 (지연 없음). '얼마까지 버티나' 를 재려고 토크를 아래로도 훑는다.
 %     0.3 N*m 에서 0 kg 가 25.6 cm 밀리는 건 가벼우니 당연하고, 능력 카드에
 %     들어가야 하는 숫자는 '못 버티는 크기' 가 아니라 '버티는 크기' 다.
 'S', '무지연 + 토크 0.10',                    0,   5, 1.00, [0.10 0 0], [0 0 0]
 'T', '무지연 + 토크 0.15',                    0,   5, 1.00, [0.15 0 0], [0 0 0]
 'U', '무지연 + 토크 0.20',                    0,   5, 1.00, [0.20 0 0], [0 0 0]
 'V', '무지연 + 힘 y 2N',                      0,   5, 1.00, [0 0 0],    [0 2 0]
 'W', '무지연 + 힘 y 5N',                      0,   5, 1.00, [0 0 0],    [0 5 0]
 % --- 외란 방향 (사용자 요청 08-28). 지금까지 토크는 roll 한 축만 쟀다.
 %   yaw 는 권한이 roll 대비 한 자릿수 약하다 (tau_max 0.317 N*m) — 같은 0.3 이라도
 %   roll 엔 여유가 있고 yaw 엔 사실상 포화다. 진짜 취약점이 거기일 수 있다.
 'X',  '무지연 + 토크 pitch 0.3',              0,   5, 1.00, [0 0.3 0], [0 0 0]
 'Y',  '무지연 + 토크 yaw 0.3',                0,   5, 1.00, [0 0 0.3], [0 0 0]
 'Y2', '무지연 + 토크 yaw 0.1',                0,   5, 1.00, [0 0 0.1], [0 0 0]
 'Z',  '무지연 + 힘 x 2N (진행방향)',          0,   5, 1.00, [0 0 0],   [2 0 0]
 'Z2', '무지연 + 힘 z -2N (하강압)',           0,   5, 1.00, [0 0 0],   [0 0 -2]
};

only = strtrim(getenv('WC_ONLY'));
if ~isempty(only)
    keep = false(size(C,1),1);
    parts = strsplit(only, ',');
    for i = 1:size(C,1)
        keep(i) = any(strcmpi(strtrim(parts), C{i,1}));
    end
    C = C(keep, :);
end

hdr = sprintf('%3s %-32s %6s %6s %6s %8s %8s %9s %8s %9s %9s %7s', ...
    '', '시나리오', 'tauP', 'tauA', 's', '종단[cm]', '추종[cm]', '외란y[cm]', ...
    '복귀[s]', 'est[Wh]', 'act[Wh]', '비');
fprintf('\n===== 극한 상황 배터리 (짐 %g kg, %g m 이동, v_ref %.1f m/s) =====\n', PKG, DX, V_REF);
fprintf('%s\n', hdr);
wc_log(PROG, '%s\n', hdr);

R = [];
for i = 1:size(C,1)
    lab = C{i,1}; desc = C{i,2};
    tauP = C{i,3}; tauA = C{i,4}; s = C{i,5}; amp = C{i,6}; fvec = C{i,7};
    r = wc_run(mdl, tauP, tauA, s, DX, Z0, T0, TM0, amp, DUR, fvec, PKG);

    % 시계열 저장 (그림용). 500 Hz 로 솎는다 (WC_TSDT 로 조절).
    % 100 Hz 였을 때: 20 ms 지연에서 roll 이 11 Hz 로 떨는 것으로 보였는데,
    % 100 Hz 로그의 나이퀴스트가 50 Hz 라 89/111 Hz 가 접혀 온 것인지 가릴 수
    % 없었다. 진동의 참 주파수를 판정하려면 로그가 그보다 충분히 빨라야 한다.
    tsDt = str2double(getenv_or_num('WC_TSDT', '0.002'));
    tq = (r.t(1):tsDt:r.t(end))';
    TS = [tq, interp1(r.t, r.x, tq), interp1(r.t, r.y, tq), ...
          interp1(r.t, r.z, tq), interp1(r.t, r.xref, tq), ...
          interp1(r.t, r.roll, tq), interp1(r.t, r.pitch, tq), ...
          interp1(r.t, r.yaw, tq), interp1(r.t, r.Pest, tq)];
    fid = fopen(fullfile(outDir, ['ts_' TAG lab '.csv']), 'w');
    fprintf(fid, 't,x,y,z,xref,roll,pitch,yaw,P_est_W\n');
    fprintf(fid, '%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f\n', TS');
    fclose(fid);

    if isnan(r.act_Wh); ratio = NaN; else; ratio = r.act_Wh / max(r.est_Wh, 1e-12); end
    row = sprintf('%3s %-32s %6g %6g %6.2f %8.2f %8.2f %9.2f %8s %9.4f %9s %7s', ...
        lab, desc, tauP, tauA, s, r.end_cm, r.trk_cm, r.devy_cm, ...
        num2str(r.rec_s, '%.2f'), r.est_Wh, num2str(r.act_Wh, '%.4f'), ...
        num2str(ratio, '%.3f'));
    fprintf('%s   (%.0fs)\n', row, r.sec);
    wc_log(PROG, '%s   (%.0fs)\n', row, r.sec);

    e.lab = lab; e.desc = desc; e.tauP = tauP; e.tauA = tauA; e.s = s; e.amp = norm(amp);
    e.fvec = fvec;
    e.vpk = r.vpk; e.end_cm = r.end_cm; e.trk_cm = r.trk_cm;
    e.devy_cm = r.devy_cm; e.rec_s = r.rec_s; e.sec = r.sec; e.tPulse = r.tPulse;
    e.est_Wh = r.est_Wh; e.act_Wh = r.act_Wh; e.act_src = r.act_src;
    e.ratio = ratio; e.P_mean = r.P_mean; e.P_peak = r.P_peak;
    e.est_Wh_per_m = r.est_Wh / DX; e.T_end = r.t(end);
    if isempty(R); R = e; else; R(end+1) = e; end %#ok<SAGROW>
end

fid = fopen(fullfile(outDir, [TAG 'summary.csv']), 'w');
% desc 에 쉼표를 넣지 말 것 — 따옴표 없이 쓰므로 CSV 필드가 밀린다.
fprintf(fid, ['label,desc,tau_pos_ms,tau_att_ms,s,pulse_Nm,v_peak,end_cm,track_cm,' ...
              'dev_y_cm,recover_s,t_pulse,T_end,wall_s,fx_N,fy_N,fz_N,' ...
              'energy_est_Wh,energy_act_Wh,energy_ratio,energy_src,' ...
              'est_Wh_per_m,P_mean_W,P_peak_W\n']);
for i = 1:numel(R)
    if isnan(R(i).rec_s); recs = ''; else; recs = sprintf('%.3f', R(i).rec_s); end
    if isnan(R(i).act_Wh); acts = ''; else; acts = sprintf('%.5f', R(i).act_Wh); end
    if isnan(R(i).ratio);  rats = ''; else; rats = sprintf('%.4f', R(i).ratio); end
    fprintf(fid, '%s,%s,%g,%g,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%.1f,%.5f,%s,%s,%s,%.5f,%.2f,%.2f\n', ...
        R(i).lab, R(i).desc, R(i).tauP, R(i).tauA, R(i).s, R(i).amp, R(i).vpk, ...
        R(i).end_cm, R(i).trk_cm, R(i).devy_cm, recs, R(i).tPulse, R(i).T_end, R(i).sec, ...
        R(i).fvec(1), R(i).fvec(2), R(i).fvec(3), ...
        R(i).est_Wh, acts, rats, R(i).act_src, R(i).est_Wh_per_m, R(i).P_mean, R(i).P_peak);
end
fclose(fid);
save(fullfile(outDir, [TAG 'worstcase.mat']), 'R');

fprintf('\n===== 판정 =====\n');
labs = {R.lab};
wc_pair(R, labs, 'B', 'C', '60 ms 지연: 안 깎음 vs 표대로(0.75)');
wc_pair(R, labs, 'E', 'F', '80 ms 지연: 안 깎음 vs 표대로(0.37)');
wc_pair(R, labs, 'B', 'H', '60 ms 지연 무보정: 외란 있음 vs 없음');

fprintf('\n===== 사용 전력량 (추정치 -> 실측 -> 피드백) =====\n');
good = ~isnan([R.ratio]);
for i = 1:numel(R)
    fprintf('  %s  est %.4f Wh (%.4f Wh/m, 평균 %.0f W / 최대 %.0f W)   act %s Wh [%s]   비 %s\n', ...
        R(i).lab, R(i).est_Wh, R(i).est_Wh_per_m, R(i).P_mean, R(i).P_peak, ...
        num2str(R(i).act_Wh, '%.4f'), R(i).act_src, num2str(R(i).ratio, '%.3f'));
end
if any(good)
    k = mean([R(good).ratio]);
    fprintf('\n  ** 피드백: act/est 평균 %.3f (표본 %d)\n', k, sum(good));
    fprintf('     energy.py 의 효율곱을 이 비로 나눠 교정한다 (FM*eta <- FM*eta / %.3f).\n', k);
    fprintf('     energy_feedback.json 으로 상위에 올릴 값이 이것이다.\n');
    fid = fopen(fullfile(outDir, [TAG 'energy_feedback.json']), 'w');
    fprintf(fid, '{\n');
    fprintf(fid, '  "source": "simscape_worstcase",\n');
    fprintf(fid, '  "n_samples": %d,\n', sum(good));
    fprintf(fid, '  "ratio_act_over_est": %.6f,\n', k);
    fprintf(fid, '  "note": "energy.py 의 FM*eta 를 이 비로 나눠 교정. 가드레일은 INTERFACE_SPEC 6절과 같다."\n');
    fprintf(fid, '}\n');
    fclose(fid);
else
    fprintf('\n  ** 실측 전력을 못 잡았다 (%s). 추정치만 유효하다.\n', R(1).act_src);
    fprintf('     모델에 Simscape 로깅이 꺼져 있으면 그렇다 — 켜야 act 가 나온다.\n');
end

fprintf('\n결과: %s\n', outDir);
wc_log(PROG, '==== 끝 ====\n');


%% ================= 로컬 =================

function r = wc_run(mdl, tau_pos_ms, tau_att_ms, s, DX, Z0, T0, TM0, amp, dur, fvec, pkg)
if nargin < 11 || isempty(fvec); fvec = [0 0 0]; end
if nargin < 12 || isempty(pkg); pkg = 1; end
TM = TM0 / s;
T_END = T0 + TM + 12.0;
tPulse = T0 + TM / 2;             % 이동 한복판 = 순항 속도에서 맞는 것이 최악

if bdIsLoaded(mdl); close_system(mdl, 0); end
load_system(mdl);
qctest.disable_drop(mdl);
if pkg == 0
    qc_0kg_tuned_apply(mdl);      % 08-18 채택 0 kg 구성 (parameters.m 미동기 대체분)
end
assignin('base', 'dly_att_s', max(tau_att_ms, 0) * 1e-3);
assignin('base', 'dly_pos_s', max(tau_pos_ms, 0) * 1e-3);
% WC_KIPOS: 위치 적분 게인을 덮어쓴다 (기본 = 안 건드림).
% 08-29 가설 시험용 — 외란 뒤 8 s 꼬리가 적분 되감기인가.
% 오버슈트가 외란 크기에 비례하고 감쇠 시정수는 고정(~8 s)인 것이 되감기 서명이라,
% ki=0 이면 오버슈트와 꼬리가 함께 사라져야 한다. 안 사라지면 원인은 딴 데다.
kiEnv = getenv('WC_KIPOS');
if ~isempty(kiEnv)
    assignin('base', 'ki_position', str2double(kiEnv));
    fprintf('[WC_KIPOS] ki_position -> %s\n', kiEnv);
end
[tr, xr, vpk] = qctest.raised_cos_move(mdl, T_END, DX, Z0, T0, TM);
qctest.log_signals(mdl, 'all');   % 자세도 필요하다 — 전력 추정에 경사가 들어간다
axl = 'xyz';
for k = 1:3
    if amp(k) ~= 0
        qctest.torque_pulse(mdl, amp(k), tPulse, dur, axl(k));
    end
end
% 힘 외란 — 0 이 아닌 축마다 블록을 하나씩 붙인다. 물리 연결은 분기가 되므로
% 토크 블록과 같은 Body 프레임 포트에 함께 걸린다 (실패하면 compile 에서 잡힌다).
for k = 1:3
    if fvec(k) ~= 0
        qctest.force_pulse(mdl, fvec(k), tPulse, dur, axl(k));
    end
end
if tau_att_ms > 0 || tau_pos_ms > 0
    qc_delay_apply(mdl);          % 0/0 이면 블록을 아예 안 넣는다 (항등 기준선)
    % 지연 보상 예측기 (WC_PRED=1). 지연 블록 **뒤에** 리드 보상기를 끼운다.
    % 목적: delay_compensator.py 가 추정기 단독으로 보인 이득이 폐루프에서도
    % 나오는지, 특히 roll 11 Hz 한계사이클에도 듣는지를 가른다.
    if str2double(getenv_or_num('WC_PRED', '0')) > 0
        qc_predictor_apply(mdl);
    end
end
qc_yawwrap_apply(mdl);
qc_antiwindup_apply(mdl, 'clamping', {'Control Yaw'});

tic; sim(mdl); el = toc;
t = real_x.time(:);
x = real_x.signals.values(:);
y = interp1(real_y.time(:), real_y.signals.values(:), t, 'linear', 'extrap');
z = interp1(real_z.time(:), real_z.signals.values(:), t, 'linear', 'extrap');
roll  = interp1(real_roll.time(:),  real_roll.signals.values(:),  t, 'linear', 'extrap');
pitch = interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear', 'extrap');
yaw   = interp1(real_yaw.time(:),   real_yaw.signals.values(:),   t, 'linear', 'extrap');
xrq = interp1(tr, xr, t, 'linear', 'extrap');

r.t = t; r.x = x; r.y = y; r.z = z; r.xref = xrq;
r.roll = roll; r.pitch = pitch; r.yaw = yaw;
r.vpk    = vpk;
r.end_cm = 100 * abs(x(end) - DX);
r.trk_cm = 100 * max(abs(x - xrq));
mD = t >= tPulse;
r.devy_cm = 100 * max(abs(y(mD)));
r.rec_s = NaN;
if any(amp ~= 0) || any(fvec ~= 0)
    i0 = find(t > tPulse + dur, 1);
    ok = abs(y) < 0.02;                       % 2 cm 밴드 (sweep_delay_spec 과 동일)
    for ii = i0:numel(t)
        if all(ok(ii:end)); r.rec_s = t(ii) - (tPulse + dur); break; end
    end
end

% ---- 전력: 추정치 ----
[r.Pest, r.est_Wh] = wc_power_est(t, z, roll, pitch);
r.P_mean = mean(r.Pest); r.P_peak = max(r.Pest);
% ---- 전력: 실측 ----
[r.act_Wh, r.act_src] = wc_power_act(mdl, t);

r.tPulse = tPulse; r.sec = el;
end


function [P, Wh] = wc_power_est(t, z, roll, pitch)
% 운동량 이론 기반 전기 동력 **추정치**.
% control_seoungjin/energy.py 의 estimate_power 와 **같은 식**이어야 한다 — 상위
% 계획기가 비행 전에 쓰는 것이 그쪽이고, 여기서 그 식을 실측으로 채점하는 구조다.
%
%   추력    T_tot = m*(g + z_ddot) / (cos(roll)*cos(pitch))
%   유도동력 P_ideal = n*(T_tot/n)^1.5 / sqrt(2*rho*A)      (호버 유도동력, 정지대기)
%   전기동력 P = P_ideal / (FM*eta)
%
% FM(프로펠러 효율) 0.70, eta(모터+ESC) 0.80 은 **실측 전까지의 통상값**이다.
% 이 둘의 곱이 곧 act/est 비로 교정될 자리다 (energy_feedback.json).
% 검산: 1 kg 짐 호버에서 P = 267 W (8.5 g/W) — 소형 멀티로터 실측 대역.
RHO = 1.225;  R_PROP = 0.127;  N_ROT = 4;  G = 9.81;
FM = 0.70;  ETA = 0.80;
A = pi * R_PROP^2;
M_TOT = evalin('base', 'drone_mass') + evalin('base', 'm_pkg_now');

% z 가속도: 가변 스텝이라 균일격자로 옮겨 미분하고 되돌린다
tu = (t(1):0.005:t(end))';
zu = interp1(t, z, tu, 'linear', 'extrap');
zdd = gradient(gradient(zu, tu), tu);
zdd = movmean(zdd, 21);                       % 수치미분 잡음 억제
zdd = interp1(tu, zdd, t, 'linear', 'extrap');

ct = max(cos(roll) .* cos(pitch), 0.2);       % 60도 넘게 기울면 이 근사는 무의미
T_tot = max(M_TOT * (G + zdd) ./ ct, 0);
P = N_ROT * (T_tot / N_ROT).^1.5 / sqrt(2 * RHO * A) / (FM * ETA);
Wh = trapz(t, P) / 3600;
end


function [Wh, src] = wc_power_act(mdl, t) %#ok<INUSD>
% 이 시뮬이 **실제로** 쓴 전기 에너지. Simscape 배터리가 진실이다.
% 못 잡으면 NaN 을 돌려준다 — 추정치를 실측인 척 쓰지 않는다.
Wh = NaN;  src = 'none';
simName = ['simlog_' mdl];
try
    sl = evalin('caller', simName);
catch
    try
        sl = evalin('base', simName);
    catch
        src = 'no_simlog';
        return;
    end
end
try
    ib = sl.Quadcopter.Electrical.Battery.i.series.values('A');
    tb = sl.Quadcopter.Electrical.Battery.i.series.time;
    try
        vb = sl.Quadcopter.Electrical.Battery.v.series.values('V');
    catch
        % 전압 채널이 없으면 공칭 전압을 쓴다 (근사임을 이름에 남긴다)
        vb = ones(size(ib)) * wc_nominal_v();
        src = 'battery_i_nomV';
    end
    if strcmp(src, 'none'); src = 'battery_vi'; end
    Wh = trapz(tb, abs(vb .* ib)) / 3600;
catch ME
    src = ['simlog_fail:' ME.identifier];
end
end


function v = wc_nominal_v()
try
    v = evalin('base', 'battery_voltage');
catch
    v = 22.2;      % 6S 공칭 (qc_motor.hpp 의 Vbatt 와 같은 값)
end
end


function wc_pair(R, labs, a, b, ttl)
ia = find(strcmp(labs, a), 1); ib = find(strcmp(labs, b), 1);
if isempty(ia) || isempty(ib); return; end
A = R(ia); B = R(ib);
fprintf('  %s\n', ttl);
fprintf('     %s: 추종 %.2f cm / 외란y %.2f cm / 복귀 %s s / est %.4f Wh\n', ...
        a, A.trk_cm, A.devy_cm, num2str(A.rec_s, '%.2f'), A.est_Wh);
fprintf('     %s: 추종 %.2f cm / 외란y %.2f cm / 복귀 %s s / est %.4f Wh\n', ...
        b, B.trk_cm, B.devy_cm, num2str(B.rec_s, '%.2f'), B.est_Wh);
end


function v = getenv_or_num(name, dflt)
v = getenv(name);
if isempty(v); v = dflt; end
end

function wc_log(path, fmt, varargin)
% 한 줄마다 열고 닫는다 — -batch 의 stdout 은 리다이렉트되면 블록 버퍼링돼
% 끝날 때까지 아무것도 안 보인다. 긴 실행을 밖에서 지켜보려면 이 방식이어야 한다.
fid = fopen(path, 'a');
if fid > 0
    fprintf(fid, fmt, varargin{:});
    fclose(fid);
end
end
