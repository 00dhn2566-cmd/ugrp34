%% 재계획 인터벌 다리 — 플랜트 검증 (2026-08-23)
%%
%% 다리(`control_seoungjin/traj_bridge.py`)는 여기 오기 전까지 **파이썬 안에서만**
%% 확인됐다: 물리 한계를 안 넘고 기하가 안 바뀐다는 것까지. 실제 기체가 그걸
%% 따라가는지는 안 봤다. 이 저장소의 규율대로 구운 Simulink 모델을 통과시켜야 검증이다.
%%
%% 물음: 지연이 걸린 상태에서 순항 중 스펙을 깎기로 했을 때,
%%   A) 그냥 전속 기준을 계속 따라가는 것 (감쇄 결정을 궤적에 반영 안 함)
%%   B) 다리로 갈아타는 것
%% 둘 중 어느 쪽이 외란에 강건한가. 외란은 이동 중간에 넣는다.
%%
%% 입력: control_seoungjin/output/bridge_case.mat  (export_bridge_case.py 가 생성)
%% 규칙: save_system 금지. 투하 로직 무력화.
%%
%% env:
%%   BRIDGE_TAU_MS = '40'    위치 경로 지연 [ms] (자세는 5 ms 고정)
%%   BRIDGE_ATT_MS = '5'

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(fullfile(modelDir, 'diagnose'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';

% control_seoungjin/output/bridge_case.mat — modelDir 은 .../control_seoungjin/controller/<submodule>
csDir = fileparts(fileparts(modelDir));
caseFile = fullfile(csDir, 'output', 'bridge_case.mat');
if ~isfile(caseFile)
    error(['다리 케이스 없음: %s\n먼저 `python export_bridge_case.py` 를 돌릴 것'], caseFile);
end
S = load(caseFile);
t = S.t(:);  M = S.meta;
if iscell(M); M = M{1}; end

TAU_POS = str2double(getenv_or('BRIDGE_TAU_MS', '40')) * 1e-3;
TAU_ATT = str2double(getenv_or('BRIDGE_ATT_MS', '5')) * 1e-3;
AMP = 0.3; DUR = 0.3;                 % 외란 펄스 (능력 카드 R1)
tPulse = M.t_derate + 0.5;            % 감쇄 직후 = 다리가 아직 램프 중인 최악 시점
T_END = t(end);

fprintf('\n===== 다리 플랜트 검증 =====\n');
fprintf('지연 위치 %.0f ms / 자세 %.0f ms, 외란 %.1f N*m x %.1f s @ t=%.2f s\n', ...
        TAU_POS*1e3, TAU_ATT*1e3, AMP, DUR, tPulse);
fprintf('감쇄 t=%.2f s -> 배율 %.2f (v %.3f), 램프 %.2f s, 새 한계 진입 %.2f s\n\n', ...
        M.t_derate, M.s_target, M.lim_v, M.t_ramp, M.compliant_after);

cases = {'A 전속 유지(다리 없음)', S.x_base; ...
         'B 다리로 갈아탐',        S.x_bridge};
fprintf('%24s %10s %10s %11s %10s\n', '구성', '종단[cm]', '오버[cm]', '외란y[cm]', '복귀[s]');
R = struct('name', {}, 'end_cm', {}, 'over_cm', {}, 'devy_cm', {}, 'rec_s', {});

for k = 1:size(cases, 1)
    name = cases{k,1};  xr = cases{k,2};

    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);
    qctest.disable_drop(mdl);
    assignin('base', 'dly_att_s', TAU_ATT);
    assignin('base', 'dly_pos_s', TAU_POS);

    wp = [xr(1,:); xr(end,:)]';
    if norm(wp(:,1) - wp(:,2)) < 1e-6; wp(:,2) = wp(:,2) + [0.001;0;0]; end
    qctest.set_path(mdl, T_END, xr, [], wp, t(2)-t(1));
    qctest.log_signals(mdl, 'pos');
    qctest.torque_pulse(mdl, AMP, tPulse, DUR);
    qc_delay_apply(mdl);
    qc_yawwrap_apply(mdl);
    qc_antiwindup_apply(mdl, 'clamping', {'Control Yaw'});

    sim(mdl);
    tt = real_x.time(:);
    x  = real_x.signals.values(:);
    y  = interp1(real_y.time(:), real_y.signals.values(:), tt, 'linear', 'extrap');
    xrq = interp1(t, xr(:,1), tt, 'linear', 'extrap');

    % 종단 = 각 구성이 **자기 기준의 종점**에 얼마나 붙었나.
    % 다리는 정지 래치로 끝나므로 목표가 base 보다 앞이다 (같은 목표로 비교하면
    % 다리에 불리한 것이 아니라 '다른 임무'를 비교하는 셈이 된다).
    endTgt = xr(end,1);
    e.name    = name;
    e.end_cm  = 100 * abs(x(end) - endTgt);
    e.over_cm = 100 * max(0, max(x) - endTgt);
    mD = tt >= tPulse;
    e.devy_cm = 100 * max(abs(y(mD)));
    e.rec_s = NaN;
    i0 = find(tt > tPulse + DUR, 1);
    ok = abs(y) < 0.02;
    for ii = i0:numel(tt)
        if all(ok(ii:end)); e.rec_s = tt(ii) - (tPulse + DUR); break; end
    end
    e.trk_cm = 100 * max(abs(x - xrq));
    R(end+1) = e; %#ok<SAGROW>
    fprintf('%24s %10.2f %10.2f %11.2f %10s\n', name, e.end_cm, e.over_cm, ...
            e.devy_cm, num2str(e.rec_s, '%.2f'));
end

fprintf('\n===== 판정 =====\n');
if numel(R) == 2
    dY = R(2).devy_cm - R(1).devy_cm;
    fprintf('  외란 횡이탈 %+.2f cm (%+.0f%%)\n', dY, 100*dY/max(R(1).devy_cm,1e-9));
    if ~isnan(R(1).rec_s) && ~isnan(R(2).rec_s)
        fprintf('  복귀 시간   %+.2f s\n', R(2).rec_s - R(1).rec_s);
    else
        fprintf('  복귀 시간   A %s / B %s\n', num2str(R(1).rec_s,'%.2f'), num2str(R(2).rec_s,'%.2f'));
    end
    better = (R(2).devy_cm <= R(1).devy_cm) && ...
             (isnan(R(1).rec_s) || (~isnan(R(2).rec_s) && R(2).rec_s <= R(1).rec_s));
    if better
        fprintf('  -> 다리가 외란 강건성에서 우세\n');
    else
        fprintf('  -> 다리가 우세하지 않음 — 원인 분석 필요 (감쇄 폭/램프 길이/펄스 시점)\n');
    end
end
outp = fullfile(modelDir, 'diagnose', 'results', 'verify_bridge_sim.mat');
save(outp, 'R', 'TAU_POS', 'TAU_ATT', 'tPulse');
fprintf('결과 저장: %s\n', outp);

%% ================= 로컬 =================
function v = getenv_or(name, dflt)
v = getenv(name);
if isempty(v); v = dflt; end
end
