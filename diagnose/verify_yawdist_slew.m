%% yaw 외란 적응 적분 검증 ② — scan 슬루 반증 시험 (docs/YAW_DISTURBANCE_I.md §6-3)
%% 목적: 기능을 켜도 scan 미션(의도적 yaw 슬루)이 영향을 받지 않아야 한다.
%%       |psi_ref_dot| 게이트가 실제로 닫히는지 확인하는 '반증' 시험이다.
%% 구성: 외란 없음. yaw 참조를 t=3~11s 동안 0.6 rad/s 로 램프 (실기 scan 속도).
%%   g1 : 수술 + yd_gmax=1   (기준선)
%%   g3 : 수술 + yd_gmax=3
%%   g5 : 수술 + yd_gmax=5
%% 판정: **슬루 구간(t <= T1)** 안에서 g3/g5 의 yaw 궤적이 g1 과 같아야 통과 (최대 차 < 0.1도).
%%       달라지면 게이트 설계(yd_rate / yd_tau)가 틀린 것.
%%       ※ 슬루가 끝난 뒤 갈라지는 것은 정상(게이트가 열려 기능이 일하는 구간)이므로
%%         전 구간으로 재면 안 된다 — 초판이 이 창을 잘못 잡아 오판했다.
%% 규칙: save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
qc_yawdist_defaults;
mdl = 'quadcopter_package_delivery';

SLEW = 0.6;    % rad/s (실기 scan 속도, 3267d34 확정)
T0 = 3; T1 = 11; T_END = 20;

gms = [1 3 5];
S = struct();

for gi = 1:numel(gms)
    gm = gms(gi); tag = sprintf('g%d', gm);
    fprintf('\n########## slew %s ##########\n', tag);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);

    slew_setup(mdl, T_END, SLEW, T0, T1);
    log_signals(mdl);
    assignin('base', 'yd_gmax', gm);
    qc_yawdist_apply(mdl);

    tic; sim(mdl); el = toc;
    t  = real_yaw.time(:);
    yw = rad2deg(real_yaw.signals.values(:));
    r  = rad2deg(real_roll.signals.values(:));
    pc = rad2deg(real_pitch.signals.values(:));

    % 참조 yaw (도) 재구성 — 추종오차 산출용
    ref = rad2deg(min(max(t - T0, 0), T1 - T0) * SLEW);
    err = yw - ref;
    mS = t > T0 & t < T1;               % 슬루 구간
    mA = t > T1 + 1;                    % 슬루 종료 후 정착
    S.(tag) = struct('t',t,'yw',yw,'err',err,'sec',el, ...
                     'errSlew', max(abs(err(mS))), 'errEnd', mean(err(mA)), ...
                     'over', max(err(mA)), 'att', sqrt(mean(r(t>2).^2 + pc(t>2).^2)));
    fprintf('>> %s | 슬루중 최대추종오차 %7.3f도 | 종료후 잔차 %7.3f도 | 오버슈트 %7.3f도 | 자세RMS %.3f | %.0fs\n', ...
            tag, S.(tag).errSlew, S.(tag).errEnd, S.(tag).over, S.(tag).att, el);
end

fprintf('\n===== 판정 (게이트가 슬루를 배제하는가) =====\n');
% 판정 창은 반드시 '슬루 구간'으로 한정한다. 슬루가 끝난 뒤에는 게이트가 열리고
% 기능이 일하는 게 정상이므로, 전 구간으로 재면 정상 동작을 불합격으로 오판한다.
tg = linspace(0, T_END, 4001)';
y1 = interp1(S.g1.t, S.g1.yw, tg, 'linear', 'extrap');
mSlew = tg <= T1;
for gm = [3 5]
    tag = sprintf('g%d', gm);
    yk = interp1(S.(tag).t, S.(tag).yw, tg, 'linear', 'extrap');
    dSlew = max(abs(yk(mSlew) - y1(mSlew)));
    dAll  = max(abs(yk - y1));
    idx = find(abs(yk - y1) > 0.1, 1);
    if isempty(idx); tFirst = NaN; else; tFirst = tg(idx); end
    if dSlew < 0.1
        v = '합격 (슬루 중 게이트 닫힘 확인)';
    else
        v = '★불합격 - 슬루 구간에 개입함, yd_rate/yd_tau 재조정 필요';
    end
    fprintf('  %s vs g1: 슬루중(t<=%gs) 최대차 %.5f도 -> %s\n', tag, T1, dSlew, v);
    fprintf('        (참고) 전 구간 최대차 %.4f도 / 개입 시작 t=%.2fs = 슬루 종료 +%.2fs\n', ...
            dAll, tFirst, tFirst - T1);
end

save(fullfile(modelDir,'diagnose','results','verify_yawdist_slew.mat'), '-struct', 'S');
fprintf('\n결과 저장: diagnose/results/verify_yawdist_slew.mat\n');

%% ================= 로컬 함수 =================
function slew_setup(mdl, T, slew, t0, t1)
% 제자리 호버 + yaw 참조 램프 (scan 미션 모사)
dt = 0.01; N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
spline_data = repmat(hoverPoint, N, 1);
spline_yaw = min(max(timespot_spl - t0, 0), t1 - t0) * slew;
waypoints = [hoverPoint; hoverPoint + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));
fprintf('슬루 궤적: %g rad/s, t=%g~%gs (총 %.1f도)\n', slew, t0, t1, rad2deg((t1-t0)*slew));
end

function log_signals(mdl)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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
