%% 전체 파이프라인 스텝응답 (18차, 사용자 요청): 성형기(traj_smoother) 포함 끝단 고전 지표
%% 배경: 생 스텝은 입력 계약 위반(클램프 릴레이 한계사이클) - 정당한 스텝 시험은
%%       "스텝 명령 -> 성형기 -> 제어기" 전체 사슬. 소신호(0.1m)는 엔벌로프 안이라
%%       성형기가 사실상 무개입 -> 제어기 고유 스텝응답. 대신호(1m)는 성형 포함 끝단.
%% 지표: rise time(실측 x의 10->90%), 정착(±2%A 최종 진입), 오버슈트(%A),
%%       정상상태 오차(마지막 2s 평균), z피크. 성형 몫 분리용으로 기준 궤적 99% 도달시간 병기.
%% 구성: {precision, agile} x {0.1m, 1.0m} - 1kg 탑재.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);
qc_zsplit_apply(mdl);   % z분리 (precision에선 무해 - posErrSatZ==posErrSat)

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

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 20; tStep = 3;   % T=20: 느린 적분기(Ti=200s) 방전 관찰 여유
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('timespot_spl', tt);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

% 구성: {라벨, 프로파일, 스텝 크기 A[m]}
cfgs = { ...
    'prec 0.1m ', 'precision', 0.1; ...
    'prec 1.0m ', 'precision', 1.0; ...
    'agile 0.1m', 'agile',     0.1; ...
    'agile 1.0m', 'agile',     1.0; ...
};

nC = size(cfgs,1);
rows = nan(nC, 9);
fprintf('===== 전체 파이프라인 스텝응답 (성형기 포함, 1kg): rise/정착/오버슈트/SSE =====\n');
for c = 1:nC
    prof = cfgs{c,2};
    A = cfgs{c,3};

    clearvars ctrl_profile
    ctrl_profile = prof;
    quadcopter_package_parameters;    % 프로파일별 게인 산출 (1kg 기본 물성)

    % 스텝 명령(0.1s 램프 근사) -> 성형기. 소신호는 게이트 통과 수준이라 무개입에 가깝고
    % 대신호는 jerk-limited S-커브로 성형됨 (실사용 파이프라인과 동일 경로).
    tau = min(max((tt-tStep)/0.1,0),1);
    xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
    sm = traj_zv(tt, sm, 1.80, 'zvd');   % 실사용 사슬 완성: 짐 진자(1.80Hz) 잔류진동 소거
    wp = [0 0 1; max(A,0.5) 0 1]';
    mws.assignin('spline_data', sm);
    mws.assignin('waypoints', wp);
    mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(wp));

    % 성형 몫: 기준 궤적이 99%A 도달하는 시각 (명령 시점 기준)
    iRef = find(sm(:,1) >= 0.99*A, 1);
    tRef99 = tt(iRef) - tStep;

    try
        sim(mdl);
    catch e
        fprintf('%-10s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.002:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz);

    i10 = find(xg >= 0.1*A & tu > tStep, 1);
    i90 = find(xg >= 0.9*A & tu > tStep, 1);
    riseT = NaN;
    if ~isempty(i10) && ~isempty(i90); riseT = tu(i90) - tu(i10); end
    % 정착: |x-A| <= 2%A 를 끝까지 유지하기 시작하는 시각 (명령 시점 기준)
    iS = find(tu > tStep, 1);
    settleAt = @(tol) local_settle(tu, xg, A, tol, iS, tStep);
    settleT  = settleAt(0.02 * A);   % 고전 ±2%
    settleAb = settleAt(0.02);       % 절대 ±2cm (짐 진동 대비 실용 밴드)
    ovPct = max(0, (max(xg) - A) / A) * 100;
    sse   = mean(xg(tu >= T-2)) - A;
    zpk   = max(abs(zg(tu > 1) - 1)) * 100;
    rows(c,:) = [A, double(strcmp(prof,'agile')), tRef99, riseT, settleT, ovPct, sse*1000, zpk, settleAb];
    fprintf('%-10s | 성형도달 %5.2fs | rise %5.2fs | 정착±2%% %5.2fs | 정착±2cm %5.2fs | 오버 %5.2f%% | SSE %+6.1fmm | z피크 %4.1fcm\n', ...
        cfgs{c,1}, tRef99, riseT, settleT, settleAb, ovPct, sse*1000, zpk);
    % 시계열 덤프 (판독용): 기준 vs 실측
    xr_ref = interp1(tt, sm(:,1), tu);
    Ts = table(tu, xr_ref, xg, 'VariableNames', {'t','x_ref','x_meas'});
    writetable(Ts, fullfile(modelDir, 'diagnose', 'results', ...
        sprintf('step_ts_%s_%gm.csv', prof, A)));
end
fprintf('(성형도달 = 성형기 몫(기준 궤적 99%% 도달), rise/정착은 실측 위치 기준 - 파이프라인 끝단 지표)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'step_m','is_agile','t_ref99_s','rise_s','settle2pct_s','overshoot_pct','sse_mm','z_peak_cm','settle2cm_s'});
writetable(Tb, fullfile(csvDir, 'verify_step_pipeline.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'verify_step_pipeline.csv'));

function s = local_settle(tu, xg, A, tol, iS, tStep)
    okm = abs(xg - A) <= tol;
    s = NaN;
    for ii = iS:numel(tu)
        if all(okm(ii:end)); s = tu(ii) - tStep; return; end
    end
end
