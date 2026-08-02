%% 전체 파이프라인 램프응답 (18차, 사용자 요청): 등속 기준 추종 성능
%% 램프(일정 속도 v)는 엔벌로프(v2.0) 안이라 성형기 개입은 가감속 모서리뿐 -
%% 등속 구간의 추종 지연(following error)이 제어기 고유 성능. 이론상 이 루프는
%% 기울기->가속의 이중적분 플랜트라 등속 지연 = 0이어야 하고, 실측 잔차가 곧 성적.
%% 지표: 등속 구간 지연 평균/RMS, 램프 중 자세 RMS, 정지 후 오버슈트/정착, 잔류 진동.
%% 구성: {precision, agile} x {0.5, 1.5 m/s} - 1kg, 이동 거리 3m (등속 구간 확보).

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

% 구성: {라벨, 프로파일, 램프 속도 v[m/s], 거리 D[m]} - 등속 구간 >= 2.5s 확보
% (1차에서 1.5m/s x 3m은 등속 2s뿐이라 측정창 공집합 - 거리 확장 재실행)
cfgs = { ...
    'prec 1.5m/s ', 'precision', 1.5, 6.0; ...
    'prec 2.0m/s ', 'precision', 2.0, 8.0; ...
    'agile 1.5m/s', 'agile',     1.5, 6.0; ...
    'agile 2.0m/s', 'agile',     2.0, 8.0; ...
};

nC = size(cfgs,1);
rows = nan(nC, 12);
fprintf('===== 전체 파이프라인 램프응답 (성형기 포함, 1kg, 3m): 등속 추종 지연 =====\n');
for c = 1:nC
    prof = cfgs{c,2};
    vR = cfgs{c,3};
    D  = cfgs{c,4};

    clearvars ctrl_profile
    ctrl_profile = prof;
    quadcopter_package_parameters;    % 프로파일별 게인 산출 (1kg 기본 물성)

    % 램프 명령: t=tStep부터 등속 vR로 D까지, 이후 유지 -> 성형기(모서리 성형) -> ZVD
    xk = min(max(tt - tStep, 0) * vR, D);
    sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
    sm = traj_zv(tt, sm, 1.80, 'zvd');   % 실사용 사슬 완성
    wp = [0 0 1; D 0 1]';
    mws.assignin('spline_data', sm);
    mws.assignin('waypoints', wp);
    mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(wp));

    tRef99 = NaN; %#ok<NASGU> % (램프에선 미사용)
    tEndRamp = tStep + D / vR;           % 명목 등속 종료 시각

    try
        sim(mdl);
    catch e
        fprintf('%-10s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.002:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz);
    pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));

    xr_ref = interp1(tt, sm(:,1), tu);
    ferr = xr_ref - xg;                  % following error (기준 - 실측)
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    % 등속 구간: 가감속 모서리/ZVD 지연 제외 (시작+1.5s ~ 종료-1.0s)
    segV = tu >= tStep+1.5 & tu <= tEndRamp-1.0;
    lagMean = mean(ferr(segV)) * 100;    % cm (양수 = 뒤처짐)
    lagRms  = rmsf(ferr(segV)) * 100;
    % 정지 후: 오버슈트/정착/말기
    ovStop = max(0, max(xg) - D) * 100;
    iS = find(tu > tEndRamp, 1);
    settleAb = local_settle(tu, xg, D, 0.02, iS, tEndRamp);   % 정지 후 ±2cm
    sse = mean(xg(tu >= T-2)) - D;
    zpk = max(abs(zg(tu > 1) - 1)) * 100;
    % 자세: 램프 중 피크/RMS + 말기 잔류 RMS
    segR = tu >= tStep & tu <= tEndRamp;
    segC = tu >= T-5;
    attPk   = max(max(abs(pg(segR))), max(abs(rg(segR))));
    attRun  = rmsf(pg(segR));
    attTail = rmsf(pg(segC));
    rows(c,:) = [vR, double(strcmp(prof,'agile')), lagMean, lagRms, ovStop, settleAb, sse*1000, zpk, attPk, attRun, attTail, 0];
    fprintf('%-12s | 등속지연 평균 %+6.2fcm RMS %5.2fcm | 정지오버 %5.1fcm | 정착±2cm %5.2fs | SSE %+6.1fmm | z피크 %4.1fcm\n', ...
        cfgs{c,1}, lagMean, lagRms, ovStop, settleAb, sse*1000, zpk);
    fprintf('%-12s | 자세: 램프피크 %5.2f도 | 램프 RMS %5.3f도 | 말기 RMS %5.3f도\n', '', attPk, attRun, attTail);
    Ts = table(tu, xr_ref, xg, pg, 'VariableNames', {'t','x_ref','x_meas','pitch_deg'});
    writetable(Ts, fullfile(modelDir, 'diagnose', 'results', ...
        sprintf('ramp_ts_%s_%gmps.csv', prof, vR)));
end
fprintf('(등속지연 = 등속 구간 following error. 이론상 0(이중적분 플랜트) - 잔차가 곧 성적)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'ramp_v_mps','is_agile','lag_mean_cm','lag_rms_cm','stop_over_cm','settle2cm_s','sse_mm','z_peak_cm','att_pk_deg','att_run_rms_deg','att_tail_rms_deg','reserved'});
writetable(Tb, fullfile(csvDir, 'verify_ramp_pipeline2.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'verify_ramp_pipeline.csv'));

function s = local_settle(tu, xg, A, tol, iS, tStep)
    okm = abs(xg - A) <= tol;
    s = NaN;
    for ii = iS:numel(tu)
        if all(okm(ii:end)); s = tu(ii) - tStep; return; end
    end
end
