%% 0 kg 앵커 재튜닝 (성능 지표 세션, 2026-08-18, 사용자 지시 "0kg 기준 튜닝 다시")
%% 배경: 현행 0 kg 앵커(sA=0.75/sZ=0.56, refine_0kg_r1/r2 "유일 무발산")를 perf_battery로 재실측하니
%%       호버 ±8°/5 Hz 한계사이클 + 1 m 이동 추종 24 cm + 대각 2 m 이동 발산. 자세 관성이 절반(0.93e-2)인데
%%       게인은 25 %만 감쇠 → 실효 루프 이득 1.37배 → 16차 실측 "kp −95~−100 모터 공진 절벽" 진입으로 추정.
%%       기록상 sA를 물리비(0.55)까지 내리면 xy 붕괴 → sA 한 축만으론 해가 없고, 자세 감쇠비(kd/kp)와
%%       위치 게인(kp_pos)을 같이 움직여야 한다는 가설. 이 스크립트는 그 3축 격자.
%% 절차: 구성마다 ① 12 s 호버 (자세 지터/새그/드리프트) ② 1 m 이동 (추종/오버슈트/z피크/자세피크/꼬리) 순차.
%%       ① 지터 RMS > 2° 면 ②는 건너뜀 (호버가 안 되면 이동은 무의미 — 시간 절약).
%% 골격: refine_pos_r1.m 정본. 규칙: save_system 금지 / 투하 영구 off / 1 스크립트 = 1 프로세스.
%% 출력: diagnose/results/tune_0kg_r1.csv (+ 구성별 시계열 tune_0kg_ts_<id>.csv 는 이동 시험 통과 구성만)
%% 사용: matlab -batch "cd(fullfile(pwd,'diagnose')); tune_0kg" > out.txt 2>&1

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

% --- 0 kg 고정 물성 (perf_battery와 동일: 극소 밀도 웰드) ---
pkgSize = [1 1 1] * 0.14;
pkgDensity = 1e-6 / (pkgSize(1)*pkgSize(2)*pkgSize(3));
m_pkg_now = 0;
sZ_mass = 0.56;                                   % 고도 앵커는 물리 예측과 일치(기록) → 이번 격자에서 고정
kp_altitude = 0.5 * sT * sZ_mass; ki_altitude = 0.1 * sT * sZ_mass; kd_altitude = 0.15 * sT * sZ_mass;
wind_speed = 0;

% --- 궤적 2종 (스무더 통과; ZVD는 튜닝 관측을 가리므로 여기선 미적용 = refine 하네스와 동일) ---
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0; dt = 0.01;
s5 = @(tau) (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
Th = 12; tth = (0:round(Th/dt))' * dt; smH = repmat([0 0 1], numel(tth), 1);
Tm = 14; ttm = (0:round(Tm/dt))' * dt;
smM = traj_smoother(ttm, [s5(min(max((ttm-3)/0.9,0),1)), zeros(numel(ttm),1), ones(numel(ttm),1)], VMAX, AMAX, JMAX);
mws = get_param(mdl, 'ModelWorkspace');
setTraj = @(tt, sm) cellfun(@(k,v) mws.assignin(k, v), ...
    {'timespot_spl','spline_data','spline_yaw','waypoints','wayp_path_vis'}, ...
    {tt, sm, zeros(numel(tt),1), [sm(1,:); sm(1,:)+[1 0 0]]', quadcopter_waypoints_to_path_vis([sm(1,:); sm(1,:)+[1 0 0]]')});

% --- 격자: sA(자세 배율) x r(kd/kp 비, 현행 1.5) x kp_pos (현행 8, kd_pos = 0.4*kp 비 유지) ---
sAList = [0.45 0.55 0.65 0.75];
rList  = [1.5 2.0 2.5];
kpList = [8 5];
csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
rows = [];
fprintf('===== 0 kg 재튜닝 r1: sA x kd/kp x kp_pos (sZ=0.56 고정) — %d 구성 =====\n', numel(sAList)*numel(rList)*numel(kpList));
fprintf('%5s %4s %5s | %8s %8s %8s | %8s %8s %8s %8s %8s\n', 'sA','r','kpP', '호버RMS','새그cm','드리프트', '추종cm','오버cm','z피크','자세피크','꼬리');
cid = 0;
for sA = sAList
    for r = rList
        for kpP = kpList
            cid = cid + 1;
            sA_mass = sA;
            kp_attitude = -85 * sT * sA;  ki_attitude = -10 * sT * sA;  kd_attitude = -85 * r * sT * sA;
            kp_position = kpP; kd_position = 0.4 * kpP;  posErrSat = 1.2 / kpP;  posErrSatZ = posErrSat;
            % ① 호버
            setTraj(tth, smH); set_param(mdl, 'StopTime', num2str(Th));
            try
                sim(mdl);
            catch e
                fprintf('%5.2f %4.1f %5.1f | 호버 시뮬 실패: %s\n', sA, r, kpP, e.message);
                rows(end+1,:) = [cid sA r kpP nan(1,8)]; %#ok<AGROW>
                continue;
            end
            tu = (0:0.005:Th)';
            gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
            xg = gi(px); yg = gi(py); zg = gi(pz); pg = rad2deg(gi(real_pitch)); rg = rad2deg(gi(real_roll));
            w = tu >= 2;
            hovRms = sqrt(mean([pg(w)-mean(pg(w)); rg(w)-mean(rg(w))].^2));
            sag = (1 - min(zg(tu < 2))) * 100;
            drift = max(hypot(xg(w)-mean(xg(w)), yg(w)-mean(yg(w)))) * 100;
            mv = NaN; ov = NaN; zpk = NaN; apk = NaN; tailv = NaN;
            if hovRms <= 2.0
                % ② 1 m 이동
                setTraj(ttm, smM); set_param(mdl, 'StopTime', num2str(Tm));
                try
                    sim(mdl);
                    tu2 = (0:0.005:Tm)';
                    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu2, 'linear', 'extrap');
                    xg2 = gi2(px); zg2 = gi2(pz); pg2 = rad2deg(gi2(real_pitch)); rg2 = rad2deg(gi2(real_roll));
                    xr = interp1(ttm, smM(:,1), tu2);
                    seg = @(a,b) (tu2>=a & tu2<b);
                    mv = sqrt(mean((xg2(seg(3,7)) - xr(seg(3,7))).^2)) * 100;
                    ov = max(0, max(xg2) - 1) * 100;
                    zpk = max(abs(zg2(tu2 >= 2) - 1)) * 100;
                    apk = max(max(abs(pg2)), max(abs(rg2)));
                    tailv = sqrt(mean((pg2(seg(8,14)) - mean(pg2(seg(8,14)))).^2));
                    Tb = array2table([tu2, xr, xg2, zg2, rg2, pg2], 'VariableNames', {'t','x_ref','x','z','roll_deg','pitch_deg'});
                    writetable(Tb, fullfile(csvDir, sprintf('tune_0kg_ts_%02d.csv', cid)));
                catch e
                    fprintf('   (이동 시뮬 실패: %s)\n', e.message);
                end
            end
            rows(end+1,:) = [cid sA r kpP hovRms sag drift mv ov zpk apk tailv]; %#ok<AGROW>
            fprintf('%5.2f %4.1f %5.1f | %8.3f %8.1f %8.2f | %8.2f %8.1f %8.1f %8.1f %8.2f%s\n', sA, r, kpP, hovRms, sag, drift, mv, ov, zpk, apk, tailv, ...
                tern(abs(sA-0.75)<1e-9 && abs(r-1.5)<1e-9 && kpP==8, '  <현행 0kg 앵커', ''));
            Tb = array2table(rows, 'VariableNames', {'id','sA','kd_kp','kp_pos','hover_att_rms_deg','sag_cm','drift_cm','track_rms_cm','overshoot_cm','z_peak_cm','att_peak_deg','tail_rms_deg'});
            writetable(Tb, fullfile(csvDir, 'tune_0kg_r1.csv'));   % 진행 중에도 계속 갱신 (중단 대비)
        end
    end
end
fprintf('완료: %.0fs. CSV: %s\n', toc(t0all), fullfile(csvDir, 'tune_0kg_r1.csv'));
close_system(mdl, 0);

function s = tern(c,a,b); if c; s=a; else; s=b; end; end
