%% 짐 크기 강건성: pkgSize 0.10/0.14(기준)/0.20m 큐브 (질량은 1kg 유지 - 밀도 재계산)
%% 크기 변화 = CG 높이/관성/진자 길이 변화. 질량 강건성은 2kg 시험(§V)으로 기합격.
%% 비행: 검증된 성형 1m 이동 (기준 성적: RMS 2.8cm, 자세 13.2도, 모터 81%, 흔들림 1.75Hz).
%% 예상: 저중심 강체 모드 주파수가 L에 따라 이동 (큰 짐 = CG 더 아래 = 주파수 하락).
%% 규칙: 메모리 수술만, save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smKill = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

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

waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', smKill);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element11','w1'; 'In Bus Element10','w2'; 'In Bus Element12','w3'; 'In Bus Element13','w4'};
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

sizes = [0.10, 0.14, 0.20];
fprintf('===== 짐 크기 스윕 (질량 1kg 고정, 성형 1m 이동) =====\n');
summ = {};
for ci = 1:numel(sizes)
    s = sizes(ci);
    pkgSize = [1 1 1] * s;
    pkgDensity = 1 / prod(pkgSize);   % 질량 1kg 유지
    fprintf('\n--- 케이스 %d: %gm 큐브 (밀도 %.1f) ---\n', ci, s, pkgDensity);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        summ(end+1,:) = {s, NaN, NaN, NaN, NaN, NaN, NaN}; %#ok<SAGROW>
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(sig) interp1(sig.time(:), sig.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz);
    pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll)); yw = gi2(real_yaw);
    wCeil = 1025;
    W = [abs(gi2(w1)), abs(gi2(w2)), abs(gi2(w3)), abs(gi2(w4))] / wCeil * 100;
    xr = interp1(tt, smKill(:,1), tu);
    ex = xg - xr;
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    iM = seg(3, 7);
    iT = seg(7, 12);
    pgT = pg(iT) - mean(pg(iT));
    freq = sum(abs(diff(sign(pgT)))>0)/2/5;
    eRms = sqrt(mean(ex(iM).^2))*100;
    ePk = max(abs(ex(iM)))*100;
    attPk = max(max(abs(pg(iM))), max(abs(rg(iM))));
    wPk = max(W(iM,:),[],'all');
    tailRms = rmsf(pg(iT));
    fprintf('  이동: 추종 RMS %.1fcm 피크 %.1fcm | 자세피크 %.1f도 | 모터피크 %.0f%% | z %.2f~%.2f\n', ...
        eRms, ePk, attPk, wPk, min(zg(seg(3,12))), max(zg(seg(3,12))));
    fprintf('  도착 후: pitch 흔들림 RMS %.2f도 @ %.2fHz (기준 0.14m: 4.0도 @ 1.75Hz)\n', tailRms, freq);
    summ(end+1,:) = {s, eRms, ePk, attPk, wPk, tailRms, freq}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%8s | %8s | %8s | %8s | %8s | %10s | %6s\n', '크기[m]','RMS[cm]','피크[cm]','자세[도]','모터%%','흔들림[도]','Hz');
for ci = 1:size(summ,1)
    fprintf('%8.2f | %8.1f | %8.1f | %8.1f | %8.0f | %10.2f | %6.2f\n', summ{ci,:});
end
fprintf('(합격 기준: 추종 피크 <15cm, 모터 <100%%, z 유지. 흔들림 주파수는 L 변화로 이동하는 게 정상)\n');
