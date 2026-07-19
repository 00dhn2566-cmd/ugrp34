%% agile 전질량 1차식 검증 (18차, 사용자 설계 "위치 PID도 똑같이 1차식"):
%% 위치: kp(m)=8+16·m, kd(m)=3.2+7.6·m (0kg=precision 8/3.2, 1kg=agile 24/10.8)
%%       posErrSat=1.2/kp 자동 연동. 자세/고도: 채택된 표준 1차식(0.75/0.56 앵커).
%% 근거: agile 고정 위치 게인으로는 0kg 격자 전멸(refine_0kg_agile_r1) - 범인은 위치 kp.
%%       0kg 끝점(8/3.2 + 0.75/0.56)은 refine_0kg_r1/r2에서 생존 실측된 점.
%% 변형 2종: A = 전 구간 외삽(2kg 캡, kp(2)=40 - 절벽 33 위험) / B = 위치만 1kg 캡.
%% 합격: 전 질량 무발산 + 0.5kg/2kg 정상화(agile 관문 실패점) + 1kg = 24/10.8 회귀.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
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

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 14; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.9,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', sm);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

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

% 구성: {라벨, 질량kg, 위치캡(1=1kg 캡 B안 / 2=외삽 A안)}
cfgs = { ...
    'A 0kg   ', 1e-6, 2; ...
    'A 0.25kg', 0.25, 2; ...
    'A 0.5kg ', 0.5,  2; ...
    'A 1.0kg ', 1.0,  2; ...
    'A 1.5kg ', 1.5,  2; ...
    'A 2.0kg ', 2.0,  2; ...
    'B 1.5kg ', 1.5,  1; ...
    'B 2.0kg ', 2.0,  1; ...
};

nC = size(cfgs,1);
rows = nan(nC, 12);
fprintf('===== agile 전질량 1차식: 위치 kp=8+16m kd=3.2+7.6m + 표준 자세/고도 1차식 =====\n');
fprintf('(A=위치 외삽 2kg캡, B=위치 1kg캡. 이동 1m 젠틀무브 0.9s)\n');
for c = 1:nC
    m_pkg = cfgs{c,2};
    posCap = cfgs{c,3};
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    mA = min(m_pkg, 2);                 % 자세/고도는 항상 2kg 캡 (채택 법칙)
    mP = min(m_pkg, posCap);            % 위치는 변형별 캡
    sA = 0.75 + 0.25 * mA;
    sZ = 0.56 + 0.44 * mA;
    kp_position = 8   + 16  * mP;
    kd_position = 3.2 + 7.6 * mP;
    posErrSat   = 1.2 / kp_position;
    kp_attitude = -85 * sA;   ki_attitude = -10 * sA;  kd_attitude = -127.5 * sA;
    kp_altitude = 0.5 * sZ;   ki_altitude = 0.1 * sZ;  kd_altitude = 0.15 * sZ;
    fprintf('[%s] pos %.1f/%.2f sat %.3f | sA %.3f sZ %.3f\n', ...
        strtrim(cfgs{c,1}), kp_position, kd_position, posErrSat, sA, sZ);

    try
        sim(mdl);
    catch e
        fprintf('%-9s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz); pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));
    xr = interp1(tt, sm(:,1), tu);
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    hovp  = rmsf(xg(seg(1,3)))*100;
    mv    = sqrt(mean((xg(seg(3,7))-xr(seg(3,7))).^2))*100;
    ov    = max(0, max(xg) - A)*100;
    tailv = rmsf(pg(seg(8,14)));
    zpk   = max(abs(zg(seg(1,14)) - 1))*100;
    apk   = max(max(abs(pg)), max(abs(rg)));
    rows(c,:) = [m_pkg, posCap, kp_position, kd_position, sA, sZ, hovp, mv, ov, tailv, zpk, apk];
    fprintf('%-9s | 호버 %5.2f 추종 %7.2f 오버 %6.1f 꼬리 %5.2f z피크 %6.1f 자세피크 %5.1f\n', ...
        cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('(합격: 전 질량 무발산 + 0.5/2kg 정상화 + 1kg 회귀 1.32cm급. 2kg에서 A 발산 시 B 채택)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','pos_cap','kp_pos','kd_pos','sA','sZ','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'refine_agile_law.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_agile_law.csv'));
