%% agile 삼각 법칙 검증 (18차): kp_xy(m) = 24 - 16·|m-1| (1kg 정점, 양끝 precision 수렴)
%% 배경: z분리(C안)로 이동 중 z는 해결됐으나 2kg 정착 후 z 한계사이클 85cm 잔존
%%       (x/y kp 24의 자세 지터 -> 기울기 추력손실 -> 고도 진동. 이동 창 아님).
%%       2kg에서 xy를 precision(8/3.2)으로 되돌리면 그 점은 검증 완료(3.96cm/z 1.9cm).
%% 이번 측정: 1.5kg(xy 16/7.0)과 1.75kg(xy 12/5.1) - 삼각 경사의 미검증 구간만.
%% z축은 C안 그대로 8/3.2 고정 (이동 중 z 해결 실증분 유지).
%% 합격: 1.5/1.75kg 추종 수 cm + z꼬리 한자릿수 -> 삼각 법칙 채택.

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

% --- z축 posErrSat 분리 (메모리 수술): PosErr Sat Z만 posErrSatZ 참조 ---
satZ = [mdl '/Maneuver Controller/Position Control/PosErr Sat Z'];
p = get_param(satZ, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
set_param(satZ, 'UpperLimit', 'posErrSatZ', 'LowerLimit', '-posErrSatZ');
fprintf('PosErr Sat Z -> posErrSatZ 분리 완료 (메모리)\n');

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

% 구성: {라벨, 질량kg} - 삼각 법칙 미검증 경사 구간 + 2kg 확인
cfgs = { ...
    'T 1.5kg ', 1.5;  ...
    'T 1.75kg', 1.75; ...
    'T 2.0kg ', 2.0;  ...
};

nC = size(cfgs,1);
rows = nan(nC, 14);
fprintf('===== agile 삼각 법칙: 위치 xy = 24-16·|m-1| / z=8 고정 + 표준 자세/고도 1차식 =====\n');
fprintf('(이동 1m 젠틀무브 0.9s. z이동/z꼬리 = 이동 구간(2.8~7s)/정착 후 z 최대이탈)\n');
for c = 1:nC
    m_pkg = cfgs{c,2};
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    mA = min(m_pkg, 2);                 % 자세/고도는 2kg 캡 (채택 법칙)
    tri = max(0, 1 - abs(m_pkg - 1));   % 삼각 가중 (1kg=1, 0/2kg=0)
    sA = 0.75 + 0.25 * mA;
    sZ = 0.56 + 0.44 * mA;
    kpxy = 8   + 16  * tri;
    kdxy = 3.2 + 7.6 * tri;
    kp_position = [kpxy kpxy 8];        % 벡터 게인 - PID 블록 축별 적용
    kd_position = [kdxy kdxy 3.2];
    posErrSat   = 1.2 / kpxy;           % x/y 클램프 (스칼라 유지)
    posErrSatZ  = 1.2 / 8;              % z 클램프 (분리 변수)
    kp_attitude = -85 * sA;   ki_attitude = -10 * sA;  kd_attitude = -127.5 * sA;
    kp_altitude = 0.5 * sZ;   ki_altitude = 0.1 * sZ;  kd_altitude = 0.15 * sZ;
    fprintf('[%s] pos xy %.1f/%.2f z 8/3.2 | sat %.3f/%.3f | sA %.3f sZ %.3f\n', ...
        strtrim(cfgs{c,1}), kpxy, kdxy, posErrSat, posErrSatZ, sA, sZ);

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
    zmove = max(abs(zg(seg(2.8,7)) - 1))*100;   % 이동 시작~끝 구간 z 이탈 (사용자 관찰 지점)
    ztail = max(abs(zg(seg(7,14)) - 1))*100;    % 정착 후 z 이탈
    apk   = max(max(abs(pg)), max(abs(rg)));
    rows(c,:) = [m_pkg, kpxy, kdxy, 8, 3.2, sA, sZ, hovp, mv, ov, tailv, zmove, ztail, apk];
    fprintf('%-9s | 호버 %5.2f 추종 %7.2f 오버 %6.1f 꼬리 %5.2f z이동 %6.1f z꼬리 %5.1f 자세피크 %5.1f\n', ...
        cfgs{c,1}, hovp, mv, ov, tailv, zmove, ztail, apk);
end
fprintf('(합격: B안 잔존 z피크(1.5kg 42/2kg 85cm)가 한자릿수로 + 수평 성적 유지(1kg 1.32cm급))\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','kp_xy','kd_xy','kp_z','kd_z','sA','sZ','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_move_cm','z_tail_cm','att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'refine_agile_tri.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_agile_tri.csv'));
