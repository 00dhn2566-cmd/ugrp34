%% precision 프로파일 @ 무질량(0kg) 단독 검증 (사용자 지시)
%% 하네스는 refine_linear_law.m과 동일 (1m 젠틀무브 0.9s, T=14s) - 수치 직접 비교 가능.
%% 3구성 대조:
%%   A 현행 1차식  sA=0.750 sZ=0.560  <- refine_linear_law.csv 0kg 행(28.6cm) 회귀 확인
%%   B 물리 I/sqrt(m) sA=0.730 sZ=0.560  <- 가설: 자세 이득 보존은 I/sqrt(m)
%%   C 고정게인     sA=1.000 sZ=1.000  <- 대조군(1kg 게인 그대로)
%% ctrl_profile 미설정 = precision 기본값 (위치 8/3.2, posErrSat 0.15)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
fprintf('프로파일 확인: ctrl_profile=%s, kp_position=%s, posErrSat=%.4f\n', ...
        ctrl_profile, mat2str(kp_position), posErrSat);
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 투하 로직 무력화 (짐이 중간에 떨어지면 실험 무효)
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

G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;
G0.kp_yaw = 15;   G0.ki_yaw = 1.5;  G0.kd_yaw = 4;
G0.kp_alt = 0.5;  G0.ki_alt = 0.1;  G0.kd_alt = 0.15;

M_PKG = 1e-6;                       % 무질량
cfgs = { 'A 1차식 ', 0.750, 0.560; ...
         'B I/sqrtm', 0.730, 0.560; ...
         'C 고정   ', 1.000, 1.000 };

nC = size(cfgs,1);
rows = nan(nC, 9);
fprintf('\n===== precision @ 0kg (1m 젠틀무브 0.9s, T=14s) =====\n');
fprintf('%-10s | %6s %6s | %7s %7s %7s %7s %7s %7s\n', ...
        '구성','sA','sZ','호버cm','추종cm','오버cm','꼬리deg','z피크cm','자세피크');
for c = 1:nC
    sA = cfgs{c,2};  sZ = cfgs{c,3};
    m_pkg = M_PKG;
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    kp_attitude = G0.kp_att * sA;  ki_attitude = G0.ki_att * sA;  kd_attitude = G0.kd_att * sA;
    kp_yaw      = G0.kp_yaw;       ki_yaw      = G0.ki_yaw;       kd_yaw      = G0.kd_yaw;
    kp_altitude = G0.kp_alt * sZ;  ki_altitude = G0.ki_alt * sZ;  kd_altitude = G0.kd_alt * sZ;

    try
        sim(mdl);
    catch e
        fprintf('%-10s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
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
    rows(c,:) = [m_pkg, sA, sZ, hovp, mv, ov, tailv, zpk, apk];
    fprintf('%-10s | %6.3f %6.3f | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', ...
            cfgs{c,1}, sA, sZ, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('\n(A 행이 refine_linear_law.csv 0kg 행 = 추종 28.61 / 오버 33.52 와 일치해야 회귀 무결)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','sA_att','sZ_alt','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'probe_0kg_precision.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'probe_0kg_precision.csv'));
