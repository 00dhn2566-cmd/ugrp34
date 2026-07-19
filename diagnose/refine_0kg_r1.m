%% 0kg 앵커 좌표하강 1라운드 (18차): 자세 배율 x 고도 배율 격자 - 1차식 법칙의 g0 앵커 확보
%% 배경: 정규화 앵커가 1kg 탑재 시점이라 0kg에서 sM=0.56/sIa=0.55 - 게인 절반 급.
%%       물리적으론 맞는 방향이나 미검증 + 드롭 직후는 어차피 "옛 게인 생 드론" 상태.
%% 판정: OFF(게인 동결)가 우세하면 "정규화는 탑재 시에만" 비대칭 규칙으로 전환.
%% 0kg 처리: pkgDensity를 극소(1e-6)로 - §V diagnose_swing_mass0.m 전례.

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

% 구성: {라벨, pkg질량kg(1e-6=생 드론), 정규화ON}
% {라벨, 자세 배율 sA, 고도 배율 sZ} - 질량은 전부 0kg(1e-6) 고정, yaw는 동결
cfgs = { ...
    'sA0.55 sZ0.56', 0.55, 0.56; ...
    'sA0.55 sZ0.75', 0.55, 0.75; ...
    'sA0.55 sZ1.00', 0.55, 1.00; ...
    'sA0.75 sZ0.56', 0.75, 0.56; ...
    'sA0.75 sZ0.75', 0.75, 0.75; ...
    'sA0.75 sZ1.00', 0.75, 1.00; ...
    'sA1.00 sZ0.56', 1.00, 0.56; ...
    'sA1.00 sZ0.75', 1.00, 0.75; ...
    'sA1.00 sZ1.00', 1.00, 1.00; ...
};

G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;
G0.kp_yaw = 15;   G0.ki_yaw = 1.5;  G0.kd_yaw = 4;
G0.kp_alt = 0.5;  G0.ki_alt = 0.1;  G0.kd_alt = 0.15;

nC = size(cfgs,1);
rows = nan(nC, 10);
fprintf('===== 0kg 앵커 1라운드: 자세 배율 x 고도 배율 (질량 0 고정, 이동 1m 젠틀무브 0.9s) =====\n');
fprintf('%-15s | %7s %7s %7s %7s %7s %7s\n', '구성', '호버cm','추종cm','오버cm','꼬리deg','z피크cm','자세피크');
for c = 1:nC
    sA = cfgs{c,2};                       % 자세 채널 배율
    sZ = cfgs{c,3};                       % 고도 채널 배율
    m_pkg = 1e-6;                         % 생 드론 (0kg)
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    kp_attitude = G0.kp_att * sA;  ki_attitude = G0.ki_att * sA;  kd_attitude = G0.kd_att * sA;
    kp_yaw      = G0.kp_yaw;       ki_yaw      = G0.ki_yaw;       kd_yaw      = G0.kd_yaw;
    kp_altitude = G0.kp_alt * sZ;  ki_altitude = G0.ki_alt * sZ;  kd_altitude = G0.kd_alt * sZ;

    try
        sim(mdl);
    catch e
        fprintf('%-18s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
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
    rows(c,:) = [sA, sZ, 1e-6, hovp, mv, ov, tailv, zpk, apk, 0];
    fprintf('%-15s | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('(판정: 0kg에서 살아나는 배율 조합 = 1차식 g0 앵커의 출발점. 전멸이면 게인 문제가 아님 - 동작점/수치 수술 필요)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'sA_att','sZ_alt','pkg_mass_kg','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg','reserved'});
writetable(Tb, fullfile(csvDir, 'refine_0kg_r1.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_0kg_r1.csv'));

function [I_att, I_yaw, m_tot] = qc_phys_local(m_drone, m_pkg, pkgSz)
    m_ch  = 0.9650346;
    z_ch  = +0.0038181;
    I_ch  = [1.488e-3, 1.538e-3, 2.399e-3];
    m_rot = m_drone - m_ch;
    r_arm = 0.225/sqrt(2);
    z_rot = +0.02;
    z_pkg = -0.012 - pkgSz(3)/2;
    m_tot = m_drone + m_pkg;
    z_cg  = (m_ch*z_ch + m_rot*z_rot + m_pkg*z_pkg) / m_tot;
    Ix = I_ch(1) + m_ch*(z_ch-z_cg)^2 ...
       + m_rot*r_arm^2 + m_rot*(z_rot-z_cg)^2 ...
       + m_pkg/12*(pkgSz(2)^2+pkgSz(3)^2) + m_pkg*(z_pkg-z_cg)^2;
    Iy = I_ch(2) + m_ch*(z_ch-z_cg)^2 ...
       + m_rot*r_arm^2 + m_rot*(z_rot-z_cg)^2 ...
       + m_pkg/12*(pkgSz(1)^2+pkgSz(3)^2) + m_pkg*(z_pkg-z_cg)^2;
    I_att = (Ix + Iy)/2;
    I_yaw = I_ch(3) + m_rot*(2*r_arm^2) + m_pkg/12*(pkgSz(1)^2+pkgSz(2)^2);
end


