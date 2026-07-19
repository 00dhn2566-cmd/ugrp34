%% 질량 탐침 (18차, 1차식 게인 법칙 선행): 0kg 붕괴 지점 국소화 - 0.5(대조)~0.03kg x ON/OFF
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
cfgs = { ...
    '대조 0.5kg  ON ', 0.5,  true;  ...
    '탐침 0.3kg  ON ', 0.3,  true;  ...
    '탐침 0.3kg  OFF', 0.3,  false; ...
    '탐침 0.1kg  ON ', 0.1,  true;  ...
    '탐침 0.1kg  OFF', 0.1,  false; ...
    '탐침 0.03kg ON ', 0.03, true;  ...
    '탐침 0.03kg OFF', 0.03, false; ...
};

G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;
G0.kp_yaw = 15;   G0.ki_yaw = 1.5;  G0.kd_yaw = 4;
G0.kp_alt = 0.5;  G0.ki_alt = 0.1;  G0.kd_alt = 0.15;

nC = size(cfgs,1);
rows = nan(nC, 11);
fprintf('===== 질량 탐침: 0.5~0.03kg x ON/OFF (이동 1m 젠틀무브 0.9s) =====\n');
fprintf('%-18s | %7s %7s %7s %7s %7s %7s\n', '구성', '호버cm','추종cm','오버cm','꼬리deg','z피크cm','자세피크');
for c = 1:nC
    m_pkg  = cfgs{c,2};
    normOn = cfgs{c,3};
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    [I_att_c, I_yaw_c, m_tot_c] = qc_phys_local(drone_mass, m_pkg, pkgSize);
    [I_att_r, I_yaw_r, m_tot_r] = qc_phys_local(1.2726, 1.0, [1 1 1]*0.14);
    if normOn
        sIa_c = I_att_c/I_att_r; sIz_c = I_yaw_c/I_yaw_r; sM_c = m_tot_c/m_tot_r;
    else
        sIa_c = 1; sIz_c = 1; sM_c = 1;
    end
    kp_attitude = G0.kp_att * sIa_c;  ki_attitude = G0.ki_att * sIa_c;  kd_attitude = G0.kd_att * sIa_c;
    kp_yaw      = G0.kp_yaw * sIz_c;  ki_yaw      = G0.ki_yaw * sIz_c;  kd_yaw      = G0.kd_yaw * sIz_c;
    kp_altitude = G0.kp_alt * sM_c;   ki_altitude = G0.ki_alt * sM_c;   kd_altitude = G0.kd_alt * sM_c;
    fprintf('[%s] sIa=%.3f sIz=%.3f sM=%.3f\n', strtrim(cfgs{c,1}), sIa_c, sIz_c, sM_c);

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
    rows(c,:) = [m_pkg, double(normOn), sIa_c, sIz_c, sM_c, hovp, mv, ov, tailv, zpk, apk];
    fprintf('%-18s | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('(판정: 어느 질량부터 무너지는지 + ON/OFF 어느 쪽이 버티는지 - 0kg 앵커 튜닝 시작점 결정)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','norm_on','sIa','sIz','sM','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'refine_mass_probe.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_mass_probe.csv'));

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

