%% 질량 1차식 게인 법칙 검증 (18차, 사용자 설계): 배율(m) = s0 + (1-s0)·m 로 질량 6점 시험
%% 실행 전: 아래 S0_ATT/S0_ALT를 0kg 앵커 하강(refine_0kg_r1/r2) 승자로 채울 것.
%% 합격 기준: 전 질량에서 발산 없음 + 0.5kg 내삽/1.5~2kg 외삽 지표가 고정게인 대비 비열등.

S0_ATT = NaN;   % <- 0kg 앵커 자세 배율 (예: 0.75)
S0_ALT = NaN;   % <- 0kg 앵커 고도 배율 (예: 0.56)
if isnan(S0_ATT) || isnan(S0_ALT)
    error('S0_ATT/S0_ALT 미기입 - 0kg 앵커 확보 후 채울 것');
end
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
% {라벨, 질량 kg} - 배율은 1차식 배율(m) = s0 + (1-s0)·m 으로 자동 계산 (1kg에서 정확히 1)
cfgs = { ...
    '0kg   ', 1e-6; ...
    '0.25kg', 0.25; ...
    '0.5kg ', 0.5;  ...
    '1.0kg ', 1.0;  ...
    '1.5kg ', 1.5;  ...
    '2.0kg ', 2.0;  ...
};

G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;
G0.kp_yaw = 15;   G0.ki_yaw = 1.5;  G0.kd_yaw = 4;
G0.kp_alt = 0.5;  G0.ki_alt = 0.1;  G0.kd_alt = 0.15;

nC = size(cfgs,1);
rows = nan(nC, 10);
fprintf('===== 질량 1차식 법칙 6점 검증: 배율(m)=s0+(1-s0)m, s0_att=%.2f s0_alt=%.2f (이동 1m 젠틀무브 0.9s) =====\n', S0_ATT, S0_ALT);
fprintf('%-15s | %7s %7s %7s %7s %7s %7s\n', '구성', '호버cm','추종cm','오버cm','꼬리deg','z피크cm','자세피크');
for c = 1:nC
    m_pkg = cfgs{c,2};                    % 질량 스윕
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    % 1차식 법칙: 배율(m) = s0 + (1-s0)·m  (m=1에서 1 = 현행 채택 게인 보존)
    sA = S0_ATT + (1 - S0_ATT) * min(m_pkg, 2);   % 외삽은 2kg까지만 (안전 상한)
    sZ = S0_ALT + (1 - S0_ALT) * min(m_pkg, 2);

    kp_attitude = G0.kp_att * sA;  ki_attitude = G0.ki_att * sA;  kd_attitude = G0.kd_att * sA;
    kp_yaw      = G0.kp_yaw;       ki_yaw      = G0.ki_yaw;       kd_yaw      = G0.kd_yaw;
    kp_altitude = G0.kp_alt * sZ;  ki_altitude = G0.ki_alt * sZ;  kd_altitude = G0.kd_alt * sZ;
    fprintf('[m=%.2f] sA=%.3f sZ=%.3f\n', m_pkg, sA, sZ);

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
    rows(c,:) = [m_pkg, sA, sZ, hovp, mv, ov, tailv, zpk, apk, 0];
    fprintf('%-15s | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('(합격: 전 질량 무발산 + 내삽/외삽 비열등. 1kg 행은 현행 채택치와 동일해야 함 - 회귀 감지)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','sA_att','sZ_alt','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg','reserved'});
writetable(Tb, fullfile(csvDir, 'refine_linear_law.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_linear_law.csv'));

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


