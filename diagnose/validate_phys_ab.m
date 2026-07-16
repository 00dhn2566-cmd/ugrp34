%% 모델 타당성 조사 축A (17차): 물성 변수 스윕 x 정규화 ON/OFF A/B
%% 주장 검증: sIa/sIz/sM 물성 스케일이 패키지 질량/크기 변화에서 성능을 보존하는가.
%% ON = parameters.m 계산 게인 그대로 (물성 자동 추종)
%% OFF = 게인을 기준 물성(1kg, 0.14 큐브) 값으로 동결 (정규화 없던 16차 상태 재현)
%% 판정: ON >= OFF 면 정규화 타당. OFF가 우세하면 스케일 법칙 재검토.
%% 지표: 호버RMS(1-3s) / 추종RMS(3-7s) / 오버슈트 / 꼬리RMS(8-14s) / z오차피크 / 자세피크
%% 주의: 추력 바이어스(Bias Load Gain)는 모델이 pkg 질량 연동 - 두 조건 공통 적용됨.

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
tau = min(max((tt-tStep)/0.9,0),1);   % 젠틀무브 0.9s (질량 스윕이라 여유)
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

% --- 구성 목록: {라벨, pkg질량kg, pkg한변m, 정규화ON} ---
cfgs = { ...
    '기준 1.0kg 0.14 ON ', 1.0, 0.14, true;  ...
    '질량 0.5kg      ON ', 0.5, 0.14, true;  ...
    '질량 0.5kg      OFF', 0.5, 0.14, false; ...
    '질량 2.0kg      ON ', 2.0, 0.14, true;  ...
    '질량 2.0kg      OFF', 2.0, 0.14, false; ...
    '크기 0.20m      ON ', 1.0, 0.20, true;  ...
    '크기 0.20m      OFF', 1.0, 0.20, false; ...
};

% 기준 물성 게인 (OFF 조건용 동결값 = 오늘 채택치, 스케일 1)
G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;
G0.kp_yaw = 15;   G0.ki_yaw = 1.5;  G0.kd_yaw = 4;
G0.kp_alt = 0.5;  G0.ki_alt = 0.1;  G0.kd_alt = 0.15;

nC = size(cfgs,1);
rows = nan(nC, 12);
fprintf('===== 타당성 축A: 물성 스윕 x 정규화 A/B (이동 1m 젠틀무브 0.9s) =====\n');
fprintf('%-20s | %7s %7s %7s %7s %7s %7s\n', '구성', '호버cm','추종cm','오버cm','꼬리deg','z피크cm','자세피크');
for c = 1:nC
    m_pkg  = cfgs{c,2};
    edge   = cfgs{c,3};
    normOn = cfgs{c,4};

    % 패키지 물성 주입 (모델 바이어스도 이 변수로 질량 연동됨)
    pkgSize = [1 1 1] * edge;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    % 게인 재계산: parameters.m 로직 재현 (스케일만 갱신)
    m_pkg_now2 = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
    [I_att_c, I_yaw_c, m_tot_c] = qc_phys_local(drone_mass, m_pkg_now2, pkgSize);
    [I_att_r, I_yaw_r, m_tot_r] = qc_phys_local(1.2726, 1.0, [1 1 1]*0.14);
    if normOn
        sIa_c = I_att_c/I_att_r; sIz_c = I_yaw_c/I_yaw_r; sM_c = m_tot_c/m_tot_r;
    else
        sIa_c = 1; sIz_c = 1; sM_c = 1;
    end
    kp_attitude = G0.kp_att * sIa_c;  ki_attitude = G0.ki_att * sIa_c;  kd_attitude = G0.kd_att * sIa_c;
    kp_yaw      = G0.kp_yaw * sIz_c;  ki_yaw      = G0.ki_yaw * sIz_c;  kd_yaw      = G0.kd_yaw * sIz_c;
    kp_altitude = G0.kp_alt * sM_c;   ki_altitude = G0.ki_alt * sM_c;   kd_altitude = G0.kd_alt * sM_c;

    try
        sim(mdl);
    catch e
        fprintf('%-20s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
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
    rows(c,:) = [m_pkg, edge, double(normOn), sIa_c, sIz_c, sM_c, hovp, mv, ov, tailv, zpk, apk];
    fprintf('%-20s | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('(판정: 같은 물성에서 ON >= OFF 면 정규화 타당. z피크는 고도 sM 검증 핵심 지표)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'pkg_mass_kg','pkg_edge_m','norm_on','sIa','sIz','sM', ...
     'hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'validate_phys_ab.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'validate_phys_ab.csv'));

% parameters.m의 qc_phys와 동일식 (로컬 사본 - 스크립트에서 함수 접근 불가라 복제.
% parameters.m 쪽을 바꾸면 여기도 같이 갱신할 것)
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
