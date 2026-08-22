%% 2 kg 판별점 — 자세 게인 스케일 법칙 3종 대조 (DYNAMICS.md §11 미확정 #3)
%%
%% 배경: 0 kg은 혼돈 구간(게인 3e-7 섭동에 추종 34% 변동)이라 판별이 불가능했다.
%%       2 kg은 안정 구간이면서 세 법칙이 최대로 갈리는 지점이다.
%%
%% 후보 (자세 채널 배율 sA, m_pkg = 2 kg 기준):
%%   A 현행 1차식     sA = 0.75 + 0.25*m           = 1.250
%%   B 물리 I/sqrt(m) sA = (I/sqrt(m)) 비          = 1.078
%%   C 관성비 I/I_ref sA = I/I_ref                 = 1.293
%%
%% 고도 배율 sZ는 1.44로 고정 (1차식 = 총질량비와 2kg에서 정확히 일치하므로 논쟁 없음).
%% 하네스는 refine_linear_law.m 과 동일 (1 m 젠틀무브 0.9 s, T = 14 s).
%% 규칙: save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);
fprintf('프로파일=%s  kp_position=%s\n', ctrl_profile, mat2str(kp_position));

% 투하 로직 무력화
dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(dropBlocks{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved', 'inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(dropBlocks)
    set_param(dropBlocks{i}, 'Value', '-1');
end

% --- 시나리오 ---
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 14; tStep = 3; Amp = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt - tStep)/0.9, 0), 1);
xk = Amp * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
waypoints = [0 0 1; Amp 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', sm);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element', 'px'; 'In Bus Element2', 'pz'; ...
          'In Bus Element4', 'real_roll'; 'In Bus Element3', 'real_pitch'};
for i = 1:size(sigMap, 1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw)
        delete_block(oldTw{1});
    end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, ...
              'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;
G0.kp_yaw = 15;   G0.ki_yaw = 1.5;  G0.kd_yaw = 4;
G0.kp_alt = 0.5;  G0.ki_alt = 0.1;  G0.kd_alt = 0.15;

M_PKG = 2.0;
SZ    = 1.44;      % 세 후보 공통 (1차식 = 총질량비)

% 물성으로 후보 B, C 계산
[I2, ~, m2] = qc_phys_local(drone_mass, M_PKG, [1 1 1]*0.14);
[I1, ~, m1] = qc_phys_local(drone_mass, 1.0,   [1 1 1]*0.14);
sA_B = (I2/sqrt(m2)) / (I1/sqrt(m1));
sA_C = I2 / I1;
fprintf('\n물성: I(2kg)=%.5e  I(1kg)=%.5e  m(2kg)=%.4f  m(1kg)=%.4f\n', I2, I1, m2, m1);

cfgs = { 'A 1차식    ', 0.75 + 0.25*M_PKG; ...
         'B I/sqrt(m)', sA_B; ...
         'C 관성비   ', sA_C };

nC = size(cfgs, 1);
rows = nan(nC, 8);
fprintf('\n===== 2kg 판별점 (precision, sZ=%.2f 고정) =====\n', SZ);
fprintf('%-12s | %6s | %7s %7s %7s %7s %7s %7s\n', ...
        '구성', 'sA', '호버cm', '추종cm', '오버cm', '꼬리deg', 'z피크cm', '자세피크');
for c = 1:nC
    sA = cfgs{c,2};
    m_pkg = M_PKG;
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));

    kp_attitude = G0.kp_att * sA;  ki_attitude = G0.ki_att * sA;  kd_attitude = G0.kd_att * sA;
    kp_yaw      = G0.kp_yaw;       ki_yaw      = G0.ki_yaw;       kd_yaw      = G0.kd_yaw;
    kp_altitude = G0.kp_alt * SZ;  ki_altitude = G0.ki_alt * SZ;  kd_altitude = G0.kd_alt * SZ;

    fprintf('[%s] sA=%.4f\n', strtrim(cfgs{c,1}), sA);
    try
        sim(mdl);
    catch e
        fprintf('%-12s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz);
    pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));
    xr = interp1(tt, sm(:,1), tu);
    seg  = @(t1, t2) (tu >= t1 & tu < t2);
    rmsf = @(v) sqrt(mean((v - mean(v)).^2));
    hovp  = rmsf(xg(seg(1,3))) * 100;
    mv    = sqrt(mean((xg(seg(3,7)) - xr(seg(3,7))).^2)) * 100;
    ov    = max(0, max(xg) - Amp) * 100;
    tailv = rmsf(pg(seg(8,14)));
    zpk   = max(abs(zg(seg(1,14)) - 1)) * 100;
    apk   = max(max(abs(pg)), max(abs(rg)));
    rows(c,:) = [M_PKG, sA, hovp, mv, ov, tailv, zpk, apk];
    fprintf('%-12s | %6.3f | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', ...
            cfgs{c,1}, sA, hovp, mv, ov, tailv, zpk, apk);
end

fprintf('\n(참고: 18차 refine_linear_law 의 2kg 행 = 추종 3.96 / 오버 7.64 / z 1.86)\n');
fprintf('(판정: 추종·오버슈트·z피크를 함께 볼 것. 한 지표만으로 결론 내지 말 것)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
     {'pkg_kg', 'sA', 'hover_cm', 'tracking_rms_cm', 'overshoot_cm', ...
      'tail_rms_deg', 'z_peak_cm', 'att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'probe_2kg_law.csv'));
fprintf('CSV 저장: probe_2kg_law.csv\n');

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
