%% 자기 질량(기체) 증량 @ 0kg 페이로드 - precision (사용자 지시)
%% 배경: 0kg에서 A(0.750/0.560)가 오버슈트 37cm로 스펙(10cm) 3.7배 초과.
%%       "나누는 값이 작아서"라면 기체 자체를 무겁게 하면 회복돼야 한다.
%% knob: Quadcopter/Body/Body/Flight Computer (Brick Solid, BasedOnType=Mass, 기본 638 g,
%%       동체 중심 배치 = 배터리 증량 등가). drone_mass 변수는 .slx가 안 쓰므로 무효.
%% 게인: 전 구성 0kg 앵커(sA=0.750, sZ=0.560) 고정 - 현행 1차식은 m_pkg만 보므로
%%       기체를 무겁게 해도 출하 제어기는 게인을 안 바꾼다. 그 상태를 그대로 본다.
%% 트림: 고도 피드포워드 Bias Chassis(56.5 rev/s)는 기체 질량을 모른다.
%%       D 구성에서만 56.5 + 44.4*dm 으로 보정해 "트림만의 몫"을 분리한다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);
fprintf('프로파일: %s, kp_position=%s\n', ctrl_profile, mat2str(kp_position));

% --- 라이브러리 링크 해제 (unlink 로컬함수는 파일 끝에 정의) ---
fcBlk = [mdl '/Quadcopter/Body/Body/Flight Computer'];
unlink(mdl, fcBlk);
fprintf('Flight Computer 기본값: BasedOnType=%s, Mass=%s %s\n', ...
        get_param(fcBlk,'BasedOnType'), get_param(fcBlk,'Mass'), get_param(fcBlk,'MassUnits'));

subA = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem'];
bcBlk = [subA '/Bias Chassis'];
unlink(mdl, bcBlk);
BC0 = get_param(bcBlk, 'Bias');
fprintf('Bias Chassis 기본값: %s\n', BC0);

% --- 투하 로직 무력화 ---
dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
unlink(mdl, dropBlocks{1});
for i = 1:numel(dropBlocks); set_param(dropBlocks{i}, 'Value', '-1'); end

% --- 시나리오 (refine_linear_law.m와 동일 하네스) ---
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

% --- 게인: 0kg 앵커 고정 ---
sA = 0.750; sZ = 0.560;
kp_attitude = -85*sA;  ki_attitude = -10*sA;  kd_attitude = -127.5*sA;
kp_yaw = 15;  ki_yaw = 1.5;  kd_yaw = 4;
kp_altitude = 0.5*sZ;  ki_altitude = 0.1*sZ;  kd_altitude = 0.15*sZ;

% 짐은 0으로 고정
m_pkg = 1e-6;
pkgSize = [1 1 1]*0.14;
pkgDensity = m_pkg/(pkgSize(1)*pkgSize(2)*pkgSize(3));

% {라벨, 기체 추가질량 g, 트림보정 여부}
cfgs = { 'A  +0g      ',    0, false; ...
         'B  +250g    ',  250, false; ...
         'C  +500g    ',  500, false; ...
         'D  +500g+트림', 500, true  };

nC = size(cfgs,1);
rows = nan(nC, 8);
fprintf('\n===== 기체 증량 @ 0kg 페이로드, precision, 게인 0kg앵커 고정 =====\n');
fprintf('%-14s | %7s %7s %7s %7s %7s %7s\n', '구성','호버cm','추종cm','오버cm','꼬리deg','z피크cm','자세피크');
for c = 1:nC
    dmg = cfgs{c,2};  dm = dmg/1000;
    set_param(fcBlk, 'BasedOnType', 'Mass');
    set_param(fcBlk, 'MassUnits', 'g');
    set_param(fcBlk, 'Mass', num2str(638 + dmg));
    if cfgs{c,3}
        set_param(bcBlk, 'Bias', num2str(56.5 + 44.4*dm));
    else
        set_param(bcBlk, 'Bias', BC0);
    end
    fprintf('[%s] FC Mass=%s g, Bias=%s\n', strtrim(cfgs{c,1}), get_param(fcBlk,'Mass'), get_param(bcBlk,'Bias'));

    try
        sim(mdl);
    catch e
        fprintf('%-14s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
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
    rows(c,:) = [dmg, double(cfgs{c,3}), hovp, mv, ov, tailv, zpk, apk];
    fprintf('%-14s | %7.2f %7.2f %7.1f %7.2f %7.1f %7.1f\n', cfgs{c,1}, hovp, mv, ov, tailv, zpk, apk);
end
fprintf('\n(A행은 probe_0kg_precision.csv 1행 = 추종 21.35 / 오버 37.12 와 일치해야 회귀 무결)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
Tb = array2table(rows, 'VariableNames', ...
    {'add_mass_g','trim_comp','hover_cm','tracking_rms_cm','overshoot_cm','tail_rms_deg','z_peak_cm','att_peak_deg'});
writetable(Tb, fullfile(csvDir, 'probe_airframe_mass.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'probe_airframe_mass.csv'));

function unlink(mdl, blk)
    % 주의: 블록 자신이 아니라 '부모'부터 올라간다. Simscape 라이브러리 리프 블록
    % (Brick Solid 등)의 링크를 끊으면 컴파일이 거부된다 (diagnose_pid_ident.m 전례).
    p = get_param(blk, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
                set_param(p, 'LinkStatus', 'none');
            end
        catch
        end
        p = get_param(p, 'Parent');
    end
end
