%% P1: 프로펠러 회전속도 단위 실측  +  P2: 믹서 차동 부호표 확정
%% 배경: DYNAMICS.md §11 미확정 #1(믹서 부호), #2(회전속도 단위).
%%
%% P1 판정 - Prop.w 채널(포트 라벨 'rpm')이 실제로 무엇인가:
%%   가설 A(물리 일치): 프롭 634 rad/s -> 로그값 6057
%%   가설 B(단위 흡수): 프롭  66 rad/s -> 로그값  634
%%   호버 추력이 5.57 N/모터로 고정이므로 로그값이 어느 쪽인지로 갈린다.
%%
%% P2 판정 - roll/pitch/yaw 명령에 대해 어느 모터가 빨라지는가:
%%   +x 이동(pitch), +y 이동(roll), yaw 스텝 3편의 모터속도 편차 부호.
%% 규칙: save_system 금지 (in-memory 편집만).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);
m_pkg_here = pkgSize(1)^3 * pkgDensity;
fprintf('프로파일=%s  짐=%.3f kg  Kthrust=%.4f  Kdrag=%.4f\n', ...
        ctrl_profile, m_pkg_here, propeller.Kthrust, propeller.Kdrag);

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

% --- 로깅 배선 (신호명은 .slx InterfaceData에서 확인한 매핑) ---
scope = [mdl '/Scope'];
sigMap = { 'In Bus Element11', 'w1'; 'In Bus Element10', 'w2'; ...
           'In Bus Element12', 'w3'; 'In Bus Element13', 'w4'; ...
           'In Bus Element6',  'T1'; 'In Bus Element7',  'T2'; ...
           'In Bus Element8',  'T3'; 'In Bus Element9',  'T4'; ...
           'In Bus Element4',  'rl'; 'In Bus Element3',  'pt'; ...
           'In Bus Element5',  'yw' };
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

% --- 시나리오 ---
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt - tStep)/0.9, 0), 1);
prof = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

runNames = { 'X이동(pitch)', 'Y이동(roll)', 'YAW스텝' };
runKinds = { 'x', 'y', 'yaw' };
mws = get_param(mdl, 'ModelWorkspace');
set_param(mdl, 'StopTime', num2str(T));

nRun = numel(runKinds);
W = cell(nRun, 1);   % 각 런의 [w1 w2 w3 w4] 시계열
A = cell(nRun, 1);   % 각 런의 [roll pitch yaw]
TT = cell(nRun, 1);  % 각 런의 [T1..T4]
TI = cell(nRun, 1);  % 시간축

for c = 1:nRun
    kind = runKinds{c};
    xk = zeros(N,1); yk = zeros(N,1); yawk = zeros(N,1);
    if strcmp(kind, 'x')
        xk = 1.0 * prof;   wp = [0 0 1; 1 0 1]';
    elseif strcmp(kind, 'y')
        yk = 1.0 * prof;   wp = [0 0 1; 0 1 1]';
    else
        yawk = (pi/4) * prof;   wp = [0 0 1; 0 0 1.001]';
    end
    sm = traj_smoother(tt, [xk, yk, ones(N,1)], VMAX, AMAX, JMAX);
    mws.assignin('waypoints', wp);
    mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(wp));
    mws.assignin('timespot_spl', tt);
    mws.assignin('spline_data', sm);
    mws.assignin('spline_yaw', yawk);

    fprintf('\n>>> RUN %d: %s\n', c, runNames{c});
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    TI{c} = tu;
    W{c}  = [gi(w1), gi(w2), gi(w3), gi(w4)];
    TT{c} = [gi(T1), gi(T2), gi(T3), gi(T4)];
    A{c}  = [gi(rl), gi(pt), gi(yw)];
end

%% ================= P1: 단위 판정 =================
fprintf('\n\n========== P1: 프로펠러 회전속도 단위 ==========\n');
tu = TI{1};
hov = (tu >= 1 & tu <= 2.8);
wbar = mean(W{1}(hov, :), 1);
Tbar = mean(TT{1}(hov, :), 1);
fprintf('호버 Prop.w  (로그값) : %10.2f %10.2f %10.2f %10.2f\n', wbar);
fprintf('호버 Prop.thrust [N]  : %10.4f %10.4f %10.4f %10.4f  (|합| %.3f N)\n', ...
        Tbar, sum(abs(Tbar)));
m_tot = drone_mass + m_pkg_here;
fprintf('필요 추력 = m*g/4      = %.4f N   (m_tot = %.4f kg)\n', m_tot*9.80665/4, m_tot);

wm = mean(abs(wbar));
fprintf('\n판정:  |w| 평균 = %.1f\n', wm);
if wm > 3000
    fprintf('  -> 가설 A: 로그값이 rpm이고 프롭이 실제 %.0f rpm (= %.1f rad/s)\n', wm, wm*pi/30);
    fprintf('     물리 현실 쌍(Ct=0.1072)과 정합. 문서 5.3절 서술 수정 필요.\n');
else
    fprintf('  -> 가설 B: 프롭이 %.0f (로그단위) = %.1f rad/s 로 물리보다 느리게 돎\n', wm, wm*pi/30);
    fprintf('     kT=9.79가 속도 단위 불일치를 흡수 중. 문서 5.3절 서술이 맞음.\n');
end
n_a = wm / 60;
n_b = (wm * pi/30) / (2*pi);
fprintf('  검산: 로그값을 rpm으로 보면 n=%.3f rev/s -> kT=9.79 대입 T=%.3f N/모터\n', ...
        n_a, propeller.Kthrust * air_rho * n_a^2 * propeller.diameter^4);
fprintf('        로그값을 rad/s로 보면 n=%.3f rev/s -> Ct=0.1072 대입 T=%.3f N/모터\n', ...
        n_b, 0.1072 * air_rho * n_b^2 * propeller.diameter^4);

%% ================= P2: 믹서 부호표 =================
fprintf('\n\n========== P2: 믹서 차동 부호표 ==========\n');
fprintf('기동 구간(3.3~4.5s) 모터속도 편차, 호버(1~2.8s) 대비\n\n');
fprintf('%-14s | %9s %9s %9s %9s | %s\n', '런', 'M1', 'M2', 'M3', 'M4', '자세 변화 [deg]');
rows = nan(nRun, 4);
att  = nan(nRun, 3);
for c = 1:nRun
    if isempty(W{c}); continue; end
    tu = TI{c};
    h = (tu >= 1 & tu <= 2.8);
    a = (tu >= 3.3 & tu <= 4.5);
    dv = mean(W{c}(a,:), 1) - mean(W{c}(h,:), 1);
    da = rad2deg(mean(A{c}(a,:), 1) - mean(A{c}(h,:), 1));
    rows(c,:) = dv;
    att(c,:)  = da;
    fprintf('%-14s | %+9.3f %+9.3f %+9.3f %+9.3f | roll %+.2f  pitch %+.2f  yaw %+.2f\n', ...
            runNames{c}, dv, da);
end

fprintf('\n부호 요약 (믹서 표):\n');
fprintf('%-14s | %4s %4s %4s %4s\n', '축', 'M1', 'M2', 'M3', 'M4');
for c = 1:nRun
    if all(isnan(rows(c,:))); continue; end
    ss = repmat(' ', 1, 4);
    for k = 1:4
        if rows(c,k) >= 0
            ss(k) = '+';
        else
            ss(k) = '-';
        end
    end
    fprintf('%-14s | %4c %4c %4c %4c\n', runNames{c}, ss(1), ss(2), ss(3), ss(4));
end

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table([(1:nRun)', rows, att], 'VariableNames', ...
     {'run', 'dw1', 'dw2', 'dw3', 'dw4', 'droll_deg', 'dpitch_deg', 'dyaw_deg'});
writetable(Tb, fullfile(csvDir, 'probe_mixer_signs.csv'));
Tb2 = array2table([wbar(:), Tbar(:)], 'VariableNames', {'hover_w', 'hover_thrust_N'});
writetable(Tb2, fullfile(csvDir, 'probe_prop_hover.csv'));
fprintf('\nCSV 저장: probe_mixer_signs.csv / probe_prop_hover.csv\n');
