%% 골든 트레이스 — Simscape 쪽 내보내기 (DYNAMICS.md §9 / §11.1)
%%
%% 목적: 폐루프로 정상 비행시키면서 **모터 입력 n(t)와 궤적을 함께** 기록한다.
%%       같은 n(t)를 독립 적분기(plant_sim.py)에 먹여 궤적이 일치하는지 본다.
%%       제어기가 무엇을 하든 무관해진다 - 액추에이터 입력만 같으면 된다.
%%
%% 내보내는 것: t, w1..w4 [rpm], px,py,pz, vx,vy,vz, roll,pitch,yaw
%%   (Prop.w 가 rpm 임은 probe_prop_and_mixer.m 에서 실측 확정)
%%
%% 대조 시작점은 t = 2.5 s (정착 호버). 이유:
%%   - 지면 접촉·이륙 과도를 피한다 (plant_sim 에는 접촉 모델이 없다)
%%   - 그 시점 각속도가 거의 0이라 초기조건이 깨끗하다 (각속도는 버스에 없음)
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
m_pkg_here = pkgSize(1)^3 * pkgDensity;
fprintf('프로파일=%s  짐=%.3f kg  총질량=%.4f kg\n', ...
        ctrl_profile, m_pkg_here, drone_mass + m_pkg_here);

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

% --- 로깅 배선 ---
scope = [mdl '/Scope'];
sigMap = { 'In Bus Element11', 'w1'; 'In Bus Element10', 'w2'; ...
           'In Bus Element12', 'w3'; 'In Bus Element13', 'w4'; ...
           'In Bus Element',   'px'; 'In Bus Element1',  'py'; ...
           'In Bus Element2',  'pz'; 'In Bus Element14', 'vx'; ...
           'In Bus Element24', 'vy'; 'In Bus Element25', 'vz'; ...
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

% --- 시나리오: 1 m 젠틀무브 (표준 하네스) ---
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; Amp = 1.0;
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

fprintf('\n>>> 골든 런 실행 (1 m 젠틀무브, T=%g s)\n', T);
sim(mdl);

% --- 균일 리샘플 후 CSV ---
tu = (0:0.002:T)';                      % 500 Hz
gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
M = [tu, gi(w1), gi(w2), gi(w3), gi(w4), ...
     gi(px), gi(py), gi(pz), gi(vx), gi(vy), gi(vz), ...
     gi(rl), gi(pt), gi(yw)];

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
outFile = fullfile(csvDir, 'golden_plant_trace.csv');
Tb = array2table(M, 'VariableNames', ...
     {'t', 'w1_rpm', 'w2_rpm', 'w3_rpm', 'w4_rpm', ...
      'px', 'py', 'pz', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw'});
writetable(Tb, outFile);
fprintf('CSV 저장: %s  (%d행)\n', outFile, size(M,1));

% --- 요약 ---
PX = M(:,6);  PY = M(:,7);  PZ = M(:,8);
VX = M(:,9);  VY = M(:,10); VZ = M(:,11);
RL = M(:,12); PT = M(:,13); YW = M(:,14);
W  = abs(M(:,2:5));

h = (tu >= 2.0 & tu <= 2.8);
fprintf('\n[t=2.0~2.8 s 호버 구간 — 대조 초기조건]\n');
fprintf('  위치   : %.5f  %.5f  %.5f  m\n', mean(PX(h)), mean(PY(h)), mean(PZ(h)));
fprintf('  속도   : %.5f  %.5f  %.5f  m/s\n', mean(VX(h)), mean(VY(h)), mean(VZ(h)));
fprintf('  자세   : %.4f  %.4f  %.4f  deg\n', ...
        rad2deg(mean(RL(h))), rad2deg(mean(PT(h))), rad2deg(mean(YW(h))));
fprintf('  |w|    : %.2f  %.2f  %.2f  %.2f  rpm\n', ...
        mean(W(h,1)), mean(W(h,2)), mean(W(h,3)), mean(W(h,4)));

fprintf('\n[기동 후 꼬리 구간 t=8~12 s]\n');
tl = (tu >= 8 & tu <= 12);
pd = rad2deg(PT(tl));
fprintf('  pitch RMS : %.4f deg  (1.8 Hz 짐 모드가 여기 나타난다)\n', ...
        sqrt(mean((pd - mean(pd)).^2)));
fprintf('\n다음: python compare_golden.py\n');
