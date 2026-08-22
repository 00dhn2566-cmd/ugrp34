%% 1.8 Hz 꼬리 진동의 정체 — 루프 모드인가 중력 진자인가
%%
%% 배경 충돌:
%%   기록(TUNING_STATUS §V/§567): "제어 루프 바깥의 저중심 중력 진자, w^2 = g/L, L=8.1cm"
%%   골든 트레이스 실측(08-18): 꼬리 구간에서 tau_prop 만으로 각가속도가 상관 0.893,
%%                              이득 1.052 로 설명된다 -> 다른 복원 토크가 없다는 뜻
%%
%% 판별 원리:
%%   중력 진자면  w^2 = g/L  -> 게인과 무관, 질량과 무관
%%   루프 모드면  게인을 바꾸면 주파수가 움직인다
%%
%% 구성 (전부 짐 고정, 자세 게인만 스윕 -> 게인 의존성 단독 분리):
%%   A sA=0.5   B sA=1.0(현행)   C sA=2.0     @ 1kg
%%   D sA=1.0                                 @ 2kg  (질량 의존성)
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

scope = [mdl '/Scope'];
sigMap = {'In Bus Element3', 'pt'; 'In Bus Element4', 'rl'; 'In Bus Element', 'px'};
for i = 1:size(sigMap, 1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, ...
              'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 16; tStep = 3; Amp = 1.0;     % 꼬리를 길게 (주파수 분해능)
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt - tStep)/0.67, 0), 1);      % 빠른 이동 = 모드 강하게 가진
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

G0.kp_att = -85;  G0.ki_att = -10;  G0.kd_att = -127.5;

% {라벨, 자세게인 배율 sA, 짐질량}
cfgs = { 'A sA=0.5 @1kg', 0.5, 1.0; ...
         'B sA=1.0 @1kg', 1.0, 1.0; ...
         'C sA=2.0 @1kg', 2.0, 1.0; ...
         'D sA=1.0 @2kg', 1.0, 2.0 };

nC = size(cfgs, 1);
rows = nan(nC, 4);
fprintf('\n===== 1.8 Hz 정체 판별 =====\n');
fprintf('%-16s | %7s %7s | %10s %12s\n', '구성', 'sA', '짐kg', '꼬리주파수', 'pitch RMS');
for c = 1:nC
    sA = cfgs{c,2};  m_pkg = cfgs{c,3};
    pkgSize = [1 1 1] * 0.14;
    pkgDensity = m_pkg / (pkgSize(1)*pkgSize(2)*pkgSize(3));
    kp_attitude = G0.kp_att * sA;
    ki_attitude = G0.ki_att * sA;
    kd_attitude = G0.kd_att * sA;

    try
        sim(mdl);
    catch e
        fprintf('%-16s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.002:T)';
    gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    pd = rad2deg(gi(pt));
    sel = (tu >= 9) & (tu <= 16);          % 꼬리 7초
    x = pd(sel) - mean(pd(sel));
    Nw = numel(x);
    wnd = 0.5*(1 - cos(2*pi*(0:Nw-1)'/(Nw-1)));   % Hann (툴박스 불필요)
    x = x .* wnd;
    S = abs(fft(x));  S = S(1:floor(numel(x)/2));
    fax = (0:numel(S)-1)' / (numel(x)*0.002);
    S(1) = 0;
    [~, k] = max(S);
    f0 = fax(k);
    rms = sqrt(mean((pd(sel) - mean(pd(sel))).^2));
    rows(c,:) = [sA, m_pkg, f0, rms];
    fprintf('%-16s | %7.2f %7.2f | %9.3f Hz %10.4f deg\n', cfgs{c,1}, sA, m_pkg, f0, rms);
end

fprintf('\n판정:\n');
fprintf('  A/B/C 주파수가 게인에 따라 움직이면  -> 폐루프 모드\n');
fprintf('  A/B/C 주파수가 불변이면              -> 루프 바깥 물리 모드 (중력 진자)\n');
if all(~isnan(rows(1:3,3)))
    sp = (max(rows(1:3,3)) - min(rows(1:3,3))) / mean(rows(1:3,3)) * 100;
    fprintf('  실측 게인 4배(0.5~2.0) 구간 주파수 변동폭 : %.1f %%\n', sp);
end

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', {'sA', 'pkg_kg', 'freq_Hz', 'pitch_rms_deg'});
writetable(Tb, fullfile(csvDir, 'probe_18hz_origin.csv'));
fprintf('CSV 저장: probe_18hz_origin.csv\n');
