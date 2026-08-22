%% 1.73 Hz 모드가 모터 커플링인가 — 모터 파라미터 스윕 (사용자 가설)
%%
%% 앞선 실측(probe_18hz_origin, compare_golden):
%%   - 토크는 전부 프로펠러에서 나온다 (tau_prop 이득 1.05, 상관 0.89) -> 중력 진자 아님
%%   - 자세 게인 4배를 바꿔도 주파수 불변 -> 자세 루프도 아님
%%   - 남은 후보: 모터 루프 / 위치 캐스케이드 / 필터 극
%%
%% 판별: 모터 시정수와 모터 루프 게인을 흔든다.
%%   모터 커플링이면 주파수가 따라 움직인다. 불변이면 모터가 아니다.
%%
%% 개선점: 꼬리 창 10 s (분해능 0.1 Hz) + 포물선 보간으로 부빈 정밀도 확보.
%%         (앞 실험은 창이 짧아 빈 양자화에 걸렸다)
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
sigMap = {'In Bus Element3', 'pt'; 'In Bus Element11', 'w1'; 'In Bus Element10', 'w2'};
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
dt = 0.01; T = 20; tStep = 3; Amp = 1.0;      % 꼬리 10 s 확보
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt - tStep)/0.67, 0), 1);
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

TC0 = qc_motor.time_const;   KP0 = kp_motor;   KI0 = ki_motor;

% {라벨, 시정수, kp_motor 배율}
cfgs = { 'A 현행        ', TC0,     1.0; ...
         'B 모터 2배빠름 ', TC0/2,   1.0; ...
         'C 모터 2배느림 ', TC0*2,   1.0; ...
         'D 모터게인 x3 ', TC0,     3.0; ...
         'E 모터게인 /3 ', TC0,     1/3 };

nC = size(cfgs, 1);
rows = nan(nC, 5);
fprintf('\n===== 모터 커플링 판별 (꼬리 10 s, 포물선 보간) =====\n');
fprintf('%-16s | %8s %8s | %11s %11s %11s\n', ...
        '구성', '시정수', 'kp배율', 'pitch주파수', 'w1주파수', 'pitch RMS');
for c = 1:nC
    qc_motor.time_const = cfgs{c,2};
    kp_motor = KP0 * cfgs{c,3};
    ki_motor = KI0 * cfgs{c,3};
    try
        sim(mdl);
    catch e
        fprintf('%-16s | 시뮬 실패: %s\n', cfgs{c,1}, e.message);
        continue;
    end
    tu = (0:0.002:T)';
    gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    pd = rad2deg(gi(pt));
    wd = abs(gi(w1));
    sel = (tu >= 10) & (tu <= 20);
    f_pt = peakfreq(pd(sel), 0.002);
    f_w1 = peakfreq(wd(sel), 0.002);
    rms  = sqrt(mean((pd(sel) - mean(pd(sel))).^2));
    rows(c,:) = [cfgs{c,2}, cfgs{c,3}, f_pt, f_w1, rms];
    fprintf('%-16s | %8.4f %8.3f | %10.3f %10.3f %10.4f\n', ...
            cfgs{c,1}, cfgs{c,2}, cfgs{c,3}, f_pt, f_w1, rms);
end

fprintf('\n판정:\n');
if all(~isnan(rows([1 2 3], 3)))
    sp = (max(rows(1:3,3)) - min(rows(1:3,3))) / mean(rows(1:3,3)) * 100;
    fprintf('  시정수 4배 구간(0.01~0.04 s) 주파수 변동 : %.1f %%\n', sp);
end
if all(~isnan(rows([1 4 5], 3)))
    sp2 = (max(rows([1 4 5],3)) - min(rows([1 4 5],3))) / mean(rows([1 4 5],3)) * 100;
    fprintf('  모터게인 9배 구간(1/3~3) 주파수 변동   : %.1f %%\n', sp2);
end
fprintf('  둘 중 하나라도 크게 움직이면 -> 모터 커플링 확정\n');
fprintf('  둘 다 불변이면               -> 위치 캐스케이드 또는 필터 극\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
     {'time_const', 'kp_mult', 'f_pitch_Hz', 'f_w1_Hz', 'pitch_rms_deg'});
writetable(Tb, fullfile(csvDir, 'probe_motor_coupling.csv'));
fprintf('CSV 저장: probe_motor_coupling.csv\n');

function f0 = peakfreq(x, dts)
    x = x(:) - mean(x);
    Nw = numel(x);
    wnd = 0.5*(1 - cos(2*pi*(0:Nw-1)'/(Nw-1)));
    X = abs(fft(x .* wnd));
    X = X(1:floor(Nw/2));
    X(1) = 0;
    df = 1/(Nw*dts);
    [~, k] = max(X);
    if k > 1 && k < numel(X)
        a = X(k-1); b = X(k); c = X(k+1);
        k = k + 0.5*(a-c)/(a-2*b+c);        % 포물선 보간
    end
    f0 = (k-1) * df;
end
