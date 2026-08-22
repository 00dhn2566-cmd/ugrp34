%% 폐루프 오버슈트 vs 순항 속도 (2026-08-22)
%% 사용자 문제제기: "최대속도까지 올라가면 30 cm 까지 오버슈트, 이걸 수 cm 로 줄이고 싶다".
%%
%% 구분해 둘 것 — 오버슈트는 두 군데서 나온다.
%%   ① 기준 궤적 자체의 오버슈트 : 쉐이퍼가 만드는 것. 08-22 측정 0.00~0.87 cm (문제 아님)
%%   ② 폐루프 추종 오버슈트      : 드론이 기준을 지나치는 것. 문서 실측 20.6~22.9 cm ← 이게 문제
%% 이 스크립트는 ②를 순항 속도의 함수로 잰다. 결과가 capability.limits.v 의 근거가 된다.
%%
%% 구성: 3 m 직선(레이즈드 코사인) 이동, 이동시간만 바꿔 피크 속도를 쓸어본다. 외란 없음.
%% 규칙: save_system 금지. 투하 로직 무력화 필수.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';

DX = 3.0; Z0 = 1.0; T0 = 3.0;
TMOVES = [10.0 6.0 4.0 3.0 2.4];          % 피크 v = DX*pi/(2*T) = 0.47 0.79 1.18 1.57 1.96
R = struct('tmove', [], 'vpk', [], 'over_cm', [], 'peak_cm', [], 'settle_s', [], 'sec', []);

fprintf('\n%8s %8s %11s %11s %10s\n', '이동[s]', 'v피크', '오버슈트', '최대이탈', '정착[s]');
for k = 1:numel(TMOVES)
    TM = TMOVES(k);
    T_END = T0 + TM + 12.0;
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);
    disable_drop(mdl);
    move_setup(mdl, T_END, DX, Z0, T0, TM);
    log_signals(mdl);

    tic; sim(mdl); el = toc;
    t = real_x.time(:);
    x = real_x.signals.values(:);
    % 참조 (move_setup 과 같은 식)
    u = min(max((t - T0) / TM, 0), 1);
    xr = DX * 0.5 * (1 - cos(pi * u));
    vpk = max(abs(diff(xr) ./ diff(t)));

    over = max(x - DX);                       % 목표 너머 최대 (폐루프 오버슈트)
    peak = max(abs(x - xr));                  % 기준 대비 최대 이탈
    mEnd = t > T0 + TM;
    idx = find(abs(x - DX) > 0.02, 1, 'last');
    if isempty(idx); settle = T0 + TM; else; settle = t(idx) - (T0 + TM); end

    R.tmove(end+1) = TM;  R.vpk(end+1) = vpk;
    R.over_cm(end+1) = 100*over;  R.peak_cm(end+1) = 100*peak;
    R.settle_s(end+1) = settle;   R.sec(end+1) = el;
    fprintf('%8.1f %8.3f %10.2fcm %10.2fcm %9.2f   (%.0fs)\n', TM, vpk, 100*over, 100*peak, settle, el);
end

fprintf('\n===== 판정 =====\n');
v = R.vpk(:); o = R.over_cm(:);
% 오버슈트는 운동에너지에 비례할 것으로 예상 -> o = c * v^2 적합
c2 = (v.^2) \ o;
fprintf('적합: 오버슈트 ≈ %.2f · v²  (v [m/s], 오버슈트 [cm])\n', c2);
for tgt = [5 3 2]
    fprintf('  오버슈트 %d cm 이하로 하려면  v ≤ %.3f m/s\n', tgt, sqrt(tgt / c2));
end
save(fullfile(modelDir,'diagnose','results','sweep_overshoot_speed.mat'), '-struct', 'R');
fprintf('결과 저장: diagnose/results/sweep_overshoot_speed.mat\n');

%% ================= 로컬 =================
function disable_drop(mdl)
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
end

function move_setup(mdl, T, dx, z0, t0, tmove)
dt = 0.01; N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
u = min(max((timespot_spl - t0) / tmove, 0), 1);
xs = dx * 0.5 * (1 - cos(pi * u));
spline_data = [xs, zeros(N,1), z0*ones(N,1)];
spline_yaw = zeros(N, 1);
waypoints = [0 0 z0; dx 0 z0]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));
end

function log_signals(mdl)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, ...
        'VariableName', sigMap{i,2}, 'SaveFormat','StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    add_line(scope, srcPh.Outport(1), get_param(twBlk,'PortHandles').Inport(1), 'autorouting','on');
end
end
