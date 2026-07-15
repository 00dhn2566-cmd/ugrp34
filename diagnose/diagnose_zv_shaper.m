%% 정지 중 흔들림(1.8Hz 저중심 모드) 처방: ZV input shaper 실증
%% 원리: 기준 궤적을 ZV 임펄스열([0.5, 0.5] @ 반주기 0.278s)로 컨볼루션 -> 이동 가속이
%%   모드를 때리는 가진이 반주기 간격 두 번으로 쪼개져 상쇄 -> 도착 후 잔류 진동 소거.
%% 케이스: A 성형만(기준: 꼬리 RMS ~3.7도) / B 성형+ZV / C 성형+ZV+자세I(-10, 채택 후보 조합)
%% 대가: 도착이 반주기(0.28s)만큼 늦어짐 - 임무상 무시 가능.
%% 규칙: 메모리 수술만, save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
FSWING = 1.80;                 % 실측 모드 주파수 [Hz]
dt = 0.01; T = 14; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smBase = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

% --- ZV 셰이퍼: y(t) = 0.5*u(t) + 0.5*u(t - T/2) ---
dHalf = round(1/(2*FSWING)/dt);
zv = @(P) 0.5*P + 0.5*[repmat(P(1,:), dHalf, 1); P(1:end-dHalf, :)];
smZV = zv(smBase);

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

waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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

KI0 = ki_attitude;
cases = { 'A 성형만 (기준)',      smBase, KI0;
          'B 성형+ZV',            smZV,   KI0;
          'C 성형+ZV+자세I(-10)', smZV,   -10 };

fprintf('===== ZV input shaper 실증 (모드 %.2fHz, 반주기 %.3fs) =====\n', FSWING, dHalf*dt);
summ = {};
for ci = 1:size(cases,1)
    mws.assignin('spline_data', cases{ci,2});
    ki_attitude = cases{ci,3};
    fprintf('\n--- %s ---\n', cases{ci,1});
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        summ(end+1,:) = {cases{ci,1}, NaN, NaN, NaN, NaN}; %#ok<SAGROW>
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(sig) interp1(sig.time(:), sig.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz);
    pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));
    xr = interp1(tt, cases{ci,2}(:,1), tu);
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    iM = seg(3, 7); iT = seg(7, 14);
    eRms = sqrt(mean((xg(iM)-xr(iM)).^2))*100;
    tailP = rmsf(pg(iT));
    tailR = rmsf(rg(iT));
    attPk = max(abs(pg(iM)));
    % 1cm 도달 시각
    tArr = NaN;
    okm = abs(xg - A) < 0.01;
    ii0 = find(tu >= tStep, 1);
    for ii = ii0:numel(tu)
        if all(okm(ii:end)); tArr = tu(ii) - tStep; break; end
    end
    fprintf('  이동 추종 RMS %.1fcm | 과도 자세피크 %.1f도 | 도달(1cm) %.2fs | 꼬리 pitch RMS %.2f도 / roll %.2f도 | z %.2f~%.2f\n', ...
        eRms, attPk, tArr, tailP, tailR, min(zg(seg(3,14))), max(zg(seg(3,14))));
    summ(end+1,:) = {cases{ci,1}, eRms, attPk, tArr, tailP}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%-22s | %8s | %8s | %8s | %12s\n', '케이스','RMS[cm]','과도[도]','도달[s]','꼬리RMS[도]');
for ci = 1:size(summ,1)
    fprintf('%-22s | %8.1f | %8.1f | %8.2f | %12.2f\n', summ{ci,:});
end
fprintf('(B의 꼬리 RMS가 A 대비 크게 줄면 = ZV가 저중심 모드 가진을 상쇄 - 정지 중 흔들림 처방 실증)\n');
