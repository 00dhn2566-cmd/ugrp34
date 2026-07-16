%% 위치 채널 미세조정 7라운드: ki 1차원 스윕 (kp/kd는 r6 승자로 고정)
%% 목적: ①호버 유지 ②오버슈트 악화(와인드업) ③꼬리 영향만 확인. 차이 없으면 0.04 유지.
%% 실행 전: 아래 WINNER_* 두 값을 r6 승자로 채울 것 (채워지기 전 실행하면 error).

WINNER_KP = NaN;   % <- r6 승자 kp (예: 24)
WINNER_KD = NaN;   % <- r6 승자 kd (예: 9.6)
if isnan(WINNER_KP) || isnan(WINNER_KD)
    error('WINNER_KP/KD 미기입 - r6 결과 확인 후 채울 것');
end

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
tau = min(max((tt-tStep)/0.67,0),1);
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

kiList = [0.02, 0.04, 0.08];
rows = nan(numel(kiList), 8);
fprintf('===== 위치 7라운드: ki 스윕 (kp=%.1f kd=%.1f 고정) =====\n', WINNER_KP, WINNER_KD);
fprintf('%6s | %8s %9s %8s %8s %8s\n','ki','추종cm','오버슈트cm','꼬리','호버cm','피크deg');
for a = 1:numel(kiList)
    kp_position = WINNER_KP; kd_position = WINNER_KD; ki_position = kiList(a);
    try
        sim(mdl);
    catch e
        fprintf('%6.3f | 시뮬 실패: %s\n', kiList(a), e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); pg = rad2deg(gi2(real_pitch));
    xr = interp1(tt, sm(:,1), tu);
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    mv  = sqrt(mean((xg(seg(3,7))-xr(seg(3,7))).^2))*100;
    ov  = max(0, max(xg) - A)*100;
    tailv = rmsf(pg(seg(8,14)));
    hovp = rmsf(xg(seg(1,3)))*100;
    pk  = max(abs(pg));
    rows(a,:) = [WINNER_KP, WINNER_KD, kiList(a), mv, ov, tailv, hovp, pk];
    fprintf('%6.3f | %8.2f %9.1f %8.2f %8.2f %8.1f\n', kiList(a), mv, ov, tailv, hovp, pk);
end
fprintf('(판정: 호버/오버슈트 동률이면 0.04 유지. 와인드업 신호 = ki 증가 시 오버슈트 상승)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'kp','kd','ki','tracking_rms_cm','overshoot_cm','tail_rms_deg','hover_cm','peak_pitch_deg'});
writetable(Tb, fullfile(csvDir, 'refine_pos_r7_ki.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_pos_r7_ki.csv'));
