%% 위치 채널 미세조정 6라운드: 승자 영역 미세 격자 (절벽 33 대비 여유 22~26, 비율 축 추가) (3라운드 kp 19까지 절벽 없음 - 더 바깥 탐색)
%% 3라운드 신호: 개선 둔화(점당 추종 이득 0.46->0.19cm) + kp 19에서 자세피크 첫 상승(13.7->14.7도).
%% 이번: (22,8.8)(26,10.4)(30,12.0) - 진짜 절벽 위치를 찾아 여유폭 확정. 첫 행 19/7.6은 기준점.
%% 주의: posErrSat=1.2/kp 자동 연동. 자세 새 게인(-85/-127.5/2500)은 parameters.m에서 로드.

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

% 새 자세 게인은 parameters.m에서 이미 로드됨 (-85/-127.5/2500)
pairs = [22 7.7; 22 8.8; 22 9.9; 24 8.4; 24 9.6; 24 10.8; 26 9.1; 26 10.4; 26 11.7];   % kp{22,24,26} x 비율{0.35,0.40,0.45}
rows = nan(size(pairs,1), 8);
fprintf('===== 위치 6라운드: 승자 영역 미세 격자 (절벽 33 대비 여유 22~26, 비율 축 추가) kd/kp=0.4 (ki=0.04 고정, 자세 새 게인 하) =====\n');
fprintf('%5s %5s | %8s %9s %8s %8s %8s\n','kp','kd','추종cm','오버슈트cm','꼬리','호버cm','피크deg');
for a = 1:size(pairs,1)
    kp_position = pairs(a,1); kd_position = pairs(a,2);
    try
        sim(mdl);
    catch e
        fprintf('%5.1f %5.1f | 시뮬 실패: %s\n', pairs(a,1), pairs(a,2), e.message);
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
    pk  = max(abs(pg));   % 절벽 감지용: 지금까지 정상 케이스는 ~13도 수준
    rows(a,:) = [pairs(a,1), pairs(a,2), 0.04, mv, ov, tailv, hovp, pk];
    fprintf('%5.1f %5.1f | %8.2f %9.1f %8.2f %8.2f %8.1f\n', pairs(a,1), pairs(a,2), mv, ov, tailv, hovp, pk);
end
fprintf('(판정: 추종/오버슈트/꼬리/피크 종합. posErrSat=1.2/kp 자동 연동됨에 유의)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'kp','kd','ki','tracking_rms_cm','overshoot_cm','tail_rms_deg','hover_cm','peak_pitch_deg'});
writetable(Tb, fullfile(csvDir, 'refine_pos_r6.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_pos_r6.csv'));


