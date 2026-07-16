%% PID 미세조정 2라운드: kp 확장 스윕 (1R 승자 -90이 격자 가장자리 -> 아래로 확장)
%% filtD=2500 고정(1R에서 영향 미미), kd=1.5*kp 비율 고정, ki=-10.
%% 발견 검증 겸: kp -100 부근의 호버 지터(0.076도)가 -90에서 소멸(0.005도)한 현상의
%% kp 의존 곡선을 그린다 (모터 동역학 공진 이탈 가설).

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

kpList = [-70, -75, -80, -85, -90, -95];
filtD_attitude = 2500;
ki_attitude = -10;
fprintf('===== 자세 2라운드: kp 확장 (filtD=2500, kd=1.5kp) =====\n');
fprintf('기준(현행 -100/2000): 호버 0.076 이동 4.02 꼬리 4.33\n');
fprintf('%6s | %8s %8s %8s\n','kp','호버','이동cm','꼬리');
res = {};
for a = 1:numel(kpList)
    kp_attitude = kpList(a); kd_attitude = 1.5*kpList(a);
    try
        sim(mdl);
    catch e
        fprintf('%6.0f | 시뮬 실패: %s\n', kpList(a), e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));
    xr = interp1(tt, sm(:,1), tu);
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    hov = max(rmsf(pg(seg(1,3))), rmsf(rg(seg(1,3))));
    mv  = sqrt(mean((xg(seg(3,7))-xr(seg(3,7))).^2))*100;
    tailv = rmsf(pg(seg(8,14)));
    fprintf('%6.0f | %8.4f %8.2f %8.2f\n', kpList(a), hov, mv, tailv);
    res(end+1,:) = {kpList(a), hov, mv, tailv}; %#ok<SAGROW>
end
fprintf('\n(판정: 호버 최소 + 이동/꼬리 비열등 지점. 이동 >4.3cm 또는 꼬리 >4.6 나오면 강성 손실 신호)\n');
