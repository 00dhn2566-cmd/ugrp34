%% 위치 채널 미세조정 2라운드: 격자 확장 (1라운드 승자 9/3.6가 모서리 - 바깥 탐색)
%% 1라운드 경향: kp/kd 증가 -> 추종/오버슈트 개선, 꼬리 완만 악화 (4.08->4.70).
%% 이번: kp {9,10,11} x kd {3.6,4.0,4.4} - 이득 꺾이는 지점 또는 꼬리 급증 지점 탐색.
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
kpList = [9, 10, 11];
kdList = [3.6, 4.0, 4.4];
fprintf('===== 위치 2라운드: kp x kd 확장 (ki=0.04 고정, 자세 새 게인 하) =====\n');
fprintf('%5s %5s | %8s %9s %8s %8s\n','kp','kd','추종cm','오버슈트cm','꼬리','호버cm');
for a = 1:numel(kpList)
    for d = 1:numel(kdList)
        kp_position = kpList(a); kd_position = kdList(d);
        try
            sim(mdl);
        catch e
            fprintf('%5.1f %5.1f | 시뮬 실패: %s\n', kpList(a), kdList(d), e.message);
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
        fprintf('%5.1f %5.1f | %8.2f %9.1f %8.2f %8.2f%s\n', kpList(a), kdList(d), mv, ov, tailv, hovp, ...
            tern(kpList(a)==9 && abs(kdList(d)-3.6)<1e-9, '  <1R승자',''));
    end
end
fprintf('(판정: 추종/오버슈트/꼬리 종합. posErrSat=1.2/kp 자동 연동됨에 유의)\n');

function s = tern(c,a,b); if c; s=a; else; s=b; end; end
