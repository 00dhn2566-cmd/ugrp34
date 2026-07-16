%% PID 미세조정 실모델 좌표하강 1라운드: 자세 kp x filtD 격자 (현행 주변)
%% 종합점수 = 0.4*호버RMS/기준 + 0.3*이동RMS/기준 + 0.3*꼬리RMS/기준 (낮을수록 좋음, 1=현행)
%% 성형 1m 비행 단일 시나리오(빠른 스윕용). 승자는 2라운드에서 주변 좁힘.
%% 규칙: 메모리 수술만(게인은 base 주입), save_system 금지.

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

% 격자: kp x filtD (kd는 kp에 비례 유지 = kd/kp 1.5 고정)
kpList = [-90, -100, -110];
fDList = [1500, 2000, 2500];
ki_attitude = -10;   % 채택값 고정
fprintf('===== 자세 1라운드: kp x filtD (kd=1.5*kp, ki=-10 고정) =====\n');
fprintf('%6s %6s | %8s %8s %8s %8s | %6s\n','kp','filtD','호버','이동cm','과도도','꼬리','점수');
res = {};
base = struct('hov',NaN,'mv',NaN,'tail',NaN);
for a = 1:numel(kpList)
    for f = 1:numel(fDList)
        kp_attitude = kpList(a); kd_attitude = 1.5*kpList(a); filtD_attitude = fDList(f);
        try
            sim(mdl);
        catch e
            fprintf('%6.0f %6.0f | 시뮬 실패: %s\n', kpList(a), fDList(f), e.message);
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
        if kpList(a)==-100 && fDList(f)==2000
            base.hov=hov; base.mv=mv; base.tail=tailv;
        end
        res(end+1,:) = {kpList(a), fDList(f), hov, mv, tailv}; %#ok<SAGROW>
    end
end
% 점수 (기준 대비)
fprintf('\n--- 결과 (점수 = 0.4*호버+0.3*이동+0.3*꼬리, 기준 대비, <1 개선) ---\n');
best = inf; bi = 0;
for i = 1:size(res,1)
    sc = 0.4*res{i,3}/base.hov + 0.3*res{i,4}/base.mv + 0.3*res{i,5}/base.tail;
    fprintf('%6.0f %6.0f | %8.3f %8.2f %8s %8.2f | %6.3f%s\n', ...
        res{i,1}, res{i,2}, res{i,3}, res{i,4}, '-', res{i,5}, sc, ...
        tern(res{i,1}==-100&&res{i,2}==2000,' <현행',''));
    if sc < best; best = sc; bi = i; end
end
fprintf('\n>> 1라운드 승자: kp=%.0f filtD=%.0f (점수 %.3f). 2라운드는 이 주변 ±5%%로 좁힘\n', ...
    res{bi,1}, res{bi,2}, best);

function s = tern(c,a,b); if c; s=a; else; s=b; end; end
