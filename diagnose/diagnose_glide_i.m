%% 활공 정상오차 처방 실증: 자세 I 도입 (사용자 가설 - 활공 국면 한정 재검증)
%% ypr_tap 실측: 활공 중 pitch +5~7° 정상오차 = 드래그x저중심 레버 토크를 P항만으로
%%   상쇄한 평형 (PID출력 +10.5 = kp x 오차, 모터 여유 61~64%). I가 있으면 오차가
%%   0으로 눌리고 -> 기울기 소멸 -> 활공 급전 차단 예상.
%% 조건: C=0.01 발산 비행 (ypr_tap과 동일, T=8s). 기준(ki=0): x@8s=22.97m, pitch@8s=+5.0°
%% 주의: ki_attitude 부호는 음수 규약 (kp=-100과 동일 방향).

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

mc = [mdl '/Maneuver Controller'];
p = mc;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
pc = [mc '/Position Control'];
C = 0.01;
set_param([pc '/PosErr Sat X'], 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));
set_param([pc '/PosErr Sat Y'], 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));

dt = 0.01; T = 8; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', [xk, zeros(N,1), ones(N,1)]);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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

kiVals = [-10, -30];
fprintf('===== 활공 자세 I 실증 (기준 ki=0: x@8s=22.97m, pitch@8s=+5.0도, 오차 -5.9도) =====\n');
for k = 1:numel(kiVals)
    ki_attitude = kiVals(k);
    fprintf('\n--- ki_attitude = %g ---\n', ki_attitude);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    tu = (0:0.005:T)';
    xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
    zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), tu, 'linear', 'extrap'));
    for ct = [3.5 4 4.5 5 6 7 8]
        [~,i2] = min(abs(tu-ct));
        fprintf('  t=%4.1f | x %6.2f | P %+6.1f | Y %+6.1f | z %5.2f\n', tu(i2), xg(i2), pg(i2), yw(i2), zg(i2));
    end
    iG = tu >= 6;
    fprintf('  >> x@8s %.2fm (기준 22.97) | 활공 pitch 평균(6~8s) %+.2f도 (기준 +6도대) | z최저 %.2f\n', ...
        xg(end), mean(pg(iG)), min(zg(tu>=3)));
end
