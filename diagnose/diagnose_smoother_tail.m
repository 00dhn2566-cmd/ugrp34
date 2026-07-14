%% 성형 궤적 비행의 도착 후 자세 배회 분석 (사용자 관찰: "멈추는 항이 없어 왔다갔다")
%% 같은 비행(성형된 0.67s 스텝) 재실행 후 t=6~12 구간 고해상도 분석:
%% 1) 감쇠 판정: 전반부(6~9) vs 후반부(9~12) pitch RMS 비 - 죽어가면 <1, 한계사이클이면 ~1
%% 2) 진동 주파수 (영점 교차)
%% 3) 원인 가설 검증: pitch가 위치루프 명령(kp_pos*ex + kd_pos*dex)을 따라다니는지 상관계수
%% 4) 기준선: 스텝 전 호버(1~3s)의 같은 지표
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smKill = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

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
mws.assignin('spline_data', smKill);
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

fprintf('===== 도착 후 배회 분석 실행 =====\n');
sim(mdl);

% 균일 그리드로 리샘플
tu = (0:0.005:T)';
xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
rg = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), tu, 'linear', 'extrap'));
yg = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), tu, 'linear', 'extrap'));
xrg = interp1(tt, smKill(:,1), tu, 'linear', 'extrap');
ex = xrg - xg;                      % 위치오차 (기준-실제)
dex = gradient(ex, 0.005);

seg = @(t1,t2) (tu>=t1 & tu<t2);
rmsf = @(v) sqrt(mean(v.^2));

% 기준선: 스텝 전 호버
sB = seg(1,3);
% 도착 후 전/후반
s1 = seg(6,9); s2 = seg(9,12); sP = seg(6,12);

% 진동 주파수: pitch 평균 제거 후 영점 교차
pgP = pg(sP) - mean(pg(sP));
zc = sum(abs(diff(sign(pgP)))>0) / 2;
freq = zc / 6;

% 위치루프 명령 가설: cmd ~ kp_pos*ex + kd_pos*dex (부호/스케일 무시, 상관만)
cmdHat = kp_position*ex + kd_position*dex;
cc = corrcoef(pg(sP), cmdHat(sP)); ccP = cc(1,2);
cc = corrcoef(pg(sP), ex(sP)); ccE = cc(1,2);

fprintf('\n----- 기준선 (스텝 전 호버 1~3s) -----\n');
fprintf('  pitch RMS %.3f도 | roll RMS %.3f도 | x RMS %.2fcm | yaw RMS %.2f도\n', ...
    rmsf(pg(sB)-mean(pg(sB))), rmsf(rg(sB)-mean(rg(sB))), rmsf(xg(sB)-mean(xg(sB)))*100, rmsf(yg(sB)-mean(yg(sB))));
fprintf('----- 도착 후 (6~12s) -----\n');
fprintf('  pitch RMS: 전반(6~9) %.3f도 / 후반(9~12) %.3f도 / 비율 %.2f (1보다 크게 작으면 감쇠 중, ~1이면 한계사이클)\n', ...
    rmsf(pg(s1)-mean(pg(s1))), rmsf(pg(s2)-mean(pg(s2))), rmsf(pg(s2)-mean(pg(s2)))/rmsf(pg(s1)-mean(pg(s1))));
fprintf('  x오차 RMS: 전반 %.2fcm / 후반 %.2fcm\n', rmsf(ex(s1))*100, rmsf(ex(s2))*100);
fprintf('  roll RMS %.3f도 / yaw RMS %.2f도 (도착 후 전체)\n', rmsf(rg(sP)-mean(rg(sP))), rmsf(yg(sP)-mean(yg(sP))));
fprintf('  pitch 진동 주파수 ~%.2f Hz (영점교차 %d회/6s)\n', freq, round(zc));
fprintf('  상관: pitch vs 위치루프명령(kp*ex+kd*dex) r=%.3f | pitch vs ex r=%.3f\n', ccP, ccE);
fprintf('  (r 크면 = 자세루프는 명령 충실 추종, 배회의 근원은 위치루프 쪽)\n');

fprintf('\n  0.25s 간격 상세 (t=6~9):\n');
for ct = 6:0.25:9
    [~,i2] = min(abs(tu-ct));
    fprintf('   t=%5.2f | ex %+6.1fmm | P %+6.2f | cmdHat %+6.3f\n', tu(i2), ex(i2)*1000, pg(i2), cmdHat(i2));
end
