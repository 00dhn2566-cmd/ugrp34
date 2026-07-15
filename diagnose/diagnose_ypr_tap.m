%% YPR 내부 태핑 (발산 조건 C=0.01, 원본 배선): "명령 0.6° vs 실제 pitch 15~29°" 퍼즐 판정
%% 분기: Add7(자세오차) 크다 + 모터 천장 -> 포화 권한상실 / Add7 ~0 -> 측정경로가 딴 신호(제3 주입원)
%% 태핑: YPR의 Pitch Cmd 입력, Add7 출력(오차), Control Pitch 출력(자세 PID), 믹서 4출력, 실측 w1~4

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 투하 로직 무력화 ---
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

% --- Maneuver Controller 하위 링크 해제 + 강클램프 ---
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

% --- YPR 서브시스템 (개행 함정 대응) ---
mcKids = find_system(mc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
ypr = '';
for i = 1:numel(mcKids)
    if strcmp(strtrim(regexprep(get_param(mcKids{i},'Name'),'\s+',' ')), 'Altitude and YPR Control')
        ypr = mcKids{i};
    end
end
if isempty(ypr); error('YPR 서브시스템 미발견'); end
try
    if any(strcmp(get_param(ypr, 'LinkStatus'), {'resolved','inactive'}))
        set_param(ypr, 'LinkStatus', 'none');
    end
catch
end

% YPR 내부 태핑: {블록, 포트, 변수명}
taps = { 'Pitch Cmd', 1, 'ypr_cmd_p';     % PC가 준 pitch 명령 [rad]
         'Add7', 1, 'ypr_err_p';          % 자세 오차 (PID 입력)
         'Control Pitch', 1, 'ypr_pid_p'; % 자세 PID 출력 (Motor Pitch)
         'Filter Pitch', 1, 'ypr_meas_p' };% YPR이 보는 측정 pitch
for i = 1:size(taps,1)
    src = [ypr '/' taps{i,1}];
    twBlk = [ypr '/TW ' taps{i,3}];
    old = find_system(ypr, 'SearchDepth', 1, 'Name', ['TW ' taps{i,3}]);
    if ~isempty(old); delete_block(old{1}); end
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', taps{i,3}, 'SaveFormat', 'StructureWithTime');
    sph = get_param(src, 'PortHandles');
    tph = get_param(twBlk, 'PortHandles');
    add_line(ypr, sph.Outport(taps{i,2}), tph.Inport(1), 'autorouting', 'on');
end
fprintf('YPR 태핑 4개 + 강클램프 C=%.2f 완료\n', C);

% --- 궤적 (1m/0.67s 원본) + Scope 로깅 ---
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
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element11','w1'; 'In Bus Element10','w2'; 'In Bus Element12','w3'; 'In Bus Element13','w4'};
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

fprintf('\n===== 발산 비행 (C=0.01) YPR 내부 관찰 =====\n');
sim(mdl);

tu = (0:0.005:T)';
gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
xg = gi(px); zg = gi(pz);
pg = rad2deg(gi(real_pitch)); yw = rad2deg(gi(real_yaw));
cmdp = rad2deg(gi(ypr_cmd_p));
errp = rad2deg(gi(ypr_err_p));
pidp = gi(ypr_pid_p);
measp = rad2deg(gi(ypr_meas_p));
wCeil = 1025;
W = [abs(gi(w1)), abs(gi(w2)), abs(gi(w3)), abs(gi(w4))] / wCeil * 100;

fprintf('  시각 |  cmdP    측정P(YPR)  실제P  | errP    PID출력 | 모터%%(1/2/3/4)      | x     z    yaw\n');
for ct = [3.0 3.05 3.1 3.2 3.3 3.5 3.7 4.0 4.5 5.0 6.0 8.0]
    [~,i2] = min(abs(tu-ct));
    fprintf('  t=%5.2f | %+7.2f %+8.2f %+7.1f | %+7.2f %+8.1f | %3.0f %3.0f %3.0f %3.0f | %5.2f %5.2f %6.1f\n', ...
        tu(i2), cmdp(i2), measp(i2), pg(i2), errp(i2), pidp(i2), W(i2,1), W(i2,2), W(i2,3), W(i2,4), xg(i2), zg(i2), yw(i2));
end
i3 = tu >= 3 & tu <= 5;
fprintf('\n  >> 판정 재료: |errP|피크 %.1f도 / |PID출력|피크 %.1f / 모터피크 %.0f%% / 모터바닥 %.0f%%\n', ...
    max(abs(errp(i3))), max(abs(pidp(i3))), max(W(i3&true,:),[],'all'), min(W(i3&true,:),[],'all'));
fprintf('  (YPR측정P == 실제P 인지 먼저 확인. 다르면 측정경로 문제 = 제3 주입원)\n');
