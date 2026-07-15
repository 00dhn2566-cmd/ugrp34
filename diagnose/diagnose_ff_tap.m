%% 옆문 확인사살: Position Control 내부 태핑 + z 오차 클램프 A/B (발산 조건 C=0.01)
%% 해부 결과(pc_anatomy): z 오차만 무클램프로 PID 진입, Matrix Multiply(RBI)가
%%   기울기·yaw에 따라 z/x/y 명령을 혼합 -> z 누설 + yaw 오염의 물리적 주소.
%% 케이스 A: C=0.01 강클램프(x,y) 원본 배선 - PID 3성분/MM 출력 태핑으로 누설 실측
%% 케이스 B: A + PosErr Sat Z(±0.01) 삽입 - 조기 pitch가 죽으면 옆문 봉인 확인
%% 궤적: 1m/0.67s 원본(발산 확정 조건, 스무더 미적용). 규칙: 메모리 수술만.

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

% --- Position Control 링크 해제 + 강클램프 + 내부 태핑 ---
pc = [mdl '/Maneuver Controller/Position Control'];
p = pc;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
C = 0.01;
set_param([pc '/PosErr Sat X'], 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));
set_param([pc '/PosErr Sat Y'], 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));

taps = { 'PID Controller', 1, 'pid_out';    % 위치 PID 출력 (월드 3성분)
         'Matrix Multiply', 1, 'mm_out';    % RBI 변환 후 (바디)
         'Mux rpy', 1, 'rpy_meas' };        % RBI가 먹는 측정 rpy
for i = 1:size(taps,1)
    src = [pc '/' taps{i,1}];
    twBlk = [pc '/TW ' taps{i,3}];
    old = find_system(pc, 'SearchDepth', 1, 'Name', ['TW ' taps{i,3}]);
    if ~isempty(old); delete_block(old{1}); end
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', taps{i,3}, 'SaveFormat', 'StructureWithTime');
    sph = get_param(src, 'PortHandles');
    tph = get_param(twBlk, 'PortHandles');
    add_line(pc, sph.Outport(taps{i,2}), tph.Inport(1), 'autorouting', 'on');
end
fprintf('내부 태핑 3개 + 강클램프 C=%.2f 완료\n', C);

% --- 궤적: 1m/0.67s 원본 (발산 조건) ---
dt = 0.01; T = 12; tStep = 3; A = 1.0;
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

labels = {'A: 원본 배선 (z 무클램프)', 'B: z 오차도 클램프'};
for ci = 1:2
    if ci == 2
        % --- 케이스 B: PosErr Sat Z 삽입 ---
        dmx = [pc '/PosErr Demux'];
        mux = [pc '/PosErr Mux'];
        dph = get_param(dmx, 'PortHandles');
        l = get_param(dph.Outport(3), 'Line');
        if l == -1; error('PosErr Demux out3 라인 없음 - 배선 예상 불일치'); end
        delete_line(l);
        satZ = [pc '/PosErr Sat Z'];
        old = find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat Z');
        if ~isempty(old); delete_block(old{1}); end
        add_block('simulink/Discontinuities/Saturation', satZ, 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));
        zph = get_param(satZ, 'PortHandles');
        mph = get_param(mux, 'PortHandles');
        add_line(pc, dph.Outport(3), zph.Inport(1), 'autorouting', 'on');
        add_line(pc, zph.Outport(1), mph.Inport(3), 'autorouting', 'on');
        fprintf('\nPosErr Sat Z(±%.2f) 삽입 완료\n', C);
    end
    fprintf('\n===== 케이스 %s =====\n', labels{ci});
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', getReport(e, 'basic'));
        continue;
    end
    tu = (0:0.005:12)';
    xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
    zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), tu, 'linear', 'extrap'));
    pidv = interp1(pid_out.time(:), squeeze(pid_out.signals.values), tu, 'linear', 'extrap');
    mmv  = interp1(mm_out.time(:),  squeeze(mm_out.signals.values),  tu, 'linear', 'extrap');
    if size(pidv,2) ~= 3; pidv = pidv'; end
    if size(mmv,2) ~= 3; mmv = mmv'; end
    fprintf('  시각 |   x      z    | pitch   yaw  | PID(wx)  PID(wy)  PID(wz) | MM(bx)   MM(by)\n');
    for ct = [3.1 3.3 3.5 3.7 4 4.5 5 6 8 10]
        [~,i2] = min(abs(tu-ct));
        fprintf('  t=%4.1f | %5.2f  %5.2f | %6.1f %6.1f | %+8.3f %+8.3f %+8.3f | %+8.3f %+8.3f\n', ...
            tu(i2), xg(i2), zg(i2), pg(i2), yw(i2), pidv(i2,1), pidv(i2,2), pidv(i2,3), mmv(i2,1), mmv(i2,2));
    end
    i3 = tu >= 3 & tu <= 6;
    fprintf('  >> 조기(3~6s): pitch 피크 %.1f도 / yaw 피크 %.1f도 / |PIDz| 피크 %.3f vs |PIDx| 피크 %.3f / 최대|x| %.2f / z최저 %.2f\n', ...
        max(abs(pg(i3))), max(abs(yw(i3))), max(abs(pidv(i3,3))), max(abs(pidv(i3,1))), max(abs(xg)), min(zg(i3)));
end
