%% World-frame PID 출력(pid_out, 특히 z채널)과 cmd_roll을 t=0.2~1.2s 구간에서
%% 촘촘하게 같이 로깅해서, z채널 출력이 roll로 새어 들어오는 타이밍이 맞는지 확인.
%% (Matrix Multiply의 R_BI 회전을 통한 축간 커플링 가설 검증)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.5;  ki_altitude = 0.1;  kd_altitude = 0.3;
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;  % 원래값

mdl = 'quadcopter_package_delivery';
load_system(mdl);

dt = 0.01;
T = 2;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
startPoint = [0, 0, 0];
ramp_time = 2.0;
spline_data = zeros(N,3);
for i = 1:N
    tt = timespot_spl(i);
    frac = min(tt/ramp_time, 1.0);
    frac_smooth = 3*frac^2 - 2*frac^3;
    spline_data(i,:) = startPoint + frac_smooth*(hoverPoint - startPoint);
end
spline_yaw = zeros(N, 1);
waypoints = [hoverPoint; hoverPoint + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

pc = [mdl '/Maneuver Controller/Position Control'];
scope = [mdl '/Scope'];

% pid_out (World frame, PID Controller 출력) + cmd_roll(Element21) + real_roll
tapPoints = {[pc '/PID Controller'], 'y', 'pid_out'};
for i = 1:size(tapPoints,1)
    blk = tapPoints{i,1};
    varName = tapPoints{i,3};
    ph = get_param(blk, 'PortHandles');
    srcPortH = ph.Outport(1);
    twName = ['To Workspace ' varName];
    if isempty(find_system(pc, 'SearchDepth', 1, 'Name', twName))
        twBlk = [pc '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', varName, 'SaveFormat', 'StructureWithTime');
        twPh = get_param(twBlk, 'PortHandles');
        add_line(pc, srcPortH, twPh.Inport(1), 'autorouting', 'on');
    end
end

sigMap = {'In Bus Element21','cmd_roll'; 'In Bus Element4','real_roll'; 'In Bus Element1','real_y'};
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

fprintf('=== pid_out(월드프레임, x/y/z) vs cmd_roll/real_roll, t=0.2~1.2s ===\n');
simOut = sim(mdl);
t = pid_out.time(:);
pidOut = pid_out.signals.values;
cRoll = rad2deg(cmd_roll.signals.values(:));
rRoll = rad2deg(real_roll.signals.values(:));
ry = real_y.signals.values(:);

fprintf('\n  %6s  %9s %9s %9s   %9s %9s %9s\n', 't', 'pidX','pidY','pidZ','cmdRoll','realRoll','real_y');
checkTimes = 0.2:0.05:1.2;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  %6.2f  %9.4f %9.4f %9.4f   %9.3f %9.3f %9.4f\n', ...
        t(idx), pidOut(idx,1), pidOut(idx,2), pidOut(idx,3), cRoll(idx), rRoll(idx), ry(idx));
end
