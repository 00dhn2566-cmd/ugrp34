%% Position Control 체인 내부를 단계별로 추적:
%% 1) PID(s) 출력 (World frame, Matrix Multiply 이전)
%% 2) Filter 출력 (Body frame, Dir P/Dir R 이전 - 회전+필터 통과 직후)
%% 3) Err2P/Err2R 출력 (Pitch/Roll Limit 포화 직전)
%% 어느 단계에서 값이 비정상적으로 커지는지 t=0~1.5s 구간에서 확인.

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
spline_data = repmat(hoverPoint, N, 1);
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

% 탭 지점: PID Controller의 y(출력, World frame), Filter의 o(출력, Body frame),
% Err2P/Err2R의 출력(Pitch/Roll Limit 직전)
tapPoints = {
    [pc '/PID Controller'], 'y', 'pid_out';
    [pc '/Filter'], 'o', 'filter_out';
    [pc '/Err2P'], '', 'err2p_out';
    [pc '/Err2R'], '', 'err2r_out';
};

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

fprintf('=== Position Control 체인 내부 신호 추적 (t=0~1.5s) ===\n');
simOut = sim(mdl);

t = pid_out.time(:);
pidOut = pid_out.signals.values;   % World frame [x,y,z] 3채널
filterOut = filter_out.signals.values; % Body frame [pitch?,roll?,?] 3채널
err2pOut = err2p_out.signals.values(:);
err2rOut = err2r_out.signals.values(:);

fprintf('\n=== pid_out (World frame, PID(s) 출력) 채널별 범위 ===\n');
for c = 1:size(pidOut,2)
    fprintf('  ch%d: min=%.4f max=%.4f\n', c, min(pidOut(:,c)), max(pidOut(:,c)));
end

fprintf('\n=== filter_out (Body frame, Dir P/R 이전) 채널별 범위 ===\n');
for c = 1:size(filterOut,2)
    fprintf('  ch%d: min=%.4f max=%.4f\n', c, min(filterOut(:,c)), max(filterOut(:,c)));
end

fprintf('\n=== err2p_out/err2r_out (Pitch/Roll Limit 직전, rad) 범위 ===\n');
fprintf('  err2p_out: min=%.4f max=%.4f (deg: min=%.2f max=%.2f)\n', min(err2pOut), max(err2pOut), rad2deg(min(err2pOut)), rad2deg(max(err2pOut)));
fprintf('  err2r_out: min=%.4f max=%.4f (deg: min=%.2f max=%.2f)\n', min(err2rOut), max(err2rOut), rad2deg(min(err2rOut)), rad2deg(max(err2rOut)));

fprintf('\n=== 시간별 스냅샷 (err2r_out 기준, deg) ===\n');
checkTimes = 0:0.05:1.5;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    fprintf('  t=%5.2fs: err2r_out=%8.3f deg\n', t(idx), rad2deg(err2rOut(idx)));
end
