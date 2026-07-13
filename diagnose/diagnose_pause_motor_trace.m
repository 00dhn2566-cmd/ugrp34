%% Pause Motor의 입력(y)이 어디서 오는지, 그리고 실제 시뮬레이션 동안 En 출력값이
%% 어떻게 나오는지 직접 로깅해서, 이게 결과를 게인/피드포워드와 무관하게
%% 고정시키고 있는 원인인지 확인.

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
kp_position = 1;    ki_position = 0;    kd_position = 0.5;

mdl = 'quadcopter_package_delivery';
load_system(mdl);
propeller.Kdrag = 4.222841;
assignin('base', 'propeller', propeller);

pm = [mdl '/Maneuver Controller/Altitude and  YPR Control/Pause Motor'];
fprintf('=== Pause Motor의 y(입력) 소스 ===\n');
ph = get_param(pm, 'PortHandles');
ySrcPortH = -1;
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  In%d <- %s\n', i, get_param(srcPortH, 'Parent'));
            ySrcPortH = srcPortH;
        end
    end
end

dt = 0.01;
T = 5;
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

% Pause Motor의 En 출력과 y 입력을 직접 로깅
ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
enPh = get_param(pm, 'PortHandles');
twBlk1 = [ypr '/To Workspace pm_en'];
if isempty(find_system(ypr, 'SearchDepth', 1, 'Name', 'To Workspace pm_en'))
    add_block('simulink/Sinks/To Workspace', twBlk1, 'VariableName', 'pm_en', 'SaveFormat', 'StructureWithTime');
    twPh1 = get_param(twBlk1, 'PortHandles');
    add_line(ypr, enPh.Outport(1), twPh1.Inport(1), 'autorouting', 'on');
end

twBlk2 = [ypr '/To Workspace pm_y'];
if isempty(find_system(ypr, 'SearchDepth', 1, 'Name', 'To Workspace pm_y')) && ySrcPortH ~= -1
    add_block('simulink/Sinks/To Workspace', twBlk2, 'VariableName', 'pm_y', 'SaveFormat', 'StructureWithTime');
    twPh2 = get_param(twBlk2, 'PortHandles');
    add_line(ypr, ySrcPortH, twPh2.Inport(1), 'autorouting', 'on');
end

fprintf('\n=== 시뮬레이션 실행 후 Pause Motor En/y 값 ===\n');
simOut = sim(mdl);
en = pm_en.signals.values(:);
y = pm_y.signals.values(:);
fprintf('  En: min=%g max=%g last=%g (고유값 개수=%d)\n', min(en), max(en), en(end), numel(unique(en)));
fprintf('  y : min=%g max=%g last=%g\n', min(y), max(y), y(end));
