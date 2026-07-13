%% 모터 ref(~8663 rad/s)가 Bias(634 rad/s 상당)보다 14배나 큰 이유 추적.
%% Altitude and YPR Control 내부의 각 채널(Bias 출력, Thrust Control 출력,
%% Motor Thrust)과 Motor Mixer 출력(w1)을 분해 로깅해서
%% 어느 단계에서 ~8000 rad/s 상당의 과대값이 유입되는지 확인.

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
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
biasBlk = [ypr '/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', '100.98');

for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
end
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
flipSigns = @(s) strrep(strrep(strrep(s, '+', 'X'), '-', '+'), 'X', '-');
set_param([mixer '/Add5'], 'Inputs', flipSigns(get_param([mixer '/Add5'], 'Inputs')));
set_param([mixer '/Add7'], 'Inputs', flipSigns(get_param([mixer '/Add7'], 'Inputs')));

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

set_param(mdl, 'StopTime', num2str(T));

% --- 태핑 대상 탐색: YPR Control 내부 블록 목록 먼저 출력 ---
blks = find_system(ypr, 'SearchDepth', 1, 'LookUnderMasks','all','FollowLinks','on');
fprintf('=== Altitude and YPR Control 1단계 블록 목록 ===\n');
for i = 1:numel(blks)
    if ~strcmp(blks{i}, ypr)
        fprintf('  %s [%s]\n', strrep(blks{i}, [ypr '/'], ''), get_param(blks{i},'BlockType'));
    end
end

% Bias Chassis 출력, Thrust Control(있으면) 출력, Motor Mixer w1 출력 태핑
taps = {};
% Bias Chassis 출력
bph = get_param(biasBlk, 'PortHandles');
subsys = [ypr '/Subsystem'];
twBlk = [subsys '/To Workspace bias_out'];
if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'To Workspace bias_out'))
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'bias_out', 'SaveFormat', 'StructureWithTime');
    twPh = get_param(twBlk, 'PortHandles');
    add_line(subsys, bph.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end
taps{end+1} = 'bias_out';

% Motor Mixer의 w1 출력 (Add4)
mph = get_param([mixer '/Add4'], 'PortHandles');
twBlk = [mixer '/To Workspace mix_w1'];
if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', 'To Workspace mix_w1'))
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'mix_w1', 'SaveFormat', 'StructureWithTime');
    twPh = get_param(twBlk, 'PortHandles');
    add_line(mixer, mph.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end
taps{end+1} = 'mix_w1';

% Mixer 입력들 (thrust/pitch/roll/yaw 채널) - Add4의 입력 소스들 태핑
aph = get_param([mixer '/Add4'], 'PortHandles');
for i = 1:numel(aph.Inport)
    lineH = get_param(aph.Inport(i), 'Line');
    if lineH == -1; continue; end
    srcH = get_param(lineH, 'SrcPortHandle');
    if srcH == -1; continue; end
    varName = sprintf('mixin%d', i);
    twName = ['To Workspace ' varName];
    if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', twName))
        twBlk = [mixer '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', varName, 'SaveFormat', 'StructureWithTime');
        twPh = get_param(twBlk, 'PortHandles');
        add_line(mixer, srcH, twPh.Inport(1), 'autorouting', 'on');
    end
    srcName = get_param(srcH, 'Parent');
    fprintf('Add4 In%d <- %s (%s)\n', i, srcName, varName);
    taps{end+1} = varName; %#ok<SAGROW>
end

fprintf('\n=== 시뮬레이션 실행 ===\n');
simOut = sim(mdl);
for j = 1:numel(taps)
    v = eval([taps{j} '.signals.values']);
    v = v(:,1);
    fprintf('  %s: min=%.2f max=%.2f last=%.2f\n', taps{j}, min(v), max(v), v(end));
end
