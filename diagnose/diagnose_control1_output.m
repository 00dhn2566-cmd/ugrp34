%% Control1(모터 PID)의 출력(모터 드라이브로 가는 토크 명령)을 직접 로깅.
%% 모터가 405.32 rad/s에 못 박혀 있는데, 토크 명령이 실제로 0.25(포화)로
%% 나가는지 확인 -> 제한이 PID 쪽인지 모터/배터리 쪽인지 판별.

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

propeller.Kdrag = 4.222841;
assignin('base', 'propeller', propeller);
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;
n_hover = sqrt(T_need / (propeller.Kthrust * air_rho * propeller.diameter^4));
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', num2str(n_hover, '%.4f'));

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

% Control1의 모든 아웃포트를 태핑
ctrl1 = [mdl '/Quadcopter/Electrical/Control1'];
elec = [mdl '/Quadcopter/Electrical'];
ph = get_param(ctrl1, 'PortHandles');
fprintf('Control1 아웃포트 개수: %d\n', numel(ph.Outport));
tapNames = {};
for i = 1:numel(ph.Outport)
    varName = sprintf('c1_out%d', i);
    twName = ['To Workspace ' varName];
    if isempty(find_system(elec, 'SearchDepth', 1, 'Name', twName))
        twBlk = [elec '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', varName, 'SaveFormat', 'StructureWithTime');
        twPh = get_param(twBlk, 'PortHandles');
        add_line(elec, ph.Outport(i), twPh.Inport(1), 'autorouting', 'on');
    end
    tapNames{end+1} = varName; %#ok<SAGROW>
end

% Prop1.w도 같이
scope = [mdl '/Scope'];
twName = 'To Workspace prop1_w';
oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
if ~isempty(oldTw); delete_block(oldTw{1}); end
twBlk = [scope '/' twName];
add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'prop1_w', 'SaveFormat', 'StructureWithTime');
srcPh = get_param([scope '/In Bus Element11'], 'PortHandles');
twPh = get_param(twBlk, 'PortHandles');
add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');

fprintf('=== Control1 출력(토크 명령) 로깅, Kdrag=4.222841, Bias=%.2f ===\n', n_hover);
simOut = sim(mdl);
t = prop1_w.time(:);
w = prop1_w.signals.values(:);
fprintf('  Prop1.w: min=%.2f max=%.2f last=%.2f rad/s\n', min(w), max(w), w(end));
for j = 1:numel(tapNames)
    v = eval([tapNames{j} '.signals.values']);
    v = v(:, 1);
    fprintf('  %s: min=%.4f max=%.4f last=%.4f (limit_motor=%.2f)\n', tapNames{j}, min(v), max(v), v(end), limit_motor);
end

fprintf('\n=== 시간별 스냅샷 ===\n');
checkTimes = 0:0.1:2;
for ct = checkTimes
    [~, idx] = min(abs(t - ct));
    line = sprintf('  t=%5.2fs: w=%8.2f', t(idx), w(idx));
    for j = 1:numel(tapNames)
        v = eval([tapNames{j} '.signals.values']);
        line = [line sprintf(' | %s=%8.4f', tapNames{j}, v(idx,1))]; %#ok<AGROW>
    end
    fprintf('%s\n', line);
end

fprintf('\n=== 판정 ===\n');
fprintf('  토크 명령이 0.25(포화)로 일정한데 w가 405에 고정 -> 모터/배터리 쪽 제한 (전기 도메인 조사 필요)\n');
fprintf('  토크 명령이 0.25보다 작음 -> PID/입력경로 쪽 문제\n');
