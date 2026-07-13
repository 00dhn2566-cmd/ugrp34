%% Motor1 컨트롤러(Control1)의 모든 입력 포트를 소스에서 직접 태핑해서,
%% Bias Chassis를 100.98 vs 5000으로 극단적으로 바꿨을 때 ref(명령)가
%% 실제로 변하는지 확인. 변하면 값의 문제, 안 변하면 set_param이
%% 실제 신호 경로에 반영되지 않는 구조적 문제로 확정.

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

% Control1 찾기
ctrl1 = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', '^Control1$');
if isempty(ctrl1); error('Control1 못 찾음'); end
ctrl1 = ctrl1{1};
fprintf('Control1 경로: %s\n', ctrl1);

% Control1의 모든 인포트를 소스 포트에서 태핑
ph = get_param(ctrl1, 'PortHandles');
nIn = numel(ph.Inport);
fprintf('Control1 인포트 개수: %d\n', nIn);
tapNames = {};
for i = 1:nIn
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH == -1; continue; end
    srcPortH = get_param(lineH, 'SrcPortHandle');
    if srcPortH == -1; continue; end
    srcParent = get_param(srcPortH, 'Parent');
    parentSys = get_param(srcParent, 'Parent');
    varName = sprintf('c1_in%d', i);
    twName = ['To Workspace ' varName];
    if isempty(find_system(parentSys, 'SearchDepth', 1, 'Name', twName))
        twBlk = [parentSys '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', varName, 'SaveFormat', 'StructureWithTime');
        twPh = get_param(twBlk, 'PortHandles');
        add_line(parentSys, srcPortH, twPh.Inport(1), 'autorouting', 'on');
    end
    tapNames{end+1} = varName; %#ok<SAGROW>
    fprintf('  In%d <- %s (태핑됨: %s)\n', i, srcParent, varName);
end

biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];

% Prop1.w도 같이 로깅
scope = [mdl '/Scope'];
twName = 'To Workspace prop1_w';
oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
if ~isempty(oldTw); delete_block(oldTw{1}); end
twBlk = [scope '/' twName];
add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'prop1_w', 'SaveFormat', 'StructureWithTime');
srcPh = get_param([scope '/In Bus Element11'], 'PortHandles');  % Prop1.w
twPh = get_param(twBlk, 'PortHandles');
add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');

results = struct();
biasVals = {'100.9795', '5000'};
labels = {'A_bias100', 'B_bias5000'};
for k = 1:2
    set_param(biasBlk, 'Bias', biasVals{k});
    fprintf('\n=== [%s] Bias Chassis = %s ===\n', labels{k}, biasVals{k});
    fprintf('  (설정 직후 재확인: get_param Bias = %s)\n', get_param(biasBlk, 'Bias'));
    simOut = sim(mdl);
    for j = 1:numel(tapNames)
        v = eval([tapNames{j} '.signals.values']);
        v = v(:, 1); % 첫 채널만
        fprintf('  %s: min=%.4f max=%.4f last=%.4f\n', tapNames{j}, min(v), max(v), v(end));
        results.(sprintf('%s_%s', labels{k}, tapNames{j})) = v;
    end
    w = prop1_w.signals.values(:);
    fprintf('  Prop1.w: min=%.2f max=%.2f last=%.2f rad/s\n', min(w), max(w), w(end));
    results.(sprintf('%s_prop1w', labels{k})) = w;
end

fprintf('\n=== 판정 ===\n');
for j = 1:numel(tapNames)
    a = results.(sprintf('A_bias100_%s', tapNames{j}));
    b = results.(sprintf('B_bias5000_%s', tapNames{j}));
    if isequal(a, b)
        fprintf('  %s: A/B 완전 동일 -> Bias 변경이 이 신호에 반영 안 됨\n', tapNames{j});
    else
        fprintf('  %s: A/B 다름 (last: %.4f vs %.4f) -> Bias 변경 반영됨\n', tapNames{j}, a(end), b(end));
    end
end
wa = results.A_bias100_prop1w; wb = results.B_bias5000_prop1w;
if isequal(wa, wb)
    fprintf('  Prop1.w: A/B 완전 동일 -> 모터 속도가 Bias와 무관 (구조적 문제 확정)\n');
else
    fprintf('  Prop1.w: A/B 다름 (last: %.2f vs %.2f rad/s) -> Bias가 모터에 반영됨\n', wa(end), wb(end));
end
