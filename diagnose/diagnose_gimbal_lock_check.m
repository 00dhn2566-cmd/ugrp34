%% "요(yaw)가 실제로 빠르게 도는가, 아니면 pitch가 +-90도 근처를 지나면서
%% 오일러각(X-Y-Z extrinsic) 짐벌락 때문에 nYaw 계산 자체가 튀는가"를 확인.
%% Scope의 각 In Bus Element가 실제로 어떤 Element(roll/pitch/yaw/nRoll/nPitch/nYaw)를
%% 참조하는지 먼저 출력해서 확인(과거 세션에서 애매했던 부분), 그 다음
%% sim_time과 함께 pitch(wrapped)/nYaw를 동시에 로깅해서 nYaw가 가장 빠르게
%%튀는 시점의 pitch 값을 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.05; ki_altitude = 0;    kd_altitude = 0.05;
kp_position = 1;    ki_position = 0;    kd_position = 0.5;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

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

% (0) Scope의 In Bus Element들이 실제로 어떤 Element를 참조하는지 확인
scope = [mdl '/Scope'];
fprintf('=== Scope In Bus Element 매핑 확인 ===\n');
scopeElems = find_system(scope, 'SearchDepth', 1, 'BlockType', 'BusSelector');
inBusBlocks = find_system(scope, 'SearchDepth', 1, 'RegExp', 'on', 'Name', '^In Bus Element');
for i = 1:numel(inBusBlocks)
    try
        el = get_param(inBusBlocks{i}, 'Element');
    catch
        el = '(N/A)';
    end
    fprintf('  %s : Element=%s\n', inBusBlocks{i}, el);
end

% (1) 위에서 실제 이름을 확인한 뒤, "pitch"(wrapped)와 "nYaw"(unwrap 누적)를 찾아서 탭
targetElems = {'pitch', 'nYaw', 'yaw', 'nPitch'};
tapVars = {'pitch_w', 'nyaw_w', 'yaw_w', 'npitch_w'};
for i = 1:numel(inBusBlocks)
    try
        el = get_param(inBusBlocks{i}, 'Element');
    catch
        continue
    end
    idx = find(strcmp(targetElems, el), 1);
    if ~isempty(idx)
        twName = ['To Workspace ' tapVars{idx}];
        if isempty(find_system(scope, 'SearchDepth', 1, 'Name', twName))
            twBlk = [scope '/' twName];
            add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', tapVars{idx}, 'SaveFormat', 'Array');
            srcPh = get_param(inBusBlocks{i}, 'PortHandles');
            twPh  = get_param(twBlk, 'PortHandles');
            add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
        end
    end
end

% (2) 같은 솔버 스텝마다 시간을 찍는 Clock 추가 (연속 상태라 dt=0.01보다 훨씬 촘촘할 수 있음)
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Sim Time Clock'))
    add_block('simulink/Sources/Clock', [mdl '/Sim Time Clock']);
    add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace sim_time'], ...
        'VariableName', 'sim_time', 'SaveFormat', 'Array');
    clockPh = get_param([mdl '/Sim Time Clock'], 'PortHandles');
    twPh    = get_param([mdl '/To Workspace sim_time'], 'PortHandles');
    add_line(mdl, clockPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

vars_before = who;
simOut = sim(mdl);
vars_after = who;
new_vars = setdiff(vars_after, [vars_before; {'vars_before'}]);
result = struct();
for i = 1:numel(new_vars)
    v = eval(new_vars{i});
    if isnumeric(v)
        result.(new_vars{i}) = v;
    end
end

fprintf('\n=== 로깅된 신호 요약 ===\n');
allVars = [tapVars, {'sim_time'}];
for i = 1:numel(allVars)
    if isfield(result, allVars{i})
        v = result.(allVars{i});
        fprintf('  %s: n=%d min=%g max=%g last=%g\n', allVars{i}, numel(v), min(v), max(v), v(end));
    else
        fprintf('  %s: 로깅 안됨\n', allVars{i});
    end
end

% (3) nYaw(혹은 yaw)의 순간 변화율이 가장 큰 시점을 찾아서, 그 시점의 pitch(wrapped) 값 확인
if isfield(result, 'nyaw_w') && isfield(result, 'sim_time')
    t = result.sim_time;
    yv = result.nyaw_w;
    n = min(numel(t), numel(yv));
    t = t(1:n); yv = yv(1:n);
    dydt = diff(yv) ./ diff(t);
    [maxRate, idx] = max(abs(dydt));
    fprintf('\n=== nYaw 순간 변화율(rad/s) 최대 지점 ===\n');
    fprintf('  t=%g, |dYaw/dt|=%g rad/s (%g deg/s)\n', t(idx), maxRate, rad2deg(maxRate));
    if isfield(result, 'pitch_w')
        pv = result.pitch_w;
        pn = min(numel(t), numel(pv));
        % 가장 가까운 시간 인덱스로 pitch 값 조회 (배열 길이가 다를 수 있어 보간)
        pt = (0:numel(pv)-1)' * (t(end)/(numel(pv)-1));
        pAtSpike = interp1(pt, pv, t(idx), 'linear', 'extrap');
        fprintf('  같은 시점의 pitch(wrapped) = %g rad (%g deg)\n', pAtSpike, rad2deg(pAtSpike));
    end
    fprintf('\n=== pitch가 +-80도를 넘는 구간이 있는지 ===\n');
    if isfield(result, 'pitch_w')
        pv = result.pitch_w;
        overIdx = find(abs(rad2deg(pv)) > 80);
        if isempty(overIdx)
            fprintf('  없음 (pitch max abs = %g deg)\n', max(abs(rad2deg(pv))));
        else
            fprintf('  있음: %d개 샘플, 최대 abs pitch = %g deg\n', numel(overIdx), max(abs(rad2deg(pv))));
        end
    end
end
