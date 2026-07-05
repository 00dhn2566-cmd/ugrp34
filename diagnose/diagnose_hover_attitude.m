%% 호버 테스트에서 실제 roll/pitch 각도가 얼마나 크게 튀는지(뒤집히는지) 직접 로깅.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

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

% Scope/In Bus Element8,9,11 = Chassis.roll/pitch/yaw (앞서 확인한 매핑)
scope = [mdl '/Scope'];
attSrc  = {'In Bus Element8', 'In Bus Element9', 'In Bus Element11'};
attVars = {'act_roll', 'act_pitch', 'act_yaw'};
for i = 1:numel(attSrc)
    twName = ['To Workspace ' attVars{i}];
    if isempty(find_system(scope, 'SearchDepth', 1, 'Name', twName))
        twBlk = [scope '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', attVars{i}, 'SaveFormat', 'Array');
        srcPh = get_param([scope '/' attSrc{i}], 'PortHandles');
        twPh  = get_param(twBlk, 'PortHandles');
        add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
    end
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

fprintf('=== roll/pitch/yaw (rad) 범위 ===\n');
for i = 1:numel(attVars)
    if isfield(result, attVars{i})
        v = result.(attVars{i});
        fprintf('  %s: min=%g max=%g last=%g (deg: min=%g max=%g last=%g)\n', ...
            attVars{i}, min(v), max(v), v(end), rad2deg(min(v)), rad2deg(max(v)), rad2deg(v(end)));
    else
        fprintf('  %s: 로깅 안됨\n', attVars{i});
    end
end
