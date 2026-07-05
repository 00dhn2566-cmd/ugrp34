%% Motor Mixer의 yaw 부호 그룹핑(모터1&3 = '-', 모터2&4 = '+')에 맞춰
%% Propeller 2, 4의 회전방향을 반대로 설정하고, 호버 테스트로 yaw가
%% 더 이상 폭주하지 않는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% w1=Add4, w2=Add5, w3=Add7, w4=Add6 (Out Bus Element 매핑 기준) -> yaw 부호:
% Add4='-'(w1), Add5='+'(w2), Add6='-'(w4), Add7='+'(w3)
% => 모터1&4 vs 모터2&3 그룹 (이전에 2&4로 잘못 짚었던 것 수정)
for p = [2 3]
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    fprintf('설정 전 %s : direction=%s\n', blk, get_param(blk, 'direction'));
    set_param(blk, 'direction', 'sdl.enum.PropellerDirection.Negative');
    fprintf('설정 후 %s : direction=%s\n', blk, get_param(blk, 'direction'));
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

fprintf('\n=== 방향 수정 후 roll/pitch/yaw (deg) ===\n');
for i = 1:numel(attVars)
    if isfield(result, attVars{i})
        v = result.(attVars{i});
        fprintf('  %s: min=%g max=%g last=%g\n', attVars{i}, rad2deg(min(v)), rad2deg(max(v)), rad2deg(v(end)));
    end
end

if isfield(result, 'act_x1')
    fprintf('\n=== 위치 (m) ===\n');
    fprintf('  x: %g -> %g\n', result.act_x1(1), result.act_x1(end));
    fprintf('  y: %g -> %g\n', result.act_y1(1), result.act_y1(end));
    fprintf('  z: %g -> %g\n', result.act_z1(1), result.act_z1(end));
end
