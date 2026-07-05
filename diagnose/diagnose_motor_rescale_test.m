%% Motor 1~4의 w_t/w_eff_vec(속도축)를 실제 FX450 모터(A2212 1000KV @ 3S,
%% 무부하 최대 ~1160 rad/s) 기준으로 축소해서 호버 테스트.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% A2212 1000KV @ 3S(11.1V) 무부하 최대 rpm = 1000*11.1 = 11100 rpm = 1162.4 rad/s
% 기존 w_t 최대치(8000)를 이 값으로 스케일링 (비율 적용, 토크 배열은 유지)
scale = 1162.4 / 8000;
fprintf('스케일 비율 = %g\n', scale);
for p = 1:4
    mBlk = sprintf('%s/Quadcopter/Electrical/Motor %d', mdl, p);
    old_wt = get_param(mBlk, 'w_t');
    old_weff = get_param(mBlk, 'w_eff_vec');
    new_wt = mat2str([0, 3750, 7500, 8000] * scale);
    new_weff = mat2str([-8000, -4000, 0, 4000, 8000] * scale);
    set_param(mBlk, 'w_t', new_wt);
    set_param(mBlk, 'w_eff_vec', new_weff);
    fprintf('Motor %d: w_t %s -> %s\n', p, old_wt, new_wt);
end

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

for m = 1:4
    elec = sprintf('%s/Quadcopter/Electrical/Control%d', mdl, m);
    sigSrc  = {'ref', 'meas'};
    sigVars = {sprintf('motor%d_ref', m), sprintf('motor%d_meas', m)};
    for i = 1:numel(sigSrc)
        twName = ['To Workspace ' sigVars{i}];
        if isempty(find_system(elec, 'SearchDepth', 1, 'Name', twName))
            twBlk = [elec '/' twName];
            add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigVars{i}, 'SaveFormat', 'Array');
            srcPh = get_param([elec '/' sigSrc{i}], 'PortHandles');
            twPh  = get_param(twBlk, 'PortHandles');
            add_line(elec, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
        end
    end
end

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

fprintf('\n=== 모터 ref/meas ===\n');
for m = 1:4
    refName = sprintf('motor%d_ref', m);
    measName = sprintf('motor%d_meas', m);
    if isfield(result, refName)
        fprintf('모터%d: ref last=%g, meas last=%g\n', m, result.(refName)(end), result.(measName)(end));
    end
end

fprintf('\n=== roll/pitch/yaw (deg) ===\n');
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
