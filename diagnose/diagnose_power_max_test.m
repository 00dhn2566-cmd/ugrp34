%% torque_speed_param=torque_power 모드에서는 w_t/T_t가 아니라
%% qc_motor.max_power/max_torque(=power_max/trq_max)가 실제로 쓰인다.
%% power_max를 낮춰서 평형 속도(7420.01)가 실제로 바뀌는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;
fprintf('원래 qc_motor.max_power=%g, max_torque=%g\n', qc_motor.max_power, qc_motor.max_torque);

% 절반으로 낮춰서 평형 속도가 실제로 변하는지만 빠르게 확인 (정밀한 값 X)
qc_motor.max_power = qc_motor.max_power * 0.5;
qc_max_power = qc_motor.max_power;

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

for m = 1:4
    refName = sprintf('motor%d_ref', m);
    measName = sprintf('motor%d_meas', m);
    if isfield(result, refName)
        fprintf('모터%d: ref last=%g, meas last=%g\n', m, result.(refName)(end), result.(measName)(end));
    end
end
