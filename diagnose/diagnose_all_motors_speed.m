%% 모터 1~4 전부의 ref(명령)/meas(실제 속도)를 로깅해서, kp_motor가 너무
%% 작아서 4개 다 비슷한 "자연 평형 속도"로 수렴하는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% 4개 모터가 전부 똑같은 속도(7420.01)로 수렴하는 걸 확인함 -> kp_motor가
% 너무 작아서 차등 제어가 안 걸리는지 확인하기 위해 훨씬 크게 키워본다.
kp_motor = 0.00375 * 200;
ki_motor = 0.00045 * 200;

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

fprintf('kp_motor=%g ki_motor=%g kd_motor=%g\n', kp_motor, ki_motor, kd_motor);

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
    if isfield(result, refName) && isfield(result, measName)
        r = result.(refName);
        me = result.(measName);
        fprintf('모터%d: ref last=%g, meas last=%g (차이=%g)\n', m, r(end), me(end), r(end)-me(end));
    end
end
