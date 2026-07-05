%% 모터 전기적 속도 제어 루프(Control1~4, kp_motor=0.00375 등)가 실제로
%% w1~w4 setpoint를 잘 따라가는지, 아니면 이 안쪽 루프 자체가 불안정해서
%% 요(yaw) 폭주의 원인이 되는지 확인.

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
T = 2;   % 짧게: 초반 발산 여부만 보면 충분
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

% Control1(모터1 속도 서보)의 ref(=w1 setpoint)/meas(실제 모터 각속도) 로깅.
% torque는 Outport(싱크) 블록이라 자기 자신의 Outport가 없으므로, 그 소스를 찾아 탭.
elec = [mdl '/Quadcopter/Electrical/Control1'];
sigSrc  = {'ref', 'meas'};
sigVars = {'motor1_ref', 'motor1_meas'};
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

% torque(Outport 블록)의 소스를 찾아서 그 지점에서 탭
torqueBlk = [elec '/torque'];
torquePh = get_param(torqueBlk, 'PortHandles');
lineH = get_param(torquePh.Inport(1), 'Line');
if lineH ~= -1
    srcPortH = get_param(lineH, 'SrcPortHandle');
    twName = 'To Workspace motor1_torque';
    if srcPortH ~= -1 && isempty(find_system(elec, 'SearchDepth', 1, 'Name', twName))
        twBlk = [elec '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'motor1_torque', 'SaveFormat', 'Array');
        twPh = get_param(twBlk, 'PortHandles');
        add_line(elec, srcPortH, twPh.Inport(1), 'autorouting', 'on');
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
fprintf('로그된 변수: %s\n', strjoin(fieldnames(result), ', '));

for i = 1:numel(sigVars)
    if isfield(result, sigVars{i})
        v = result.(sigVars{i});
        fprintf('  %s: min=%g max=%g first5=%s last5=%s\n', sigVars{i}, min(v), max(v), ...
            mat2str(v(1:min(5,end))'), mat2str(v(max(1,end-4):end)'));
    end
end
