%% Position Control/Pitch Limit이 실제로 포화 중인지 직접 확인 + 낮은 게인으로
%% Traj -> Pitch Cmd 선형화가 이번엔 0이 아닌 값이 나오는지 테스트.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

limBlk = [mdl '/Maneuver Controller/Position Control/Pitch Limit'];
fprintf('=== Pitch Limit 블록 파라미터 ===\n');
fprintf('  BlockType=%s\n', get_param(limBlk, 'BlockType'));
try
    fprintf('  UpperLimit=%s LowerLimit=%s\n', get_param(limBlk,'UpperLimit'), get_param(limBlk,'LowerLimit'));
catch
    try
        fprintf('  UpLimit=%s LowLimit=%s\n', get_param(limBlk,'UpLimit'), get_param(limBlk,'LowLimit'));
    catch e
        fprintf('  파라미터 조회 실패: %s\n', e.message);
    end
end

% 위치 + 자세 게인 모두 보수적으로 낮춤
kp_position = 0.5; ki_position = 0; kd_position = 0.2;
kp_attitude = 5;   ki_attitude = 0; kd_attitude = 2;

set_param(mdl, 'BlockReduction', 'off');

S = load(fullfile(modelDir, 'trajectory.mat'));
timespot_spl = S.timespot_spl;
spline_data  = S.spline_data;
spline_yaw   = S.spline_yaw;
waypoints    = S.waypoints';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

% Pitch Cmd 신호 자체를 로깅해서 실제로 포화(리밋에 고정)돼 있는지 직접 확인
mixer = [mdl '/Maneuver Controller/Position Control'];
twName = 'To Workspace pitchcmd_check';
if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', twName))
    twBlk = [mixer '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'pitchcmd_check', 'SaveFormat', 'Array');
    srcPh = get_param([mixer '/Pitch Limit'], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(mixer, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

vars_before = who;
simOut = sim(mdl);
vars_after = who;
new_vars = setdiff(vars_after, [vars_before; {'vars_before'}]);
for i = 1:numel(new_vars)
    v = eval(new_vars{i});
    if isnumeric(v) && strcmp(new_vars{i}, 'pitchcmd_check')
        fprintf('\npitch_cmd 범위: min=%g max=%g (마지막 20개: %s)\n', min(v), max(v), mat2str(v(max(1,end-20):end)'));
    end
end

fprintf('\n=== 낮은 게인으로 Traj->Pitch Cmd 선형화 재시도 ===\n');
io2(1) = linio([mdl '/Maneuver Controller/Position Control'], 2, 'in');
io2(2) = linio([mdl '/Maneuver Controller/Position Control'], 1, 'out');
for t = [5 15 25]
    try
        [sys2, ~] = linearize(mdl, io2, t);
        fprintf('  t=%g: size=%s, nstates=%d, D=%s\n', t, mat2str(size(sys2)), size(sys2.A,1), mat2str(sys2.D));
    catch e
        fprintf('  t=%g FAILED: %s\n', t, e.message);
    end
end
