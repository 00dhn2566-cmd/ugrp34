%% "Altitude and YPR Control"의 입력 포트 이름("m","Ref" 등)이 "Position Control"의
%% 포트 이름과 겹쳐서 InputName 충돌이 나는지 확인 -> 포트 이름을 유일하게 바꿔서 재시도.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
posCtrl = [mdl '/Maneuver Controller/Position Control'];

fprintf('=== YPR 내부 Inport/Outport 블록 이름 ===\n');
inports = find_system(ypr, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Inport');
outports = find_system(ypr, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Outport');
for i = 1:numel(inports)
    fprintf('  Inport: %s\n', get_param(inports{i}, 'Name'));
end
for i = 1:numel(outports)
    fprintf('  Outport: %s\n', get_param(outports{i}, 'Name'));
end

fprintf('\n=== Position Control 내부 Inport/Outport 블록 이름 ===\n');
inports2 = find_system(posCtrl, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Inport');
outports2 = find_system(posCtrl, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Outport');
for i = 1:numel(inports2)
    fprintf('  Inport: %s\n', get_param(inports2{i}, 'Name'));
end
for i = 1:numel(outports2)
    fprintf('  Outport: %s\n', get_param(outports2{i}, 'Name'));
end

% 겹치는 이름("m","Ref" 등) 확인 후, YPR 쪽 포트 이름에 접두어를 붙여 유일하게 변경
fprintf('\n=== YPR 포트 이름에 접두어(YPR_) 붙여서 유일하게 변경 ===\n');
for i = 1:numel(inports)
    oldName = get_param(inports{i}, 'Name');
    newName = ['YPR_' oldName];
    set_param(inports{i}, 'Name', newName);
    fprintf('  %s -> %s\n', oldName, newName);
end
for i = 1:numel(outports)
    oldName = get_param(outports{i}, 'Name');
    newName = ['YPR_' oldName];
    set_param(outports{i}, 'Name', newName);
    fprintf('  %s -> %s\n', oldName, newName);
end

if isfile(fullfile(modelDir, 'trajectory.mat'))
    S = load(fullfile(modelDir, 'trajectory.mat'));
    timespot_spl = S.timespot_spl;
    spline_data  = S.spline_data;
    spline_yaw   = S.spline_yaw;
    waypoints    = S.waypoints';
else
    [waypoints, timespot_spl, spline_data, spline_yaw, ~] = quadcopter_package_select_trajectory(1);
end
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

io(1) = linio([mdl '/Scope/Demux'], 1, 'in');
io(2) = linio([mdl '/Scope/In Bus Element'], 1, 'out');

fprintf('\n=== 이름 변경 후 재시도: Altitude and YPR Control 단독 ===\n');
try
    ST = slTuner(mdl, {ypr}, io);
    ST.Options.RateConversionOptions.Method = 'tustin';
    pts = getPoints(ST);
    Req = TuningGoal.Tracking(pts{1}, pts{2}, 1, 0, 1);
    [~, fSoft] = systune(ST, Req, systuneOptions('Display','off'));
    fprintf('  OK fSoft=%g\n', fSoft);
catch e
    fprintf('  FAILED: %s\n', e.message);
end
