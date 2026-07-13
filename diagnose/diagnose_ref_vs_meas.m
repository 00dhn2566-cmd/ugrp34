%% 모터1의 실제 명령(ref)과 달성값(meas)을 같이 로깅해서,
%% "명령 자체가 낮은 것"인지 "명령은 다른데 물리적으로 405 rad/s에 눌린 것"인지 구분.
%% 두 가지 다른 Bias Chassis 값(700 vs 100.98)으로 비교해서 ref가 실제로 다른지도 확인.

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
kp_position = 1;    ki_position = 0;    kd_position = 0.5;

mdl = 'quadcopter_package_delivery';
load_system(mdl);
propeller.Kdrag = 4.222841;
assignin('base', 'propeller', propeller);

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

% Control1의 ref/meas를 로깅 (세션3에서 확인된 경로 패턴 재사용)
ctrl1 = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', '^Control1$');
if isempty(ctrl1)
    error('Control1 블록을 못 찾음');
end
ctrl1 = ctrl1{1};
fprintf('Control1 경로: %s\n', ctrl1);

% ref, meas 태핑 (소스가 있는 서브시스템 안에 To Workspace를 놓음)
tapNames = {'ref', 'meas'};
for i = 1:numel(tapNames)
    srcBlk = [ctrl1 '/' tapNames{i}];
    inPh = get_param(srcBlk, 'PortHandles');
    lineH2 = get_param(inPh.Inport(1), 'Line');
    srcPortH = get_param(lineH2, 'SrcPortHandle');
    parentSys = get_param(srcPortH, 'Parent');
    parentSys = get_param(parentSys, 'Parent');  % 블록이 아니라 그 블록이 속한 서브시스템
    twName = ['To Workspace m1_' tapNames{i}];
    if isempty(find_system(parentSys, 'SearchDepth', 1, 'Name', twName))
        twBlk = [parentSys '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', ['m1_' tapNames{i}], 'SaveFormat', 'StructureWithTime');
        twPh = get_param(twBlk, 'PortHandles');
        add_line(parentSys, srcPortH, twPh.Inport(1), 'autorouting', 'on');
    end
end

fprintf('\n=== [A] Bias Chassis=700(원래값)로 ref/meas 비교 ===\n');
simOut = sim(mdl);
refA = m1_ref.signals.values(:); measA = m1_meas.signals.values(:);
fprintf('  ref : min=%.2f max=%.2f last=%.2f\n', min(refA), max(refA), refA(end));
fprintf('  meas: min=%.2f max=%.2f last=%.2f\n', min(measA), max(measA), measA(end));

biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', '100.98');
fprintf('\n=== [B] Bias Chassis=100.98(재계산값)로 ref/meas 비교 ===\n');
simOut = sim(mdl);
refB = m1_ref.signals.values(:); measB = m1_meas.signals.values(:);
fprintf('  ref : min=%.2f max=%.2f last=%.2f\n', min(refB), max(refB), refB(end));
fprintf('  meas: min=%.2f max=%.2f last=%.2f\n', min(measB), max(measB), measB(end));

fprintf('\n=== 결론 ===\n');
if isequal(refA, refB)
    fprintf('  ref가 A/B 완전히 동일함 -> Bias Chassis 변경이 ref 자체에 반영이 안 됨(다른 원인).\n');
else
    fprintf('  ref는 다름(A last=%.2f vs B last=%.2f) -> 명령은 바뀌는데 meas가 %.2f 근처로 눌린다면 물리적 한계.\n', refA(end), refB(end), measA(end));
end
