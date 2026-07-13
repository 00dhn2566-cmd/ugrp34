%% err2rp(Err2P/Err2R 게인, Position Control 마스크 파라미터, 기본값 pos2attitude=2.4)를
%% 2.4 -> 1.0 -> 0.5 -> 0.2로 낮춰가며 베이스라인 호버에서 roll/y가 안정되는지 확인.
%% err2rp는 Position Control 서브시스템의 마스크 파라미터라 base workspace 변수로는
%% 못 바꾸고, set_param으로 마스크값 자체를 바꿔야 함.

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
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;  % 원래값

mdl = 'quadcopter_package_delivery';
load_system(mdl);
pc = [mdl '/Maneuver Controller/Position Control'];

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
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

err2rpVals = [2.4, 1.0, 0.5, 0.2];
labels = {'[A] 원래값 2.4', '[B] 1.0', '[C] 0.5', '[D] 0.2'};

for k = 1:numel(err2rpVals)
    set_param(pc, 'err2rp', num2str(err2rpVals(k)));
    fprintf('\n=== %s (err2rp=%.2f) ===\n', labels{k}, err2rpVals(k));
    simOut = sim(mdl);
    r = rad2deg(real_roll.signals.values(:));
    p = rad2deg(real_pitch.signals.values(:));
    y = real_y.signals.values(:);
    z = real_z.signals.values(:);
    fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
    fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', min(p), max(p), p(end));
    fprintf('  y: min=%.3f max=%.3f last=%.3f (목표=0)\n', min(y), max(y), y(end));
    fprintf('  z: last=%.4f (목표=1.0)\n', z(end));
end

fprintf('\n=== 결론 ===\n');
fprintf('  err2rp를 낮출수록 roll 발산폭/최종 y 이탈이 줄어들면 -> err2rp 과다가 위치 불안정의 핵심 원인.\n');
