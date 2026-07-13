%% Environment 블록의 전체 마스크 파라미터 확인(wind 방향 관련 파라미터 찾기),
%% 그 다음 pitch 방향으로 바람을 실제로 넣어서 pitch도 roll처럼 크게 튈 수 있는지 검증.

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

envBlk = [mdl '/Environment'];
fprintf('=== Environment 블록 전체 마스크 파라미터 ===\n');
mn = get_param(envBlk, 'MaskNames');
mv = get_param(envBlk, 'MaskValues');
for i = 1:numel(mn)
    fprintf('  %s = %s\n', mn{i}, mv{i});
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
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw)
        delete_block(oldTw{1});
    end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% wind_speed를 0이 아닌 값으로 설정 (일단 크기만, 방향은 위 마스크 파라미터 확인 후 결정)
wind_speed = 3; % m/s, 임의의 미풍 수준

fprintf('\n=== wind_speed=%g m/s 적용 후 테스트 ===\n', wind_speed);
simOut = sim(mdl);
r = real_roll.signals.values(:); p = real_pitch.signals.values(:); y = real_yaw.signals.values(:);
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r)), rad2deg(max(r)), rad2deg(r(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p)), rad2deg(max(p)), rad2deg(p(end)));
fprintf('  yaw: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(y)), rad2deg(max(y)), rad2deg(y(end)));
