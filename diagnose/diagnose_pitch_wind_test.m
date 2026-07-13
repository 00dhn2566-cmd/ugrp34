%% "롤만 우연히 -90도로 튄 건지, 대칭적인 시스템이라 pitch도 비슷한 외란을 주면
%% 똑같이 크게 튈 수 있는지" 검증. wind_speed가 실제 배선에 쓰이는지 먼저 확인하고,
%% 안 쓰이면(죽은 파라미터일 가능성 높음) 대신 6 DOF의 초기 pitch 각속도/각도에
%% 작은 초기 외란을 직접 주입해서 pitch가 roll처럼 크게 튀는지 확인.

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

% wind_speed가 실제로 어디 마스크 파라미터에 쓰이는지 검색
fprintf('=== wind_speed를 마스크 파라미터로 쓰는 블록 검색 ===\n');
allBlks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
found = false;
for i = 1:numel(allBlks)
    try
        mv = get_param(allBlks{i}, 'MaskValues');
        mn = get_param(allBlks{i}, 'MaskNames');
        for j = 1:numel(mv)
            if contains(mv{j}, 'wind_speed')
                fprintf('  %s : %s = %s\n', allBlks{i}, mn{j}, mv{j});
                found = true;
            end
        end
    catch
    end
end
if ~found
    fprintf('  못 찾음 - wind_speed는 죽은 파라미터로 보임. 대신 초기 각속도 외란으로 테스트.\n');
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

% 로깅: z(E2), roll(E4), pitch(E3), yaw(E5)
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

fprintf('\n=== [기준] 원본 그대로(외란 없음) ===\n');
simOut = sim(mdl);
r0 = real_roll.signals.values(:); p0 = real_pitch.signals.values(:);
fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(r0)), rad2deg(max(r0)), rad2deg(r0(end)));
fprintf('  pitch: min=%.3f max=%.3f last=%.3f deg\n', rad2deg(min(p0)), rad2deg(max(p0)), rad2deg(p0(end)));

% 초기 조건에 작은 외란을 주는 방법 탐색: 6 DOF의 Revolute Joint 초기값 또는
% Quadcopter 최상위 초기 자세. quadcopter_package_delivery의 IC 관련 변수/블록 확인.
fprintf('\n=== 초기조건(IC) 관련 후보 변수 (base workspace) ===\n');
icVars = {'q0','initial_pitch','initial_roll','init_orientation','ic_pitch','ic_roll'};
for i = 1:numel(icVars)
    if evalin('base', sprintf('exist(''%s'',''var'')', icVars{i}))
        fprintf('  %s = %s\n', icVars{i}, mat2str(evalin('base', icVars{i})));
    end
end
