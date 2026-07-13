%% 목표 위치를 t=0에 계단형으로 즉시 명령하는 대신, ramp_time초에 걸쳐
%% 서서히 접근시키면 PID의 미분 킥(derivative kick)이 사라져서 roll/y 발산이
%% 줄어드는지 확인. err2rp는 원래값(2.4) 그대로 유지해서 "궤적 스무딩만으로"
%% 효과가 있는지 독립적으로 테스트.

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
% err2rp는 원래값(2.4) 그대로 - 궤적 스무딩만의 효과를 보기 위함

dt = 0.01;
T = 5;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
startPoint = [0, 0, 0];

ramp_time = 1.0; % 초
sigMap_ramp = {'[A] 계단형(원래)', 0; '[B] 1초 램프', 1.0; '[C] 2초 램프', 2.0};

scope = [mdl '/Scope'];
tapMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
for i = 1:size(tapMap,1)
    twName = ['To Workspace ' tapMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', tapMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' tapMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

waypoints = [hoverPoint; hoverPoint + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);

for k = 1:size(sigMap_ramp,1)
    rt = sigMap_ramp{k,2};
    if rt == 0
        spline_data = repmat(hoverPoint, N, 1);
    else
        spline_data = zeros(N,3);
        for i = 1:N
            tt = timespot_spl(i);
            frac = min(tt/rt, 1.0);
            % smoothstep(3t^2-2t^3)으로 부드럽게
            frac_smooth = 3*frac^2 - 2*frac^3;
            spline_data(i,:) = startPoint + frac_smooth*(hoverPoint - startPoint);
        end
    end
    spline_yaw = zeros(N, 1);
    mws.assignin('timespot_spl', timespot_spl);
    mws.assignin('spline_data', spline_data);
    mws.assignin('spline_yaw', spline_yaw);

    fprintf('\n=== %s ===\n', sigMap_ramp{k,1});
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
fprintf('  램프 시간이 길어질수록 roll 발산/y 이탈이 줄면 -> 궤적 스텝(미분 킥)이 방아쇠 중 하나.\n');
