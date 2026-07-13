%% 모터 PID 출력이 0.25(limit_motor)에 완전 포화된 것이 확정됨.
%% limit_motor를 작은 단계(0.3/0.4/0.6)로 올리면서 모터 속도/z가 어떻게 변하는지 확인.
%% (1.0으로 한 번에 올리면 전기 솔버가 죽었으므로 점진적으로.)
%% 배터리 SOC와 모터 전류도 같이 로깅해서 전기 쪽 제한 여부도 확인.

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
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

propeller.Kdrag = 4.222841;
assignin('base', 'propeller', propeller);
pkgMass = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
totalMass = drone_mass + pkgMass;
T_need = totalMass * 9.81 / 4;
n_hover = sqrt(T_need / (propeller.Kthrust * air_rho * propeller.diameter^4));
biasBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem/Bias Chassis'];
set_param(biasBlk, 'Bias', num2str(n_hover, '%.4f'));

dt = 0.01;
T = 3;
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

set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element11','prop1_w'; 'In Bus Element17','mot1_i'; 'In Bus Element29','batt_soc'};
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

% 0.8 = qc_motor.max_torque(모터 물리 최대 토크)와 동일. 2.0은 사실상
% "PID단 제한 제거"(모터 자체 envelope만 남음)에 해당.
limitVals = [0.4, 0.8, 2.0];
for k = 1:numel(limitVals)
    limit_motor = limitVals(k);
    fprintf('\n=== [%d] limit_motor = %.2f ===\n', k, limit_motor);
    try
        simOut = sim(mdl);
        z = real_z.signals.values(:);
        r = rad2deg(real_roll.signals.values(:));
        w = prop1_w.signals.values(:);
        ii = mot1_i.signals.values(:);
        soc = batt_soc.signals.values(:);
        fprintf('  z: min=%.4f max=%.4f last=%.4f (목표=1.0)\n', min(z), max(z), z(end));
        fprintf('  Prop1.w: min=%.2f max=%.2f last=%.2f rad/s (호버 필요 ~634)\n', min(w), max(w), w(end));
        fprintf('  roll: min=%.3f max=%.3f last=%.3f deg\n', min(r), max(r), r(end));
        fprintf('  Mot1 전류: min=%.2f max=%.2f last=%.2f A\n', min(ii), max(ii), ii(end));
        fprintf('  Battery SOC: min=%.4f last=%.4f\n', min(soc), soc(end));
    catch e
        fprintf('  *** 시뮬레이션 실패: %s\n', e.message(1:min(200,end)));
    end
end

fprintf('\n=== 판정 ===\n');
fprintf('  limit을 올릴수록 w가 오르면 -> limit_motor가 유일한 병목 (필요한 한계값 역산 가능)\n');
fprintf('  w가 405 근처에 그대로면 -> 배터리/드라이브 등 다른 전기적 제한이 진짜 병목\n');
