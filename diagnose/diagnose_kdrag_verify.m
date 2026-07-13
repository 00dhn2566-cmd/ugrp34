%% Kdrag(=kp_const/(2*pi))가 실제 1045 프로펠러 기준으로 맞는지, "공식을 손으로 재구성"하지
%% 않고 직접 실측 방식으로 검증. torque_speed_param=torque_power 모드에서 평형속도는
%% "가용 파워(max_power) = 드래그 파워(Kdrag가 결정)"인 지점이므로,
%% max_power를 낮춰서 평형속도를 APC 실측 데이터 범위(1000~15000 RPM)까지 끌어내린 뒤
%% 그 지점에서 Q_model = max_power/w_eq 를 구해서 APC의 실제 Q_real(같은 RPM)과 비교.

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
twName = 'To Workspace prop1w';
oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
if ~isempty(oldTw); delete_block(oldTw{1}); end
twBlk = [scope '/' twName];
add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'prop1w', 'SaveFormat', 'StructureWithTime');
srcPh = get_param([scope '/In Bus Element11'], 'PortHandles');
twPh  = get_param(twBlk, 'PortHandles');
add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');

% max_power를 낮춰서 평형속도를 APC 실측범위(약 1000~1500 rad/s, RPM 9500~14300)로 끌어내림
qc_motor.max_power = 0.677; % W, (기존 160W -> speed~1200rad/s 목표로 speed~max_power^(1/3) 관계에서 역산)
fprintf('=== max_power=%.4gW 로 낮춰서 평형속도 확인 ===\n', qc_motor.max_power);
simOut = sim(mdl);
w = prop1w.signals.values(:);
w_eq = w(end);
rpm_eq = w_eq * 60 / (2*pi);
Q_model = qc_motor.max_power / w_eq;  % 평형점: 가용파워=드래그파워 라고 가정
fprintf('  평형속도: %.2f rad/s = %.1f RPM\n', w_eq, rpm_eq);
fprintf('  Q_model(모델의 드래그 토크) = max_power/w_eq = %.6f N*m\n', Q_model);

% 실제 APC 10x4.5 데이터 (apcprop.com PER3_10x45MR.dat, static V=0)
apc_rpm    = [1000 2000 3000 4000 5000 6000 7000 8000 9000 10000 11000 12000 13000 14000 15000];
apc_P_watt = [0.285 2.083 6.734 15.553 29.857 50.979 80.283 119.174 169.112 231.623 308.316 400.904 511.224 641.268 793.219];
apc_omega = apc_rpm * 2*pi/60;
apc_Q = apc_P_watt ./ apc_omega;

Q_real_at_eq = interp1(apc_rpm, apc_Q, rpm_eq, 'linear', 'extrap');
fprintf('  같은 RPM(%.0f)에서 실제 APC 10x4.5 Q_real(보간) = %.6f N*m\n', rpm_eq, Q_real_at_eq);
fprintf('  비율 Q_real/Q_model = %.2f배\n', Q_real_at_eq / Q_model);
fprintf('  => Kdrag를 이 비율만큼 키워야 함: %.6f -> %.6f\n', propeller.Kdrag, propeller.Kdrag * (Q_real_at_eq/Q_model));

fprintf('\n=== 참고: c_Q(=Q/w^2, rad/s 기준) 비교 ===\n');
cQ_model = Q_model / w_eq^2;
cQ_real  = Q_real_at_eq / w_eq^2;
fprintf('  cQ_model=%.4e, cQ_real=%.4e, 비율=%.2f\n', cQ_model, cQ_real, cQ_real/cQ_model);
