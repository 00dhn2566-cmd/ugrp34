%% Retune the 5 PID Compensator blocks in Maneuver Controller for the FX450 CAD
% (Position Control, Pitch/Roll/Thrust/Yaw) using Control System Toolbox's
% systune. Reference(3)/actual(3) position (x,y,z) 스칼라 신호 3쌍을 io 포인트로
% 잡는다 (전체 버스 폭이 안 맞아 나던 "9 vs 35" 에러 회피).
% - 레퍼런스: Scope/Demux의 outport 1/2/3 (des_x1/y1/z1과 동일한 신호)
% - 실제값:   Scope/In Bus Element,1,2 (Chassis.px/py/pz, act_x1/y1/z1과 동일한 신호)

addpath('Scripts_Data');
addpath('Models');
addpath('Libraries');
addpath(genpath('CAD'));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 선형화 기준이 될 operating trajectory 로드 (있으면 그대로 사용)
if isfile('trajectory.mat')
    S = load('trajectory.mat');
    timespot_spl = S.timespot_spl;
    spline_data  = S.spline_data;
    spline_yaw   = S.spline_yaw;
    waypoints    = S.waypoints';
else
    [waypoints, timespot_spl, spline_data, spline_yaw, ~] = quadcopter_package_select_trajectory(1);
end
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

% sim()은 이 스크립트의 base workspace를 그대로 보고 도는데, slTuner의 내부 배치
% 선형화(compileForLinearization)는 별도 컨텍스트라 base workspace의 waypoints/
% wayp_path_vis 등을 못 찾아 "waypoints를 찾을 수 없음" 에러가 연쇄적으로 났다.
% 모델 자체의 Model Workspace에 넣어서 항상 보이게 한다.
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

% Control Pitch/Roll/Thrust/Yaw 내부의 P/I/D 필드("attitude_kp" 등)는 그 자체 값이
% 아니라, 부모인 "Altitude and  YPR Control" 마스크가 노출하는 마스크 파라미터다
% (attitude_kp=kp_attitude, yaw_kp=kp_yaw, altitude_kp=kp_altitude 식으로 대입됨 —
% pitch/roll이 attitude_* 게인을 공유). 그래서 Control Pitch 등 개별 블록을 타겟팅하면
% systune이 실제 조정 가능한 소스를 못 찾아 "조정 가능한 파라미터 없음" 에러가 났다.
% 부모 마스크 서브시스템 자체를 타겟으로 하면 attitude_kp/ki/kd, yaw_kp/ki/kd,
% altitude_kp/ki/kd 9개가 한번에 마스크 파라미터로 노출되어 튜닝 가능해진다.
tunedBlocks = {
    'quadcopter_package_delivery/Maneuver Controller/Position Control/PID Controller'
    'quadcopter_package_delivery/Maneuver Controller/Altitude and  YPR Control'
};

io(1) = linio([mdl '/Scope/Demux'], 1, 'in');  % des_x (reference x)
io(2) = linio([mdl '/Scope/Demux'], 2, 'in');  % des_y (reference y)
io(3) = linio([mdl '/Scope/Demux'], 3, 'in');  % des_z (reference z)
io(4) = linio([mdl '/Scope/In Bus Element'],  1, 'out'); % act_x (Chassis.px)
io(5) = linio([mdl '/Scope/In Bus Element1'], 1, 'out'); % act_y (Chassis.py)
io(6) = linio([mdl '/Scope/In Bus Element2'], 1, 'out'); % act_z (Chassis.pz)

ST = slTuner(mdl, tunedBlocks, io);
% 모델에 이산(z=0) 적분기가 섞여 있어 기본 'zoh' 변환으로는 정확한 선형화가
% 불가능하다는 에러가 나서, tustin(bilinear) 변환으로 바꿔준다.
ST.Options.RateConversionOptions.Method = 'tustin';
% tunedBlocks 두 개(Position Control/PID Controller, Altitude and YPR Control)의
% 포트 짧은 이름이 겹쳐서 "InputName은 채널마다 하나의 이름을 지정해야 함" 에러가
% 났던 것 -> 전체 블록 경로로 라벨링해서 이름을 유일하게 만든다.
ST.Options.UseFullBlockNameLabels = 'on';

pts = getPoints(ST);
disp(pts);
refNames = pts(1:3);  % des_x/y/z
actNames = pts(4:6);  % act_x/y/z

% 궤적 총 시간(34s 근방)에 맞춘 대략적인 정착시간 요구조건: 5초 내 정착, 정상상태 오차 0
Req = TuningGoal.Tracking(refNames, actNames, 5, 0, 1);

opt = systuneOptions('Display', 'iter');
[ST_tuned, fSoft, ~] = systune(ST, Req, opt);

fprintf('\n=== Tuned soft goal value: %.4f (want < 1) ===\n\n', fSoft);

for i = 1:numel(tunedBlocks)
    fprintf('--- %s ---\n', tunedBlocks{i});
    showTunable(ST_tuned, tunedBlocks{i});
end

save('tuned_pid.mat', 'ST_tuned', 'fSoft', 'tunedBlocks');
fprintf('\nSaved tuned controller object to tuned_pid.mat\n');
