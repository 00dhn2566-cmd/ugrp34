% Parameters for quadcopter_package_delivery
% Copyright 2021-2026 The MathWorks, Inc.

% Size of the ground
planex = 12.5;           % m
planey = 8.5;            % m
planedepth = 0.2;        % m, distance from plane to the reference frame

% Battery Capacity
battery_capacity = 7.6*3;

%% Material Property
% Assuming the arm of the drone is manufractured by 3D Printing, the ideal
% material is PLA, safe, light and cheap, the only concern is its thermal
% property
rho_pla   = 1.25;            % g/cm^3 

% Measured drone mass
drone_mass = 1.2726;
%% package ground contact properties
pkgGrndStiff  = 1000;
pkgGrndDamp   = 300;
pkgGrndTransW = 1e-3;


%% Package parameters
pkgSize = [1 1 1]*0.14; % m
pkgDensity = 1/(pkgSize(1)*pkgSize(2)*pkgSize(3)); % kg/m^3

%% Propeller parameters
propeller.diameter = 0.254; % m
propeller.Kthrust  = 9.79;   % 재보정(11차): Aerodynamic Propeller 블록 공식 기준 (표준계수 0.1072 x 91.3)
propeller.Kdrag    = 0.597;  % 재보정(9차): APC 실측 토크 기준 (평형속도 검증됨)

air_rho            = 1.225;  % kg/m^3
air_temperature    = 273+25; % degK
wind_speed         = 0;      % Wind speed (m/s)

%% Leg parameters
drone_leg.Extr_Data = flipud([...
    0     0;
    0.5   0;
    1    -1;
    0.98 -1;
    0.5  -0.02;
   -0.5  -0.02;
   -0.98 -1;
   -1    -1;
   -0.5   0].*[1 1]*0.15);

drone_leg.width = 0.01;

%% Motor parameters
qc_motor.max_torque = 0.8;  % N*m
qc_motor.max_power  = 160;  % W
qc_motor.time_const = 0.02; % sec
qc_motor.efficiency = 25/30*100; % 0-100
qc_motor.efficiency_spd = 5000; % rpm
qc_motor.efficiency_trq = 0.05; % N*m
qc_motor.rotor_damping  = 1e-7; % N*m/(rad/s)

qc_max_power = qc_motor.max_power;

%% Controller parameters
filtM_position = 0.005;
kp_position    = 8;
ki_position    = 0.04;
kd_position    = 3.2;
filtD_position = 100;
pos2attitude   = 2.4;

filtM_attitude = 0.01;
kp_attitude    = -100;   % 재튜닝(11차): 플랜트 이득 음수(b=-0.0296) 실측 -> 음수 게인이 정답. pidtune+호버검증(RMS 0.56도)
ki_attitude    = -10;    % 채택(14차, §W): 활공 정상 자세오차(드래그x저중심 레버, +5~7도) 소거 + 트림 개선(roll 0.46->0.13도).
                         % 부호 음수 규약(kp와 동일). 1.8Hz 짐 흔들림에는 무효(그건 ZV 셰이퍼 담당 - traj_zv.m).
                         % 주의: anti-windup 없음 - 포화 장시간 지속 시나리오에서 와인드업 감시 필요. -30은 활공 과보상 기미로 보류.
kd_attitude    = -150;
filtD_attitude = 2000;   % 재조정(12차): 지터 3차 스윕 실측 - 2000이 1000 대비 RMS 10% 개선(0.534->0.480도).
                         % 잔여 0.48도는 게인 불변의 6.87Hz 외부 가진(모터 동역학 추정) - 게인으로 도달 가능한 바닥
limit_attitude = 800;

filtM_yaw      = 0.01;
kp_yaw         = 15;     % 재튜닝(12차): yaw 스윙 11.4도->2.5도. 스윕+지속외란 매트릭스 전지표 1위 (kp=20은 수확역전)
ki_yaw         = 1.5;    % 재튜닝(12차): 지속외란 필수 판명 - PD만으론 42~65도 영구 고착(약권한 채널), Ti=10s로 소거.
                         % 주의: anti-windup 없음 - 대형 과도 시 와인드업 역스윙 가능(고급기법 백로그)
kd_yaw         = 4;
filtD_yaw      = 100;
limit_yaw      = 20;

filtM_altitude = 0.05;
kp_altitude    = 0.5;    % 재튜닝(11차): 검증 구성
ki_altitude    = 0.1;
kd_altitude    = 0.15;   % 재조정(12차): 지터 4~5차 수사 - 고도 미분경로가 6.87Hz 가진원이었음.
filtD_altitude = 1000;   % 0.3/10000 -> 0.15/1000: 자세 RMS 0.48->0.098도(스펙 R4 통과), z 진동 2.5mm->0.2mm.
                         % 외란 감쇠 회귀는 z 돌풍/지속바람 배터리로 검증(diagnose_robust_xy.m)
limit_altitude = 10;

kp_motor       = 0.00375;
ki_motor       = 4.50000e-4;
kd_motor       = 0;
filtD_motor    = 10000;
filtSpd_motor    = 0.001;
limit_motor    = 0.25;

%% Drag coefficients
qd_drag.Cd_X = 0.35;
qd_drag.Cd_Y = 0.35;
qd_drag.Cd_Z = 0.6;
qd_drag.Roll = 0.2;
qd_drag.Pitch = 0.2;
qd_drag.Yaw = 0.2;
qd_area.YZ = 0.0875;
qd_area.XZ = 0.0900;
qd_area.XY = 0.2560;
qd_area.Roll = qd_area.XY*2;
qd_area.Pitch = qd_area.XY*2;
qd_area.Yaw = qd_area.XY;

