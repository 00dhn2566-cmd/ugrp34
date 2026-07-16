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
% --- 게인 정규화 앵커 (사용자 설계, 15차): 게인을 프로펠러 계수에 대한 식으로 묶음 ---
% Kthrust/Kdrag를 재실측하거나 프롭을 교체하면 루프 이득이 변하는데, 아래 스케일이
% 자동 보상해 튜닝 형상(교차주파수/위상여유)을 보존한다. 앵커 = 현 튜닝 당시 실측값
% 이므로 오늘 기준 스케일 = 정확히 1 (수치 무변화). 앵커 자체는 절대 갱신하지 말 것
% - 앵커를 새 실측값으로 바꾸면 보상이 무효가 된다 (앵커는 "튜닝했던 그 날의 값").
Kthrust_ref = 9.79;                          % 11차 재보정값 (자세/고도/모터 튜닝 기준)
Kdrag_ref   = 0.597;                         % 9차 재보정값 (yaw 튜닝 기준)
sT = Kthrust_ref / propeller.Kthrust;        % 추력 계열 게인 스케일 (현재 1)
sQ = Kdrag_ref   / propeller.Kdrag;          % 반토크(yaw) 게인 스케일 (현재 1)

% --- 물성 정규화 확장 (17차, 사용자 설계): 게인을 질량/관성모멘트 의존식으로 확장 ---
% 원리(루프 이득 보존): 자세 각가속 = 토크/I, 고도 가속 = 추력/m 이므로
%   자세 게인 ∝ I_att/Kthrust, yaw 게인 ∝ I_yaw/Kdrag, 고도 게인 ∝ m_tot/Kthrust.
%   위치 게인은 스케일 없음: 기울기→수평가속 = g·sinθ 라 질량/관성 무관
%   (내부 자세루프가 위 스케일로 정규화된다는 전제).
% 물성 출처 (17차 실측, diagnose_inertia_measure.m / Inertia Sensor, World, CoM 환산):
%   섀시(로터 제외 바디그룹) 실측 + 로터 4조 기하 추정 + 패키지 해석항을 qc_phys()가 합성.
%   합성식을 "로터 제외" 조건으로 되돌려 계산하면 실측 비행구성 관성 8.37e-3을 0.3% 내 재현 (검증됨).
%   패키지 크기/질량(pkgSize/pkgDensity)을 바꾸면 스케일이 자동 추종. 기체(섀시/암/모터)
%   변경 시에는 diagnose_inertia_measure.m 재실측 후 qc_phys 안의 섀시 상수를 갱신할 것.
% ref = 튜닝 당시(16~17차) 물성으로 고정 - 절대 갱신 금지 (앵커는 "튜닝했던 그 날의 값").
m_pkg_now = pkgSize(1)*pkgSize(2)*pkgSize(3)*pkgDensity;
[I_att_now, I_yaw_now, m_tot_now] = qc_phys(drone_mass, m_pkg_now, pkgSize);
[I_att_ref, I_yaw_ref, m_tot_ref] = qc_phys(1.2726,     1.0,      [1 1 1]*0.14);
sIa = I_att_now / I_att_ref;                 % 자세(롤/피치) 관성 스케일 (현재 1)
sIz = I_yaw_now / I_yaw_ref;                 % yaw 관성 스케일 (현재 1)
sM  = m_tot_now / m_tot_ref;                 % 총질량 스케일 (현재 1)

filtM_position = 0.005;
% 위치 게인은 물성 스케일 없음 (17차): 위치루프 플랜트 = 기울기->수평가속 = g·sinθ 라
% 질량/관성/추력계수와 무관 (자세 내부루프가 sT·sIa로 정규화된다는 전제).
kp_position    = 8;
ki_position    = 0.04;
kd_position    = 3.2;
filtD_position = 100;
pos2attitude   = 2.4;
posErrSat      = 1.2 / kp_position;  % 오차 클램프 (사용자 설계, 15차): 곱 불변식 C x kp = 1.2
                                     % -> P항 최대 명령 기울기 = 1.2 x 0.2446 = 16.8도 보존.
                                     % kp_position을 튜닝해도 클램프가 자동 연동 (현재 0.15m).
                                     % .slx의 PosErr Sat X/Y/Z가 이 변수를 참조.

filtM_attitude = 0.01;
kp_attitude    = -85 * sT * sIa;    % 미세조정(16차): 실모델 좌표하강 3라운드 수렴 - 호버 지터 0.076->0.0020도 (38배).
                              % -100 부근이 모터 동역학과 공진(지터 절벽이 -95~-90 사이 실측), -75~-85가 평탄 바닥.
                              % -85 = 바닥 중앙(절벽에서 10% 여유). 부호 음수 필수(플랜트 이득 음수 b=-0.0296).
                              % xsT: 자세 토크는 차동 추력 ∝ Kthrust - 계수 재실측 시 자동 보상 (15차)
                              % xsIa: 각가속 = 토크/I_att - 관성(패키지 포함) 변경 시 자동 보상 (17차)
ki_attitude    = -10 * sT * sIa;    % 채택(14차, §W): 활공 정상 자세오차(드래그x저중심 레버, +5~7도) 소거 + 트림 개선(roll 0.46->0.13도).
                              % 부호 음수 규약(kp와 동일). 1.8Hz 짐 흔들림에는 무효(그건 ZV 셰이퍼 담당 - traj_zv.m).
                              % 주의: anti-windup 없음 - 포화 장시간 지속 시나리오에서 와인드업 감시 필요. -30은 활공 과보상 기미로 보류.
kd_attitude    = -127.5 * sT * sIa; % 미세조정(16차): kd/kp 비율 1.5가 골짜기 바닥 (1.4/1.6 모두 열등)
filtD_attitude = 2500;   % 미세조정(16차): 1라운드에서 2500이 최선 (영향은 미미)
                         % 과거 기록: "잔여 0.48도 지터는 게인 불변" -> 16차에 반증됨. kp=-100 부근의
                         % 모터 동역학 공진이 원인이었고 kp=-85로 이탈하면 0.002도까지 소멸.
limit_attitude = 800;

filtM_yaw      = 0.01;
kp_yaw         = 15 * sQ * sIz; % 재튜닝(12차): yaw 스윙 11.4도->2.5도. 스윕+지속외란 매트릭스 전지표 1위 (kp=20은 수확역전)
                          % xsQ: yaw 권한은 반토크 ∝ Kdrag - 계수 재실측 시 자동 보상 (15차)
                          % xsIz: 각가속 = 반토크/I_yaw - 관성 변경 시 자동 보상 (17차)
ki_yaw         = 1.5 * sQ * sIz; % 재튜닝(12차): 지속외란 필수 판명 - PD만으론 42~65도 영구 고착(약권한 채널), Ti=10s로 소거.
                          % 주의: anti-windup 없음 - 대형 과도 시 와인드업 역스윙 가능(고급기법 백로그)
kd_yaw         = 4 * sQ * sIz;
filtD_yaw      = 100;
limit_yaw      = 20;

filtM_altitude = 0.05;
kp_altitude    = 0.5 * sT * sM; % 재튜닝(11차): 검증 구성. xsT: 고도 권한은 총추력 ∝ Kthrust (15차)
                                % xsM: 가속 = 추력/m_tot - 총질량(패키지 포함) 변경 시 자동 보상 (17차)
ki_altitude    = 0.1 * sT * sM;
kd_altitude    = 0.15 * sT * sM; % 재조정(12차): 지터 4~5차 수사 - 고도 미분경로가 6.87Hz 가진원이었음.
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

%% 물성 합성 함수 (17차) - 전체 기체 관성/질량을 구성요소로부터 계산 (CoM 기준)
% 섀시 상수 = Inertia Sensor 실측 (diagnose_inertia_measure.m, World 프레임, CoM 환산).
% 로터 4조 = 레볼루트 조인트 너머라 센서가 못 봄 -> 기하 추정 (X쿼드, FX450 휠베이스 450mm).
% 패키지 = 균일 직육면체 해석식. 부착면(패키지 윗면) z = -0.012m (실측 CoM -0.082 역산).
% 검증: 로터 항을 빼고 계산하면 실측 비행구성 관성(Ixx/Iyy 8.37e-3)을 0.3% 내로 재현.
function [I_att, I_yaw, m_tot] = qc_phys(m_drone, m_pkg, pkgSz)
    m_ch  = 0.9650346;                       % 섀시(로터 제외) 실측 질량 [kg]
    z_ch  = +0.0038181;                      % 섀시 CoM (몸체 원점 기준) [m]
    I_ch  = [1.488e-3, 1.538e-3, 2.399e-3];  % 섀시 CoM 기준 Ixx/Iyy/Izz [kg m^2]
    m_rot = m_drone - m_ch;                  % 로터+모터 4조 (현재 0.3076)
    r_arm = 0.225/sqrt(2);                   % X쿼드 모터 x=y 오프셋 0.159 [m]
    z_rot = +0.02;                           % 모터 높이 (플레이트 위, 추정) [m]
    z_pkg = -0.012 - pkgSz(3)/2;             % 패키지 CoM = 부착면 - 높이/2
    m_tot = m_drone + m_pkg;
    z_cg  = (m_ch*z_ch + m_rot*z_rot + m_pkg*z_pkg) / m_tot;
    Ix = I_ch(1) + m_ch*(z_ch-z_cg)^2 ...
       + m_rot*r_arm^2 + m_rot*(z_rot-z_cg)^2 ...
       + m_pkg/12*(pkgSz(2)^2+pkgSz(3)^2) + m_pkg*(z_pkg-z_cg)^2;
    Iy = I_ch(2) + m_ch*(z_ch-z_cg)^2 ...
       + m_rot*r_arm^2 + m_rot*(z_rot-z_cg)^2 ...
       + m_pkg/12*(pkgSz(1)^2+pkgSz(3)^2) + m_pkg*(z_pkg-z_cg)^2;
    I_att = (Ix + Iy)/2;
    I_yaw = I_ch(3) + m_rot*(2*r_arm^2) + m_pkg/12*(pkgSz(1)^2+pkgSz(2)^2);
end

