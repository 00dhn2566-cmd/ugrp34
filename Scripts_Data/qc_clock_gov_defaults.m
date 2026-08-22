%QC_CLOCK_GOV_DEFAULTS  외란 연동 속도 조속기 기본 파라미터 (base 워크스페이스).
%   설계: control_seoungjin/docs/SPEED_GOVERNOR.md
%   gov_on = 0 이 '꺼짐' — s ≡ 1 이라 구운 모델과 항등.
gov_on       = 0;                % 1이면 조속기 작동, 0이면 s≡1 (항등)
gov_rf       = 0.00;             % 0 = 문턱 없이 벗어난 양에 선형 비례 (사용자 지시)
gov_rs       = 1.00;             % 정규화 1.0 에서 s=s_min (EpsiN 이 이미 psi_stop 정규화)
gov_smin     = 0.00;             % 완전 정지 허용. 0.2 로 두면 크게 틀어진 채 계속 전진해 발산(§12.3)
gov_ws       = 0.50;             % 3차 임계감쇠 필터 대역 [rad/s] — §6 스냅 예산 3배
gov_tau_rho  = 0.20;             % rho 저역통과 [s]
gov_tau_psi  = 0.20;             % yaw 오차 저역통과 [s]
gov_psi_stop = 45 * pi / 180;    % yaw 오차 정규화 기준 [rad] — 안정성 경계 90도의 절반
