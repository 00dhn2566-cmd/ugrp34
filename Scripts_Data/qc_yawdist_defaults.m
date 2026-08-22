%QC_YAWDIST_DEFAULTS  yaw 외란 적응 적분 기본 파라미터 (base 워크스페이스).
%   docs/YAW_DISTURBANCE_I.md §5 표와 같은 값. yd_gmax=1 이 '꺼짐'.
yd_gmax  = 1.0;      % 최대 적분 누적률 배율 (1 = 기능 꺼짐 = 구운 모델과 항등)
yd_e0    = 0.0349;   % rad (2.0도) 판정 시작 문턱
yd_e1    = 0.0349;   % rad (2.0도) 이만큼 더 커지면 gmax 포화 (총 4.0도)
yd_tau   = 1.0;      % s  오차 저역통과(지속성 판정) 시정수
yd_rate  = 0.05;     % rad/s  |psi_ref_dot| 이 값 초과면 슬루로 보고 게이트 닫음
yd_relax = 0.5;      % 1/s  해제 시 하강 기울기 제한 (상승은 무제한)
yd_taud  = 0.02;     % s  참조 yaw 미분 필터 (s/(taud s+1))
