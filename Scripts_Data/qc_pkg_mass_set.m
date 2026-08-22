%QC_PKG_MASS_SET  짐 질량을 바꾸고 질량 의존 게인을 재계산 (parameters.m 실행 '후' 호출).
%
%   사용:  quadcopter_package_parameters;  qc_pkg_kg = 0;  qc_pkg_mass_set;
%
% parameters.m 의 질량 의존 항목만 같은 식으로 다시 푼다 (§88~186):
%   m_pkg_now -> sA_mass / sZ_mass -> 자세·고도 PID 게인
%   플랜트 쪽은 pkgDensity 로 반영 (0 kg 은 극소 밀도 웰드 — perf_battery/tune_0kg 관례)
%
% ※ 이 모델은 질량을 '추정' 하지 않는다. 설정값 m_pkg 에서 게인을 **비행 전 1회** 계산하는
%   개루프 게인 스케줄이다. 비행 중에는 안 바뀌며, 짐을 투하해도 게인은 그대로다.
%   (온라인 추정기는 없고, 오프라인 추정기 estimate_params.py 는 자동 반영 금지 규약)
%
% ※ sT/sQ 는 프로펠러 상수(Kthrust/Kdrag)에서만 오므로 질량과 무관 — 재계산 불필요.
%   ctrl_profile 기본값 precision 에서는 tri_agile(질량 삼각식)이 쓰이지 않는다.
%   agile 프로파일을 쓸 거면 kp_xy/kd_xy 도 같이 다시 풀 것.

if ~exist('qc_pkg_kg', 'var')
    error('qc_pkg_mass_set: 먼저 qc_pkg_kg [kg] 를 정의할 것');
end
if ~exist('sT', 'var')
    error('qc_pkg_mass_set: parameters.m 을 먼저 실행할 것 (sT 없음)');
end

pkgSize = [1 1 1] * 0.14;
if qc_pkg_kg <= 0
    pkgDensity = 1e-6 / (pkgSize(1)*pkgSize(2)*pkgSize(3));   % 0 kg 웰드
    m_pkg_now  = 0;
else
    pkgDensity = qc_pkg_kg / (pkgSize(1)*pkgSize(2)*pkgSize(3));
    m_pkg_now  = qc_pkg_kg;
end

sA_mass = 0.75 + (1 - 0.75) * min(m_pkg_now, 2);   % parameters.m:104
sZ_mass = 0.56 + (1 - 0.56) * min(m_pkg_now, 2);   % parameters.m:105

kp_attitude = -85    * sT * sA_mass;   % parameters.m:158
ki_attitude = -10    * sT * sA_mass;   % :163
kd_attitude = -127.5 * sT * sA_mass;   % :166
kp_altitude = 0.5    * sT * sZ_mass;   % :183
ki_altitude = 0.1    * sT * sZ_mass;   % :185
kd_altitude = 0.15   * sT * sZ_mass;   % :186

if exist('ctrl_profile','var') && strcmp(ctrl_profile, 'agile')
    tri_agile = max(0, 1 - abs(m_pkg_now - 1));    % :140
    kp_xy = 8   + 16  * tri_agile;
    kd_xy = 3.2 + 7.6 * tri_agile;
    kp_position = [kp_xy, kp_xy, 8];
    kd_position = [kd_xy, kd_xy, 3.2];
    posErrSat   = 1.2 / kp_xy;
end

fprintf(['[qc_pkg_mass_set] 짐 %.3f kg  ->  sA_mass %.4f  sZ_mass %.4f  ' ...
         '| kp_att %.2f  kp_alt %.4f\n'], m_pkg_now, sA_mass, sZ_mass, kp_attitude, kp_altitude);
