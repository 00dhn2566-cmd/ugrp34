function [ok, rep] = traj_gate(t, pos, vmax, amax, doError)
%TRAJ_GATE 궤적 물리 한계 검증 게이트 (컨트롤러 입구 백스톱, 2026-07-14 스펙)
% path_time을 거치지 않은 입구(손 궤적, yaw_spin 병합, RL 직결, config 실수)용.
% 전체 시계열을 수치미분해 v/a 피크를 검사하고, 초과 시 시끄럽게 error() 한다.
% x/y는 벡터 노름(기울기 물리는 축별이 아니라 수평합), z는 별도 채널로 같은 한계 적용.
%
% 입력: t (N,1) [s] / pos (N,3) [m] / vmax, amax [m/s, m/s2] (envelope 여유율 적용치 권장)
%       doError (기본 true) - false면 error 대신 ok=false 반환 (리포트 모드)
% 출력: ok (논리) / rep 구조체 (.vxyPk .axyPk .vzPk .azPk .tol)

if nargin < 5; doError = true; end
t = t(:);
if numel(t) < 3; error('traj_gate: 샘플 3개 미만 - 궤적 아님'); end
dt1 = diff(t);
if any(dt1 <= 0); error('traj_gate: 시간축이 단조증가 아님'); end

vv = diff(pos) ./ dt1;                    % (N-1,3)
aa = diff(vv) ./ dt1(1:end-1);            % (N-2,3)

rep.vxyPk = max(sqrt(vv(:,1).^2 + vv(:,2).^2));
rep.axyPk = max(sqrt(aa(:,1).^2 + aa(:,2).^2));
rep.vzPk  = max(abs(vv(:,3)));
rep.azPk  = max(abs(aa(:,3)));
rep.tol   = 1.001;                        % 수치미분 노이즈 허용 0.1%

ok = rep.vxyPk <= vmax*rep.tol && rep.axyPk <= amax*rep.tol && ...
     rep.vzPk  <= vmax*rep.tol && rep.azPk  <= amax*rep.tol;

if ~ok && doError
    error(['traj_gate: 궤적이 물리 한계 초과 - 컨트롤러 투입 거부.\n' ...
           '  |v_xy| %.2f / 한계 %.2f m/s\n  |a_xy| %.2f / 한계 %.2f m/s2\n' ...
           '  |v_z|  %.2f / 한계 %.2f m/s\n  |a_z|  %.2f / 한계 %.2f m/s2\n' ...
           '  -> path_time 재-시간매개화 또는 traj_smoother 적용 후 재시도'], ...
           rep.vxyPk, vmax, rep.axyPk, amax, rep.vzPk, vmax, rep.azPk, amax);
end
end
