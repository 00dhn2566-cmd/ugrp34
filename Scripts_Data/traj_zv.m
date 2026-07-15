function pos_s = traj_zv(t, pos, fMode, mode)
%TRAJ_ZV 잔류진동 소거 input shaper (현수하중/저중심 모드용, 2026-07-15 §W 실증)
% 기준 궤적을 임펄스열과 컨볼루션해 fMode 진동 모드의 가진을 자기 상쇄시킨다.
%   ZV  : [1/2, 1/2] @ 반주기      - 지연 T/2. 주파수 정확할 때 최대 소거
%   ZVD : [1/4, 1/2, 1/4] @ 반주기 - 지연 T. 주파수 추정 오차에 강건 (권장 후보)
% 파이프라인 위치: path_time -> traj_smoother(물리 한계) -> traj_zv(진동 상쇄) -> 컨트롤러.
%   스무더 뒤에 두는 이유: ZV는 볼록 결합(가중평균)이라 v/a/j 한계를 보존한다 -
%   순서를 바꾸면(ZV 먼저) 스무더가 임펄스 간격을 뭉개 상쇄 조건이 깨질 수 있음.
% 실증(§W): 1.80Hz, ZV 적용 시 도착 후 pitch RMS 4.26 -> 1.51도 (-65%).
%   잔여분은 모드 주파수 추정 오차 + 상시 가진 몫 - fMode 정밀 실측이 다음 과제.
% 주의: 감쇠비 0 가정 (실측 감쇠비 ~1.0이라 정당). 시작 구간은 첫 샘플 값으로 패딩.
%
% 입력: t (N,1) [s] 균일 샘플 / pos (N,C) / fMode [Hz] / mode 'zv'(기본)|'zvd'
% 출력: pos_s (N,C) 성형된 기준 (지연: zv T/2, zvd T)

if nargin < 4; mode = 'zv'; end
t = t(:);
N = numel(t);
if size(pos,1) ~= N; error('traj_zv: t와 pos 길이 불일치'); end
dt = t(2) - t(1);
if max(abs(diff(t) - dt)) > 1e-9; error('traj_zv: 균일 샘플 필요'); end
dHalf = round(1/(2*fMode)/dt);
if dHalf < 1; error('traj_zv: 샘플링이 모드 반주기보다 성김 (dt=%g, f=%g)', dt, fMode); end

delayed = @(P, d) [repmat(P(1,:), d, 1); P(1:end-d, :)];
switch lower(mode)
    case 'zv'
        pos_s = 0.5*pos + 0.5*delayed(pos, dHalf);
    case 'zvd'
        pos_s = 0.25*pos + 0.5*delayed(pos, dHalf) + 0.25*delayed(pos, 2*dHalf);
    otherwise
        error('traj_zv: mode는 zv 또는 zvd (받은 값: %s)', mode);
end
end
