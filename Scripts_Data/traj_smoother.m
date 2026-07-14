function [pos_s, info] = traj_smoother(t, pos, vmax, amax, jmax)
%TRAJ_SMOOTHER min/max 도달가능성 포락선 명령 성형기 (사용자 설계 2026-07-14)
% 성형된 기준의 스텝 변위 d를 매 샘플 아래 구간에 클램프한다 (사용자 식의
% 후방차분 일관형 - 연속식의 1/2, 1/6 계수는 차분 상태 정의에 흡수됨):
%   상한: min( v·dt + a·dt² + jmax·dt³,  v·dt + amax·dt²,  +vmax·dt )
%   하한: max( v·dt + a·dt² - jmax·dt³,  v·dt - amax·dt²,  -vmax·dt )
% + 정지거리 4항: 현 상태(v,a)에서 최대저크 스윙+정속제동(ab=0.8·amax)으로
%   정지하는 데 필요한 정확한 2단 정지거리가 전방 잔여 극값 g를 넘으면
%   제동 모드(d=물리 최대 제동)로 전환. sqrt 근사 대신 정확식을 쓰는 이유:
%   sqrt 법칙은 저크 천이 시간을 예측 못해 45cm 오버슈트 실측 (12차 실험).
%   g는 순간 기준이 아니라 "미래 기준의 전방 극값" (running max/min) —
%   순간 기준으로 잡으면 정상 입력에도 동적 랙 발생. 스트리밍(C코드) 이식
%   시엔 lookahead 구간 극값으로 대체할 것.
%
% 핵심 원칙 (HANDOFF_PATH_TO_CONTROLLER.md 스펙):
%  1) v, a는 성형기 내부 상태이되 반드시 "출력의 후방차분"으로 정의.
%     v_k=(r_k-r_{k-1})/dt, a_k=(v_k-v_{k-1})/dt. 저크 적분으로 병렬 전파하면
%     상태-출력 괴리가 누적돼 한계 사이클 발생 (0.37m 개입 + 미정착 실측).
%     드론 측정값 사용도 금지 (피드백 성형으로 변질).
%  2) 한계는 envelope 실측(2.5/2.5)보다 깎은 값 권장.
%  3) 각 축 독립 적용 (사용자 확정. 게이트 traj_gate가 xy 노름으로 이중 검사).
%  4) 하한이 무감쇠 활공의 정식 해결책: 순간정지 요구 → 물리 제동만 허용.
%
% 무개입 보장: 입력의 후방차분 v/a/j가 전 구간 한계 이내이고 감속이 ab
% 이내면 출력 == 입력 (검증: 정상 궤적 개입 < 2mm, path_time 정품이 이 경우).
%
% 입력: t (N,1) / pos (N,C) / vmax, amax, jmax
% 출력: pos_s (N,C) 성형된 기준 / info 구조체 (.vPk .aPk .jPk .maxDev 열별)

t = t(:);
N = numel(t);
if size(pos,1) ~= N; error('traj_smoother: t와 pos 길이 불일치'); end
C = size(pos,2);
pos_s = pos;
ab = 0.8 * amax;    % 제동 정속 가속 (저크 천이 마진 20%)
EPS_G = 0.002;      % 제동 트리거 데드밴드 [m] - 종점 수렴부 채터 방지

info.vPk = zeros(1,C); info.aPk = zeros(1,C); info.jPk = zeros(1,C);
info.maxDev = zeros(1,C);

for ax = 1:C
    p = pos(:,ax);
    % 전방 극값 (뒤에서부터 running max/min) - 정지거리 트리거의 목표점
    fwdMax = p; fwdMin = p;
    for k = N-1:-1:1
        fwdMax(k) = max(p(k), fwdMax(k+1));
        fwdMin(k) = min(p(k), fwdMin(k+1));
    end

    r = p(1); v = 0; a = 0;
    out = p;
    mode = 0;   % 0 자유추종 / +1 전진제동 / -1 후진제동
    for k = 2:N
        dt = t(k) - t(k-1);
        up3 = min([ v*dt + a*dt^2 + jmax*dt^3,  v*dt + amax*dt^2,  vmax*dt ]);
        lo3 = max([ v*dt + a*dt^2 - jmax*dt^3,  v*dt - amax*dt^2, -vmax*dt ]);
        gUp = max(fwdMax(k) - r, 0);
        gDn = max(r - fwdMin(k), 0);
        dsF = stop_dist(v, a, ab, jmax);
        dsB = stop_dist(-v, -a, ab, jmax);
        if mode == 1 && (v <= 0 || dsF <= 0.85*gUp); mode = 0; end
        if mode == -1 && (v >= 0 || dsB <= 0.85*gDn); mode = 0; end
        if mode == 0 && dsF > gUp + EPS_G; mode = 1; end
        if mode == 0 && dsB > gDn + EPS_G; mode = -1; end
        if mode == 1
            d = lo3;                              % 물리 최대 전진제동
        elseif mode == -1
            d = up3;                              % 물리 최대 후진제동
        else
            d = min(max(p(k) - r, lo3), up3);     % 자유추종
        end
        % 상태 = 출력의 후방차분 (원칙 1)
        r = r + d;
        vN = d / dt;
        a = (vN - v) / dt;
        v = vN;
        out(k) = r;
    end
    pos_s(:,ax) = out;

    dv = diff(out) ./ diff(t);
    da = diff(dv) ./ diff(t(1:end-1));
    dj = diff(da) ./ diff(t(1:end-2));
    info.vPk(ax) = max(abs(dv));
    info.aPk(ax) = max([abs(da); 0]);
    info.jPk(ax) = max([abs(dj); 0]);
    info.maxDev(ax) = max(abs(out - p));
end
end

function ds = stop_dist(v, a, ab, jmax)
% 전진(v>0) 정지거리: 최대저크로 a를 -ab까지 스윙(1단) 후 정속 -ab 제동(2단).
% 스윙 중 v가 먼저 0이 되면 1단 도중 정지점까지만 적분.
if v <= 0; ds = 0; return; end
t1 = max((a + ab)/jmax, 0);
v1 = v + a*t1 - jmax*t1^2/2;
if v1 <= 0
    ts = (a + sqrt(a^2 + 2*jmax*v)) / jmax;
    ds = max(v*ts + a*ts^2/2 - jmax*ts^3/6, 0);
else
    d1 = v*t1 + a*t1^2/2 - jmax*t1^3/6;
    ds = d1 + v1^2/(2*ab);
end
end
