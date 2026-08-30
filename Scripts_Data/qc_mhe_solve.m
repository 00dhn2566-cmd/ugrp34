function [p0, v0, b, dbstd, ok] = qc_mhe_solve(trel, acc, meas, mvalid, prior, w)
%QC_MHE_SOLVE  이동 지평 추정 한 번 — 3x3 가중 최소제곱 (파이썬 mhe_estimator 와 1:1).
%
%   창 시작점의 상태를 (p0, v0, b) 로 두면 창 안의 어느 시각에서
%       p(t_k) = p0 + v0*D_k + 0.5*b*D_k^2 + S_k       S_k = 측정 가속도 이중적분
%   이고 **미지수에 선형**이다. 그래서 반복 최적화가 아니라 정규방정식 한 번이면 된다.
%   이 사실이 이 물건을 1 kHz 에 올릴 수 있게 만든다 (결정론적, 할당 없음).
%
%   입력
%     trel   [n x 1] 창 시작(t0) 기준 상대 시각 [s], 오름차순
%     acc    [n x 1] 각 스텝의 **모델이 아는** 가속도 [m/s^2] (자세에서 환산)
%     meas   [n x 1] 그 시각으로 되감은 측정 위치 [m]
%     mvalid [n x 1] 그 슬롯에 측정이 있나 (0/1)
%     prior  [3 x 1] 직전 해 [p0; v0; b] — MHE 의 arrival cost
%     w      [4 x 1] 정보 가중 [1/sig_p^2; 1/sig_v^2; 1/sig_b^2; 1/sig_meas^2]
%
%   출력
%     dbstd  바이어스 표준편차 = sqrt(inv(M)(3,3)). **N 을 이 값이 정한다** —
%            모를수록 자주 풀어야 한다 (08-29 정정: 계산 예산이 아니라 정확도 예산).
%     ok     분해 성공 여부. 실패하면 호출자가 직전 해를 유지할 것.
%
%   ※ 사전(prior)이 있으므로 측정이 1 개뿐이어도 해가 나온다. tau=60 ms 에 VIO 30 Hz 면
%     창 안 측정이 2 개뿐이라 미지수(3)보다 적은데, 그 빈자리를 사전이 채운다.

n = numel(trel);
% 사전으로 시작 (M = 정보행렬, r = 정보벡터)
M = diag([w(1) w(2) w(3)]);
r = [w(1)*prior(1); w(2)*prior(2); w(3)*prior(3)];

% S_k: 측정 가속도만의 이중적분. 바이어스 몫은 미지수라 설계행렬로 간다.
% 전진 오일러 — 창 안에서 가속도가 스텝 상수로 들어오므로 사다리꼴로 바꿔도 이득이
% 없고, C++/파이썬 이식 때 한 스텝 어긋나기만 쉽다.
S = zeros(n,1); vv = 0;
for k = 1:n-1
    h = trel(k+1) - trel(k);
    S(k+1) = S(k) + vv*h + 0.5*acc(k)*h*h;
    vv = vv + acc(k)*h;
end

wm = w(4); nm = 0;
for k = 1:n
    if ~mvalid(k); continue; end
    d = trel(k);
    g = [1; d; 0.5*d*d];
    y = meas(k) - S(k);
    M = M + wm * (g*g.');
    r = r + wm * g * y;
    nm = nm + 1;
end

p0 = prior(1); v0 = prior(2); b = prior(3);
dbstd = sqrt(1/max(w(3), eps));   % 측정이 없으면 사전 그대로 = 모른다
ok = false;
if nm == 0; return; end

% 대칭 양정치 -> 촐레스키. inv 를 통째로 구하지 않는다 (수치·비용 둘 다).
[R, flag] = chol(M);
if flag ~= 0; return; end
x = R \ (R.' \ r);
Minv33 = sum((R.' \ [0;0;1]).^2);   % inv(M)(3,3) = e3' * inv(M) * e3
p0 = x(1); v0 = x(2); b = x(3);
dbstd = sqrt(max(Minv33, 0));
ok = true;
end
