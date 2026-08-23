function [t, xr, vpk] = raised_cos_move(mdl, T, dx, z0, t0, tmove)
% x 축 dx [m] 직선 이동, 레이즈드 코사인 시간 프로파일.
%   피크 속도 = dx·π/(2·tmove)  — 이동시간만 바꿔 스펙 배율을 쓸어볼 수 있다.
% 반환값은 참조 궤적 (후처리 비교용).
dt = 0.01; N = round(T/dt) + 1;
t = (0:N-1)' * dt;
u = min(max((t - t0) / tmove, 0), 1);
xr = dx * 0.5 * (1 - cos(pi * u));
vpk = dx * pi / (2 * tmove);
qctest.set_path(mdl, T, [xr, zeros(N,1), z0*ones(N,1)], [], [0 0 z0; dx 0 z0]', dt);
end
