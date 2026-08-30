function verify_mhe_parity()
%VERIFY_MHE_PARITY  MATLAB qc_mhe_solve 가 파이썬 mhe_estimator 와 같은 답을 내나.
%
%   두 구현이 갈리면 폐루프 결과를 해석할 수 없다 — 개선이 알고리즘 덕인지 이식
%   오류 덕인지 모르게 된다. 그래서 폐루프에 심기 **전에** 이걸 통과시킨다.
%   (같은 이유로 C++ 이식은 골든 트레이스 대조를 쓴다.)
%
%   입력은 파이썬이 덤프한 고정 시나리오 하나. 창 0.25 s, 5 ms 격자, 측정 6 개,
%   참 bias 0.8 — 관측 가능성 경계 근처라 추정이 0.56 으로 덜 수렴하는 것이 정상이다
%   (H >~ sqrt(2*sigma/b) = 0.158 s 인데 여기 창이 0.25 s 로 겨우 넘는다).

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'Scripts_Data'));
f = fullfile(here, 'results', 'mhe_parity_case.json');
if ~exist(f, 'file')
    error('verify_mhe_parity: 대조 케이스 %s 없음 (파이썬에서 먼저 덤프할 것)', f);
end
c = jsondecode(fileread(f));

[p0, v0, b, dbstd, ok] = qc_mhe_solve(c.trel(:), c.acc(:), c.meas(:), ...
                                      c.mvalid(:), c.prior(:), c.w(:));

fprintf('\n===== MHE 이식 대조 =====\n');
fprintf('           %14s %14s %12s\n', 'MATLAB', '파이썬', '차이');
nm = {'p0', 'v0', 'b', 'dbstd'};
gm = [p0, v0, b, dbstd];
gp = [c.py.p0, c.py.v0, c.py.b, c.py.dbstd];
worst = 0;
for k = 1:4
    d = abs(gm(k) - gp(k));
    worst = max(worst, d);
    fprintf('  %-6s %14.9f %14.9f %12.2e\n', nm{k}, gm(k), gp(k), d);
end
fprintf('  ok = %d,  최대 차이 %.2e\n', ok, worst);
if worst < 1e-9
    fprintf('  판정: **일치** (1e-9 이내)\n');
else
    fprintf(2, '  판정: 불일치 — 폐루프에 심기 전에 고칠 것\n');
end
end
