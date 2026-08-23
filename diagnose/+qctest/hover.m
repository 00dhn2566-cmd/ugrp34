function hover(mdl, T, z0)
% 제자리 호버 궤적. z0 기본 1 m.
if nargin < 3 || isempty(z0); z0 = 1.0; end
dt = 0.01; N = round(T/dt) + 1;
hp = [0, 0, z0];
qctest.set_path(mdl, T, repmat(hp, N, 1), [], [hp; hp + [0 0 2]]', dt);
end
