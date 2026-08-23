function set_path(mdl, T, spline_data, spline_yaw, waypoints, dt)
% 모델 워크스페이스에 궤적 변수를 심는다.
%
% ⚠ base 가 아니라 **모델 워크스페이스**여야 한다 — Lookup Table 블록이 base 를
%   보지 않는다 (CLAUDE.md 게차 항목).
if nargin < 6 || isempty(dt); dt = 0.01; end
N = size(spline_data, 1);
timespot_spl = (0:N-1)' * dt;
if isempty(spline_yaw); spline_yaw = zeros(N, 1); end
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints',     waypoints);       % 3xN
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl',  timespot_spl);
mws.assignin('spline_data',   spline_data);     % Nx3
mws.assignin('spline_yaw',    spline_yaw);
set_param(mdl, 'StopTime', num2str(T));
end
