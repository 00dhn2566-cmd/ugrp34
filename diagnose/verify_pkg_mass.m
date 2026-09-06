%% QC_PKG_MASS 훅이 **플랜트에도** 먹는지 확인 (2026-09-06)
%%
%% 왜 필요한가: parameters.m 에서 pkgDensity 를 바꾸면 게인은 확실히 따라온다
%% (그 파일 안에서 파생되므로). 하지만 Simscape 쪽 Package 솔리드가 그 변수를
%% 정말 읽는지는 별개 문제다. get_param 으로는 pkgDensity 를 참조하는 블록이
%% 안 잡혔다 — 안 읽는 것일 수도, 노출이 안 되는 것일 수도 있다.
%% 게인만 무거워지고 플랜트는 가벼우면 불안정이 나오는데, 그걸 "2 kg 성능"으로
%% 잘못 읽으면 안 된다. 그래서 **호버 모터 속도**로 직접 잰다 — 플랜트가 무거우면
%% 같은 고도를 유지하는 데 더 빨리 돌아야 한다. 이건 파라미터가 아니라 물리다.
%%
%% 판정: 속도비 ~ sqrt(질량비). 2.273 -> 3.273 kg 이면 sqrt(1.44) = 1.200 배.
%%       비가 1.00 이면 플랜트가 안 바뀐 것이고, 이 세션의 2 kg 결과는 폐기다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';

fprintf(['[verify_pkg_mass] QC_PKG_MASS=%s -> m_pkg_now=%.3f, m_tot(파라미터)=%.3f' char(10)], ...
        getenv('QC_PKG_MASS'), m_pkg_now, m_tot_now);

if bdIsLoaded(mdl); close_system(mdl, 0); end
load_system(mdl);
qctest.disable_drop(mdl);

dt = 0.01; T = 4;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
spline_data = repmat(hoverPoint, N, 1);
spline_yaw = zeros(N, 1);
waypoints = [hoverPoint; hoverPoint + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));

% 모터 1 의 실제 속도만 있으면 된다 (4개가 호버에서 같다)
elec = sprintf('%s/Quadcopter/Electrical/Control1', mdl);
twName = 'To Workspace pkgchk_m1';
if isempty(find_system(elec, 'SearchDepth', 1, 'Name', twName))
    twBlk = [elec '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'pkgchk_m1', ...
              'SaveFormat', 'StructureWithTime');
    srcPh = get_param([elec '/meas'], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(elec, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

sim(mdl);
w = pkgchk_m1.signals.values(:);
tw = pkgchk_m1.time(:);
tail = tw > 0.6 * T;                       % 과도 지나고 정상 호버 구간
fprintf(['[verify_pkg_mass] 호버 모터속도 평균 = %.2f rad/s  (마지막 %.1f s)' char(10)], ...
        mean(w(tail)), T - 0.6*T);
close_system(mdl, 0);
