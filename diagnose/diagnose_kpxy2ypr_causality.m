%% Position Control 안의 PID Controller 블록이 kp_xy2ypr/ki_xy2ypr/kd_xy2ypr/err2rp를
%% 참조하는데, 이 변수들은 repo 어디에도 정의돼 있지 않다. 그런데도 sim()은 에러 없이 성공한다.
%% 이 변수를 실제로 바꿨을 때 cmd_roll이 반응하는지 확인해서, 이 PID 블록이
%% 진짜 활성 신호경로에 있는지(죽은 블록/비활성 variant가 아닌지) 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
mdl = 'quadcopter_package_delivery';
load_system(mdl);
quadcopter_package_parameters;

dt = 0.01; T = 2;
N = round(T/dt)+1;
timespot_spl = (0:N-1)'*dt;
hoverPoint = [0,0,1.0];
spline_data = repmat(hoverPoint,N,1);
spline_yaw = zeros(N,1);
waypoints = [hoverPoint; hoverPoint+[0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl,'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

scope = [mdl '/Scope'];
sigMap = {'In Bus Element21','cmd_roll'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

fprintf('=== [1] kp_xy2ypr 정의 안 함 (베이스라인) ===\n');
simOut = sim(mdl);
c1 = rad2deg(cmd_roll.signals.values(:));
fprintf('  cmd_roll: min=%.3f max=%.3f last=%.3f\n', min(c1), max(c1), c1(end));

fprintf('\n=== [2] kp_xy2ypr=0.00001 (거의 0) ===\n');
kp_xy2ypr = 0.00001; ki_xy2ypr = 0; kd_xy2ypr = 0; filtD_xy2ypr = 100; err2rp = 1;
simOut = sim(mdl);
c2 = rad2deg(cmd_roll.signals.values(:));
fprintf('  cmd_roll: min=%.3f max=%.3f last=%.3f\n', min(c2), max(c2), c2(end));

fprintf('\n=== [3] kp_xy2ypr=99999 (거의 무한대) ===\n');
kp_xy2ypr = 99999;
simOut = sim(mdl);
c3 = rad2deg(cmd_roll.signals.values(:));
fprintf('  cmd_roll: min=%.3f max=%.3f last=%.3f\n', min(c3), max(c3), c3(end));

fprintf('\n=== 결론 ===\n');
if isequal(c1,c2) && isequal(c2,c3)
    fprintf('  kp_xy2ypr를 바꿔도 cmd_roll이 전혀 안 바뀜 -> 이 PID Controller 블록은 실제 신호경로에 없음(죽은 블록이거나 비활성 variant).\n');
else
    fprintf('  kp_xy2ypr 변화에 따라 cmd_roll이 바뀜 -> 이게 진짜 활성 게인.\n');
end
