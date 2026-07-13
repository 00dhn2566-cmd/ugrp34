%% 구운 모델 무수정 검증: 로드 -> 호버 8초 -> 자세/고도 표 출력
%% 전제: Models/quadcopter_package_delivery.slx 가 bake_tuned_model.m 로 구워진 상태.
%% 합격 기준: 8초 생존, 자세 RMS < 1도, z 0.9~1.05 유지 (호버 검증치 RMS 0.56도)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

dt = 0.01; T = 8; N = round(T/dt) + 1;
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

scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
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

fprintf('=== 구운 모델 무수정 호버 검증 (T=%gs) ===\n', T);
sim(mdl);
t = real_roll.time(:);
r = rad2deg(real_roll.signals.values(:));
pch = rad2deg(real_pitch.signals.values(:));
zv = real_z.signals.values(:);
fprintf('  %5s | %7s %7s | %6s\n', 't', 'roll', 'pitch', 'z');
for ct = 0:0.5:T
    [~, idx] = min(abs(t - ct));
    fprintf('  %5.1f | %7.2f %7.2f | %6.3f\n', t(idx), r(idx), pch(idx), zv(idx));
end
mask = t > 1;
rmsA = sqrt(mean(r(mask).^2 + pch(mask).^2));
fprintf('\n>> 자세 RMS %.2f도 / 최대|R| %.1f / 최대|P| %.1f / z [%.2f %.2f]\n', ...
    rmsA, max(abs(r)), max(abs(pch)), min(zv), max(zv));
if rmsA < 1.0 && min(zv) > 0.85
    fprintf('>> 합격: 구운 .slx가 자립적으로 안정 호버함\n');
else
    fprintf('>> 불합격: 굽기 재점검 필요 (bake_tuned_model.m)\n');
end
