%% yaw 확인 ①: Scope 버스에서 yaw Element 자동 식별 + 10초 호버 yaw 드리프트 실측
%% 목적: "yaw 안 잡는 것 같다" 관찰의 실측 검증. ki_yaw 추가 여부는 이 결과로 결정.
%% 규칙: yaw Element를 이름으로 못 찾거나 중복이면 error() 즉사 (추측 태핑 금지).
%%       구운 .slx 무수정 (로깅 To Workspace만 메모리 추가, save_system 금지).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- [1] Scope 버스 신호 전체 매핑 출력 + yaw/x/y 후보 자동 식별 ---
scope = [mdl '/Scope'];
ibs = find_system(scope, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^In Bus Element');
fprintf('=== Scope In Bus Element 매핑 (%d개) ===\n', numel(ibs));
yawBlk = {}; xBlk = {}; yBlk = {};
for i = 1:numel(ibs)
    nm = strtrim(regexprep(get_param(ibs{i}, 'Name'), '\s+', ' '));
    el = '(Element 파라미터 없음)';
    try; el = get_param(ibs{i}, 'Element'); catch; end
    fprintf('  %-22s -> %s\n', nm, el);
    elL = lower(el);
    if contains(elL, 'yaw'); yawBlk{end+1} = ibs{i}; end %#ok<SAGROW>
end
if isempty(yawBlk)
    error('Element 문자열에 yaw가 포함된 In Bus Element 없음 - 위 매핑 보고 수동 지정 필요. 실행 무효');
end
% 실측 yaw = Chassis.yaw (실측 확인: Element5=Chassis.yaw, Element16='yaw'는 명령 계열)
realYaw = {};
for i = 1:numel(yawBlk)
    elL = lower(get_param(yawBlk{i}, 'Element'));
    if contains(elL, 'chassis')
        realYaw{end+1} = yawBlk{i}; %#ok<SAGROW>
    end
end
if numel(realYaw) ~= 1
    fprintf('yaw 후보 %d개:\n', numel(realYaw));
    for i = 1:numel(realYaw)
        fprintf('  %s -> %s\n', get_param(realYaw{i}, 'Name'), get_param(realYaw{i}, 'Element'));
    end
    error('실측 yaw 후보가 1개로 안 좁혀짐 - 위 목록 보고 수동 지정 필요. 실행 무효');
end
fprintf('\n채택 yaw 신호: %s -> %s\n', strtrim(regexprep(get_param(realYaw{1},'Name'),'\s+',' ')), get_param(realYaw{1}, 'Element'));

% --- [2] 궤적: 10초 호버 (yaw 목표 0) ---
dt = 0.01; T = 10; N = round(T/dt) + 1;
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

% --- [3] 로깅: 검증된 3개 + yaw ---
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
twName = 'To Workspace real_yaw';
oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
if ~isempty(oldTw); delete_block(oldTw{1}); end
twBlk = [scope '/' twName];
add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', 'real_yaw', 'SaveFormat', 'StructureWithTime');
srcPh = get_param(realYaw{1}, 'PortHandles');
twPh  = get_param(twBlk, 'PortHandles');
add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');

% --- [4] 실행 + 평가 ---
fprintf('\n===== yaw 드리프트 실측 (10s 호버, 목표 yaw=0) =====\n');
sim(mdl);
t = real_yaw.time(:);
yw = rad2deg(real_yaw.signals.values(:));
r = rad2deg(real_roll.signals.values(:));
pch = rad2deg(real_pitch.signals.values(:));
zv = real_z.signals.values(:);
fprintf('  %5s | %8s | %7s %7s | %6s\n', 't', 'yaw', 'roll', 'pitch', 'z');
for ct = 0:1:T
    [~, idx] = min(abs(t - ct));
    fprintf('  %5.1f | %8.3f | %7.2f %7.2f | %6.3f\n', t(idx), yw(idx), r(idx), pch(idx), zv(idx));
end
% 드리프트 지표: 후반(5~10s) 선형 추세 기울기 + 정상상태 오차
maskSS = t >= 5;
p = polyfit(t(maskSS), yw(maskSS), 1);
driftRate = p(1);              % 도/초
ssErr = mean(yw(maskSS));      % 도
maxAbs = max(abs(yw));
fprintf('\n>> yaw 최대|.| %.3f도 / 후반 평균오차 %.3f도 / 드리프트율 %.4f도/초 (10초 환산 %.2f도)\n', ...
    maxAbs, ssErr, driftRate, driftRate*10);
if maxAbs < 1.0 && abs(driftRate) < 0.05
    fprintf('>> 판정: yaw 정상 유지 - ki_yaw 추가 불필요, PD 유지\n');
elseif abs(driftRate) >= 0.05
    fprintf('>> 판정: yaw 드리프트 실재 - ki_yaw 0.1~0.3 추가 검토 필요\n');
else
    fprintf('>> 판정: 정상상태 오프셋 존재(드리프트는 아님) - ki_yaw 소량 추가로 제거 가능\n');
end
