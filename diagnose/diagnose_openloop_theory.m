%% "닫힌 루프에 외란 주입" 방식이라 정상상태 적분 제어의 외란 제거 특성 때문에
%% 0이 나오는 건지 확인. 컨트롤러 자체(입력 오차 -> 출력 명령)만 떼서 테스트.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

S = load(fullfile(modelDir, 'trajectory.mat'));
timespot_spl = S.timespot_spl;
spline_data  = S.spline_data;
spline_yaw   = S.spline_yaw;
waypoints    = S.waypoints';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);

mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);

posPid = [mdl '/Maneuver Controller/Position Control/PID Controller'];
ph = get_param(posPid, 'PortHandles');
fprintf('PID Controller In=%d Out=%d\n', numel(ph.Inport), numel(ph.Outport));

fprintf('\n=== 테스트 A: 컨트롤러 입력(오차) -> Position Control 출력(Pitch Cmd), 순방향만 ===\n');
io(1) = linio(posPid, 1, 'in');   % 컨트롤러 입력(오차)에 테스트 신호 주입
io(2) = linio([mdl '/Maneuver Controller/Position Control'], 1, 'out');  % Pitch Cmd (다른 블록의 출력)
try
    [sys, ~] = linearize(mdl, io, 20);
    fprintf('  size=%s, nstates=%d, D=%s\n', mat2str(size(sys)), size(sys.A,1), mat2str(sys.D));
catch e
    fprintf('  FAILED: %s\n', e.message);
end

fprintf('\n=== 테스트 A2: PID 자체 In/Out을 AnalysisPoint(둘다 지정)로 열린루프 게인 ===\n');
try
    ioA2 = linio(posPid, 1, 'in');
    ST0 = slTuner(mdl, {}, ioA2);
    L = getLoopTransfer(ST0, ioA2.Name, -1);
    fprintf('  getLoopTransfer 성공: size=%s\n', mat2str(size(L)));
catch e
    fprintf('  FAILED: %s\n', e.message);
end

fprintf('\n=== 테스트 B: 컨트롤러 출력 -> 실제 위치(act_x), 닫힌 루프에 외란 주입 (기존 방식) ===\n');
io2(1) = linio(posPid, 1, 'in');
io2(2) = linio([mdl '/Scope/In Bus Element'], 1, 'out');
try
    [sys2, ~] = linearize(mdl, io2, 20);
    fprintf('  size=%s, nstates=%d, D=%s\n', mat2str(size(sys2)), size(sys2.A,1), mat2str(sys2.D));
catch e
    fprintf('  FAILED: %s\n', e.message);
end
