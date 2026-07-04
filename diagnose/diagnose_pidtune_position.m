%% systune 대신 linearize()+pidtune()으로 Position Control 루프를 직접 튜닝.
%% slTuner의 TunedBlocks/InputName 이슈를 완전히 우회한다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

% 내부(자세) 루프가 포화로 발산하면 위치 루프 선형화도 의미가 없어지므로,
% attitude 게인은 포화를 피하는 보수적인 값으로 임시 고정.
kp_attitude = 5;
ki_attitude = 0;
kd_attitude = 2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 선형화 지점이 "필요없는 블록"으로 오인되어 컴파일 단계에서 최적화로
% 제거되면 io 지점이 있으나마나해져서 전부 0이 나올 수 있음 -> 꺼둔다.
set_param(mdl, 'BlockReduction', 'off');

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

% Position Control의 PID Controller 자체를 배제하고, 그 앞뒤(레퍼런스 입력 x,
% 실제 위치 출력 x)만 io로 잡아서 "Plant"만 선형화한다.
% io(1) 'in'  : Position Control/PID Controller의 출력단(Pitch/Roll Cmd로 나가기 전,
%               제어기가 만든 명령)을 가상의 입력으로 끊어서 주입.
% io(2) 'out' : 실제 위치(act_x, Chassis.px)를 관측.
posPid = [mdl '/Maneuver Controller/Position Control/PID Controller'];
io(1) = linio(posPid, 1, 'in');
io(2) = linio([mdl '/Scope/In Bus Element'], 1, 'out');

for snapshotTime = [5 10 20 30]
    fprintf('=== linearize()로 Position Control Plant 추출 (snapshot t=%g) ===\n', snapshotTime);
    try
        [sys, info] = linearize(mdl, io, snapshotTime);
        fprintf('  선형화 성공. sys class=%s, size=%s, nstates=%d\n', class(sys), mat2str(size(sys)), size(sys.A,1));
        fprintf('  D = %s\n', mat2str(sys.D));
    catch e
        fprintf('  FAILED: %s\n', e.message);
    end
end

fprintf('\n=== Position Control 자체 입력(Traj)->출력(Pitch Cmd)으로 io 재설정 ===\n');
io2(1) = linio([mdl '/Maneuver Controller/Position Control'], 2, 'in');   % Traj (In2)
io2(2) = linio([mdl '/Maneuver Controller/Position Control'], 1, 'out'); % Pitch Cmd (Out1)
try
    [sys2, info2] = linearize(mdl, io2, 20);
    fprintf('  선형화 성공. size=%s, nstates=%d\n', mat2str(size(sys2)), size(sys2.A,1));
    fprintf('  D = %s\n', mat2str(sys2.D));
catch e
    fprintf('  FAILED: %s\n', e.message);
end
