%% 한계사이클 발전기 위치 확정 실험 (사용자 진단: "이동 관성이 잔류하며 롤피치 계속 남음")
%% 이전 스윕: 자세I 효과 0, kd_pos 절반 축소해도 진폭만 감소·감쇠 회복 없음.
%% 이번: 위치루프 게인을 극단까지 꺾어 진동 잔존 여부로 발전기 위치 확정.
%%  - kd_pos=0인데도 진동 잔존 -> 발전기는 위치 D 바깥 (비선형 요소 수사로 전환)
%%  - kd_pos=0에서 진동 소멸(단 오버슈트 대가) -> D 경로가 필수 고리 확정
%%  - kp_pos 절반: 루프 전체 이득 스케일링 효과 분리
%% 같은 비행(성형된 0.67s 스텝). 규칙: 구운 .slx 무수정, save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smKill = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(dropBlocks{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(dropBlocks)
    set_param(dropBlocks{i}, 'Value', '-1');
end

waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', smKill);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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

% 케이스: {이름, kp_position, kd_position}
KP0 = kp_position; KD0 = kd_position;
cases = { 'kd_pos 0',          KP0,   0;
          'kp_pos 4',            4, KD0;
          'kp4 + kd1.6',         4, 1.6 };

fprintf('===== 발전기 위치 확정: 위치루프 게인 극단 스윕 (기준: RMS 3.94/4.00, 비율 1.02) =====\n');
summ = {};
for ci = 1:size(cases,1)
    kp_position = cases{ci,2};
    kd_position = cases{ci,3};
    fprintf('\n--- 케이스 %d: %s (kp_pos=%g, kd_pos=%g) ---\n', ci, cases{ci,1}, kp_position, kd_position);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        summ(end+1,:) = {cases{ci,1}, NaN, NaN, NaN, NaN, NaN, NaN}; %#ok<SAGROW>
        continue;
    end
    tu = (0:0.005:T)';
    xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
    xrg = interp1(tt, smKill(:,1), tu, 'linear', 'extrap');
    ex = xrg - xg;
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    r1 = rmsf(pg(seg(6,9)));
    r2 = rmsf(pg(seg(9,12)));
    exR = sqrt(mean(ex(seg(6,12)).^2))*100;
    attPk = max(abs(pg(seg(3,6))));
    xOv = max(xg) - A;
    % 주파수도 (영점교차)
    pgP = pg(seg(6,12)); pgP = pgP - mean(pgP);
    freq = sum(abs(diff(sign(pgP)))>0)/2/6;
    fprintf('  pitch RMS 6~9s %.3f / 9~12s %.3f / 비율 %.2f | 주파수 %.2fHz | x오차 %.2fcm | 과도 %.1f도 | O.S. %.1fcm | z %.2f\n', ...
        r1, r2, r2/r1, freq, exR, attPk, xOv*100, min(zg(seg(3,12))));
    summ(end+1,:) = {cases{ci,1}, r1, r2, r2/r1, freq, exR, xOv*100}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%-13s | %8s | %8s | %6s | %6s | %8s | %8s\n', '케이스','RMS 6~9','RMS 9~12','비율','Hz','x오차cm','O.S.cm');
fprintf('%-13s | %8.3f | %8.3f | %6.2f | %6s | %8.2f | %8.1f\n', '기준(참고)', 3.940, 4.001, 1.02, '1.75', 0.96, 7.6);
for ci = 1:size(summ,1)
    fprintf('%-13s | %8.3f | %8.3f | %6.2f | %6.2f | %8.2f | %8.1f\n', summ{ci,:});
end
