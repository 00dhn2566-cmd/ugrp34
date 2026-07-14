%% 도착 후 pitch 한계사이클(1.75Hz, 감쇠 0) 처방 대결 (사용자 가설 vs 실측 가설)
%% 사용자 가설: 자세제어에 I항 필요 -> ki_attitude -10 / -30 (게인 부호 음수 규약)
%% 실측 가설: 위치 D항이 진동 발전기(명령상관 r=0.985, D 기여 P의 4배) -> kd_pos 2.4 / 1.6
%% 같은 비행(성형된 0.67s 스텝, 스무더 통과본)으로 5케이스 비교.
%% 판정 지표: 도착 후 pitch RMS(6~9 vs 9~12)와 그 비율(감쇠비), 과도 성능 유지 여부.
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지. 게인은 base 워크스페이스 주입.

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

% 케이스: {이름, ki_attitude, kd_position}
KI0 = ki_attitude; KD0 = kd_position;
cases = { '기준(현행)',        KI0,  KD0;
          '자세I ki=-10',      -10,  KD0;
          '자세I ki=-30',      -30,  KD0;
          'kd_pos 2.4',        KI0,  2.4;
          'kd_pos 1.6',        KI0,  1.6 };

fprintf('===== 자세 I vs 위치 D 축소: 도착 후 한계사이클 처방 대결 =====\n');
summ = {};
for ci = 1:size(cases,1)
    ki_attitude = cases{ci,2};
    kd_position = cases{ci,3};
    fprintf('\n--- 케이스 %d: %s (ki_att=%g, kd_pos=%g) ---\n', ci, cases{ci,1}, ki_attitude, kd_position);
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
    fprintf('  pitch RMS 6~9s %.3f도 / 9~12s %.3f도 / 비율 %.2f | x오차 %.2fcm | 과도피크 %.1f도 | 오버슈트 %.1fcm | z최저 %.2f\n', ...
        r1, r2, r2/r1, exR, attPk, xOv*100, min(zg(seg(3,12))));
    summ(end+1,:) = {cases{ci,1}, r1, r2, r2/r1, exR, attPk, xOv*100}; %#ok<SAGROW>
end

fprintf('\n===== 요약 (낮을수록 좋음: RMS/비율/오차) =====\n');
fprintf('%-14s | %8s | %8s | %6s | %8s | %8s | %8s\n', '케이스','RMS 6~9','RMS 9~12','비율','x오차cm','과도도','O.S.cm');
for ci = 1:size(summ,1)
    fprintf('%-14s | %8.3f | %8.3f | %6.2f | %8.2f | %8.1f | %8.1f\n', summ{ci,:});
end
fprintf('(비율<0.5 = 감쇠 회복. 자세I가 이기면 사용자 가설 채택, kd_pos가 이기면 D 발전기 가설 채택)\n');
