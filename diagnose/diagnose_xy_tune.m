%% x 스텝 saturation 정리: 스텝 크기 스윕으로 응답 한계 사슬 실측 (사용자 지정)
%% 스텝 0.5/1/2/4m @ t=3s. 작은 스텝 = 순수 rise time, 큰 스텝 = 클램프(±0.15m) 포화 후
%% 등속 접근(최대 접근 속도), 자세/모터 사용 한계까지 한 번에 정리.
%% 채점: rise(10-90%), 오버슈트, vmax, amax, 자세피크, 모터피크(한계1025 대비 %), 정착(±5cm).
%% 투하 로직 무력화 포함. 구운 .slx 무수정(메모리만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 투하 로직 무력화 ---
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

% --- 로깅 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element11','w1'; 'In Bus Element10','w2'; 'In Bus Element12','w3'; 'In Bus Element13','w4'};
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

% 사용자 가설: 발산은 튜닝으로 해결 가능 -> 발산 확정 조건(1m/0.67s) 고정, 위치 게인 스윕
% {kp_pos, ki_pos, kd_pos}
gset = [ 8 0.04 3.2;    % 기준 (발산 재현용)
         8 0.04 6.4;    % D 2배
         8 0.04 9.6;    % D 3배
         8 0    6.4;    % D 2배 + I 제거 (와인드업 차단)
         5 0.04 6.4 ];  % kp 완화 + D 2배
steps = ones(1, size(gset,1));   % 전 케이스 1m 고정
T = 12; dt = 0.01; tStep = 3;
fprintf('\n===== x 스텝 saturation 스윕 (%s m @ t=%gs) =====\n', mat2str(steps), tStep);
summ = {};
for si = 1:numel(steps)
    A = steps(si);
    kp_position = gset(si,1); ki_position = gset(si,2); kd_position = gset(si,3);
    fprintf('[게인] kp_pos=%g ki_pos=%g kd_pos=%g\n', kp_position, ki_position, kd_position);
    N = round(T/dt) + 1;
    timespot_spl = (0:N-1)' * dt;
    % 준계단: 생 스텝은 미분 킥+포화 연쇄로 발산함(1차 실행 실측) -> 짧은 최소저크로 대체
    Tm = max(0.5, A / 1.5);   % 이동시간: 최소 0.5s, 평균속도 1.5m/s 상한
    tau = min(max((timespot_spl - tStep) / Tm, 0), 1);
    xref = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    spline_data = [xref, zeros(N,1), ones(N,1)];
    spline_yaw = zeros(N, 1);
    waypoints = [0 0 1; A 0 3]';   % 목표를 z+2에 둬서 임무로직 여지 차단(이중 안전)
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    mws = get_param(mdl, 'ModelWorkspace');
    mws.assignin('waypoints', waypoints);
    mws.assignin('wayp_path_vis', wayp_path_vis);
    mws.assignin('timespot_spl', timespot_spl);
    mws.assignin('spline_data', spline_data);
    mws.assignin('spline_yaw', spline_yaw);
    set_param(mdl, 'StopTime', num2str(T));
    fprintf('\n--- 스텝 %.1fm ---\n', A);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', getReport(e, 'extended', 'hyperlinks', 'off'));
        summ(end+1,:) = {A, NaN, NaN, NaN, NaN, NaN, NaN, NaN}; %#ok<SAGROW>
        continue;
    end
    t = px.time(:);
    x = px.signals.values(:);
    zv = interp1(pz.time(:), pz.signals.values(:), t, 'linear', 'extrap');
    r = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), t, 'linear', 'extrap'));
    pch = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear', 'extrap'));
    W = zeros(numel(t), 4);
    wvars = {w1, w2, w3, w4};
    for k = 1:4
        W(:,k) = abs(interp1(wvars{k}.time(:), wvars{k}.signals.values(:), t, 'linear', 'extrap'));
    end
    % 균일 그리드 재샘플 (미분용)
    tu = (0:0.01:T)';
    xu = interp1(t, x, tu, 'linear', 'extrap');
    vu = gradient(xu, 0.01);
    au = gradient(vu, 0.01);
    % rise 10-90%
    i10 = find(xu >= 0.1*A, 1); i90 = find(xu >= 0.9*A, 1);
    tRise = NaN;
    if ~isempty(i10) && ~isempty(i90); tRise = tu(i90) - tu(i10); end
    ovs = max(xu) - A;
    % 정착: 스텝 후 |x-A|<0.05 유지 시작
    tSet = NaN;
    okm = abs(xu - A) < 0.05;
    iS = find(tu >= tStep, 1);
    for ii = iS:numel(tu)
        if all(okm(ii:end)); tSet = tu(ii) - tStep; break; end
    end
    mPost = tu > tStep & tu < tStep + 6;
    vMax = max(abs(vu(mPost)));
    aMax = max(abs(au(mPost & tu < tStep+2)));
    attPk = max(max(abs(r)), max(abs(pch)));
    wPk = max(W(t > 1, :), [], 'all');
    ceilPct = 100 * wPk / 1025;
    for ct = [2.9 3.2 3.5 4 4.5 5 6 7 8 10 12]
        [~, idx] = min(abs(tu - ct));
        fprintf('  t=%5.1f | x %6.3f v %6.2f | (자세 생략)\n', tu(idx), xu(idx), vu(idx));
    end
    fprintf('  >> rise(10-90) %.2fs / 오버슈트 %.3fm / 정착(±5cm) %.2fs / vmax %.2fm/s / amax %.2fm/s2 / 자세피크 %.1f도 / 모터 %.0f%%\n', ...
        tRise, ovs, tSet, vMax, aMax, attPk, ceilPct);
    summ(end+1,:) = {A, tRise, ovs, tSet, vMax, aMax, attPk, ceilPct}; %#ok<SAGROW>
end

fprintf('\n===== x saturation 정리표 =====\n');
fprintf('%6s | %9s | %9s | %8s | %8s | %9s | %8s | %7s\n', '스텝[m]','rise[s]','오버슈트[m]','정착[s]','vmax','amax','자세[도]','모터%%');
for si = 1:size(summ,1)
    fprintf('%6.1f | %9.2f | %9.3f | %8.2f | %8.2f | %9.2f | %8.1f | %7.0f\n', summ{si,:});
end
fprintf(['(해석) 작은 스텝의 rise = 순수 응답. 큰 스텝의 vmax 수렴값 = 클램프(±0.15m x kp_pos)가 정의하는\n' ...
    ' 최대 접근 속도. 이 표의 vmax/amax가 path_time 제약 및 quick모드 능력치의 실측 기초.\n']);
