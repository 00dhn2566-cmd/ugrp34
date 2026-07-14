%% 첫 이동 비행 테스트 6종 (사용자 지정 시나리오) — 추종 오차 채점
%% 1. x축 이동  2. x 이동 중 y로 점점 가속->최고속도  3. z 상승  4. z 하강
%% 5. x+z 동시 이동  6. xz 이동 중 y 성분 슬금슬금 증가
%% 궤적: 최소저크 프로파일(계단 명령 금지). 첫 이동이라 순항 ~1m/s, 케이스2 최고 2m/s 보수 설정.
%% 채점: 축별 추종 오차 RMS/최대/종점 (스펙: 완만 궤적 RMS<=0.1m), 자세 최대, z 최저.
%% 규칙: 구운 .slx 무수정(로깅만 추가), save_system 금지. 게인은 parameters.m 최신본.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 투하(Disengage) 로직 무력화 (run_sample_sim.m §enable_package_drop=false 방식) ---
% 1차 실행에서 최종 waypoint 도달 케이스(1,3,5)가 도착 직후 투하/착륙 시퀀스로 z 0.15 추락.
% 이동 테스트에는 임무 로직이 개입하면 안 되므로 거리 임계값을 -1로 (조건 영원히 불만족).
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
fprintf('투하 로직 무력화 완료 (dist threshold = -1)\n');

% --- 로깅 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element1','py'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
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

% --- 케이스 정의: 각 케이스는 {태그, T, 궤적함수 @(t)->[x y z]} ---
% mj(t,t0,t1,a,b): t0~t1 동안 a->b 최소저크, 밖에서는 유지
cases = {
  '1 x축 이동 (0->4m)',           10, @(t) [mj(t,1,7,0,4), zeros(size(t)), ones(size(t))];
  '2 x 이동 + y 점진가속->2m/s',  14, @(t) [mj(t,1,6,0,3), ycreep2(t), ones(size(t))];
  '3 z 상승 (1->3m)',             10, @(t) [zeros(size(t)), zeros(size(t)), mj(t,1,6,1,3)];
  '4 z 하강 (1->0.5m)',           10, @(t) [zeros(size(t)), zeros(size(t)), mj(t,1,5,1,0.5)];
  '5 x+z 동시 (x0->3, z1->2.5)',  10, @(t) [mj(t,1,7,0,3), zeros(size(t)), mj(t,1,7,1,2.5)];
  '6 xz 이동 + y 슬금슬금',        12, @(t) [mj(t,1,9,0,4), ycreep6(t), mj(t,1,9,1,2)];
};

fprintf('\n===== 이동 비행 테스트 6종 (첫 이동) =====\n');
summ = {};
for ci = 1:size(cases,1)
    tag = cases{ci,1};
    T = cases{ci,2};
    fn = cases{ci,3};
    dt = 0.01; N = round(T/dt) + 1;
    timespot_spl = (0:N-1)' * dt;
    spline_data = fn(timespot_spl);
    spline_yaw = zeros(N, 1);
    waypoints = [spline_data(1,:); spline_data(end,:)]';
    if isequal(waypoints(:,1), waypoints(:,2)); waypoints(:,2) = waypoints(:,2) + [0;0;2]; end
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    mws = get_param(mdl, 'ModelWorkspace');
    mws.assignin('waypoints', waypoints);
    mws.assignin('wayp_path_vis', wayp_path_vis);
    mws.assignin('timespot_spl', timespot_spl);
    mws.assignin('spline_data', spline_data);
    mws.assignin('spline_yaw', spline_yaw);
    set_param(mdl, 'StopTime', num2str(T));
    fprintf('\n--- %s (T=%gs) ---\n', tag, T);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        summ(end+1,:) = {tag, NaN, NaN, NaN, NaN, false}; %#ok<SAGROW>
        continue;
    end
    t = px.time(:);
    act = [px.signals.values(:), ...
           interp1(py.time(:), py.signals.values(:), t, 'linear', 'extrap'), ...
           interp1(pz.time(:), pz.signals.values(:), t, 'linear', 'extrap')];
    des = interp1(timespot_spl, spline_data, t, 'linear', 'extrap');
    err = act - des;
    r = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), t, 'linear', 'extrap'));
    pch = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear', 'extrap'));
    for ct = 0:2:T
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%4.1f | des[%6.2f %6.2f %5.2f] act[%6.2f %6.2f %5.2f] | R %5.1f P %5.1f\n', ...
            t(idx), des(idx,:), act(idx,:), r(idx), pch(idx));
    end
    mask = t > 1;
    eRms = sqrt(mean(sum(err(mask,:).^2, 2)));            % 3D 오차 RMS
    eMax = max(sqrt(sum(err(mask,:).^2, 2)));             % 3D 오차 최대
    eEnd = sqrt(sum(err(end,:).^2));
    attMax = max(max(abs(r)), max(abs(pch)));
    zMin = min(act(:,3));
    % 제어 신호량: 모터 회전수(|rad/s|) — 호버 기준 대비 이탈과 출력한계 접근도
    W = zeros(numel(t), 4);
    wvars = {w1, w2, w3, w4};
    for k = 1:4
        W(:,k) = abs(interp1(wvars{k}.time(:), wvars{k}.signals.values(:), t, 'linear', 'extrap'));
    end
    wHov = 635;             % 호버 회전수 [rad/s] (실측)
    wCeil = 1025;           % 지속 가능 상한 [rad/s] (정격 160W 역산)
    wMax = max(W(t > 1, :), [], 'all');
    wDevMax = max(abs(W(t > 1, :) - wHov), [], 'all');
    ceilPct = 100 * wMax / wCeil;
    surv = zMin > 0.2 && attMax < 30;
    ok = surv && eRms < 0.10 && eMax < 0.25 && ceilPct < 90;
    fprintf('  >> 추종오차 RMS %.3fm / 최대 %.3fm / 종점 %.3fm / 자세최대 %.1f도 / z최저 %.3f\n', ...
        eRms, eMax, eEnd, attMax, zMin);
    fprintf('  >> 제어량: 모터 최대 %.0f rad/s (한계 %d의 %.0f%%) / 호버 대비 최대 이탈 %.0f rad/s / %s\n', ...
        wMax, wCeil, ceilPct, wDevMax, tf(ok, '합격', '불합격/검토'));
    summ(end+1,:) = {tag, eRms, eMax, attMax, ceilPct, ok}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%-30s | %8s | %8s | %8s | %9s | %s\n', '케이스', 'RMS[m]', '최대[m]', '자세[도]', '모터한계%%', '판정');
for ci = 1:size(summ,1)
    fprintf('%-30s | %8.3f | %8.3f | %8.1f | %9.0f | %s\n', summ{ci,1}, summ{ci,2}, summ{ci,3}, summ{ci,4}, summ{ci,5}, tf(summ{ci,6},'합격','불합격'));
end
fprintf('(합격: 생존 + RMS<0.10m + 최대<0.25m + 모터<한계90%%. 불합격은 피드포워드/게인 개선 대상)\n');

function p = mj(t, t0, t1, a, b)
    % 최소저크 프로파일: t0~t1 동안 a->b, 이전 a 유지, 이후 b 유지
    tau = min(max((t - t0) / (t1 - t0), 0), 1);
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    p = a + (b - a) * s;
    p = p(:);
end

function y = ycreep2(t)
    % 케이스2: t=5부터 y 가속(0.4m/s^2) -> v=2m/s 도달(t=10) 후 등속
    y = zeros(size(t));
    m1 = t >= 5 & t < 10;
    y(m1) = 0.2 * (t(m1) - 5).^2;
    m2 = t >= 10;
    y(m2) = 5 + 2 * (t(m2) - 10);
    y = y(:);
end

function y = ycreep6(t)
    % 케이스6: t=4부터 y가 슬금슬금 (완만 2차, ~0.04m/s^2)
    y = zeros(size(t));
    m = t >= 4;
    y(m) = 0.02 * (t(m) - 4).^2;
    y = y(:);
end

function s = tf(c, a, b)
    if c; s = a; else; s = b; end
end
