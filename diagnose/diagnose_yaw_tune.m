%% yaw 게인 스윕: 실측된 yaw 스윙(최대 -11.4도, 복귀 ~10s)의 개선 게인 탐색
%% 배경(diagnose_yaw_check.m): 이륙 과도기 반토크가 yaw를 밀고, kp=3/kd=1이 약해 복귀가 느림.
%% 처방 순서: P/D 강성 증가 우선, 상수 오프셋 대비 소량 ki 조합 병행 시험.
%% 각 세트 20초 호버, 한 프로세스 순차 실행 (동시 실행 금지 - RAM).
%% 규칙: 대상 미발견 시 error() 즉사. 구운 .slx 무수정 (save_system 금지).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 궤적: 20초 호버 (목표 yaw=0) ---
dt = 0.01; T = 20; N = round(T/dt) + 1;
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

% --- 로깅: 실측 매핑 확정본 (Element5=Chassis.yaw 포함) ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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

% --- 게인 세트: [kp ki kd] ---
gainSets = [ 3   0    1;    % 기준 (현재 채택값)
             6   0    2;    % 강성 2배
            10   0    3;    % 강성 3배+
             6   0.3  2;    % 강성 2배 + 소량 적분
            10   0.5  3;    % 강성 3배 + 적분
            15   0    4;    % 강성 5배 (사용자: 더 세게)
            20   0.5  5 ];  % 강성 극단 + 적분 (포화/발진 경계 확인용)
fprintf('\n===== yaw 게인 스윕 (20s 호버, 세트 %d개) =====\n', size(gainSets,1));
results = {};
for gi = 1:size(gainSets, 1)
    kp_yaw = gainSets(gi,1); ki_yaw = gainSets(gi,2); kd_yaw = gainSets(gi,3);
    fprintf('\n--- 세트 %d: kp=%g ki=%g kd=%g ---\n', gi, kp_yaw, ki_yaw, kd_yaw);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        results(end+1,:) = {gi, kp_yaw, ki_yaw, kd_yaw, NaN, NaN, NaN, NaN, false}; %#ok<SAGROW>
        continue;
    end
    t = real_yaw.time(:);
    yw = rad2deg(real_yaw.signals.values(:));
    r = rad2deg(real_roll.signals.values(:));
    pch = rad2deg(real_pitch.signals.values(:));
    zv = real_z.signals.values(:);
    for ct = 0:2:T
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%4.1f | yaw %8.3f | roll %6.2f pitch %6.2f | z %6.3f\n', t(idx), yw(idx), r(idx), pch(idx), zv(idx));
    end
    maxY = max(abs(yw));
    % 정착: |yaw|<1도가 끝까지 유지되는 첫 시각
    tSet = NaN;
    okm = abs(yw) < 1.0;
    for ii = 1:numel(t)
        if all(okm(ii:end)); tSet = t(ii); break; end
    end
    maskSS = t >= T-5;
    ssY = mean(yw(maskSS));
    mask2 = t > 2;
    rmsA = sqrt(mean(r(mask2).^2 + pch(mask2).^2));
    surv = (min(zv) > 0.3) && (max(abs(r)) < 45) && (max(abs(pch)) < 45);
    fprintf('  >> 최대|yaw| %.2f도 / 정착(<1도) %.1fs / 말기평균 %.3f도 / 자세RMS %.2f / z [%.2f %.2f] / %s\n', ...
        maxY, tSet, ssY, rmsA, min(zv), max(zv), tstr(surv,'생존','추락'));
    results(end+1,:) = {gi, kp_yaw, ki_yaw, kd_yaw, maxY, tSet, ssY, rmsA, surv}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%3s | %4s %4s %4s | %9s | %8s | %9s | %7s | %s\n', '세트','kp','ki','kd','최대yaw','정착[s]','말기[도]','자세RMS','판정');
for ci = 1:size(results,1)
    ok = results{ci,9} && results{ci,5} < 5 && ~isnan(results{ci,6});
    fprintf('%3d | %4g %4g %4g | %9.2f | %8.1f | %9.3f | %7.2f | %s\n', ...
        results{ci,1}, results{ci,2}, results{ci,3}, results{ci,4}, results{ci,5}, results{ci,6}, results{ci,7}, results{ci,8}, tstr(ok,'합격','불합격'));
end
fprintf('(합격 기준: 생존 + 최대|yaw|<5도 + 1도 이내 정착 달성. roll/pitch RMS가 기준세트 대비 나빠지지 않는 세트 중 최소 게인 채택 권장)\n');

function s = tstr(c, a, b)
    if c; s = a; else; s = b; end
end
