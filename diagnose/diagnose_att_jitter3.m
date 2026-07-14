%% roll/pitch 지터(RMS 0.56도, ±1도 배회) 원인 분리 배터리 — 스펙 R4/R5 대응
%% 용의자: (a) 위치루프가 만드는 자세명령 지터 (b) kd=-150의 노이즈 증폭
%%         (c) 미분필터 filtD_attitude=1000 과대역 (d) 내루프 자체 리미트사이클
%% 구성 5개 x 10s 호버. FFT로 지배 주파수까지 채점 (규칙적 주파수=리미트사이클, 광대역=노이즈).
%% 구성2(위치루프 절단)는 배선 변경이 비가역이라 맨 마지막에 실행.
%% 규칙: 대상 미발견 시 error() 즉사. 구운 .slx 무수정(메모리만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;   % 최종 게인 (yaw 포함)
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 궤적: 10초 호버 ---
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

% --- 로깅: 자세 + 모터 회전수 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element11','w1'; 'In Bus Element10','w2'; ...
          'In Bus Element12','w3'; 'In Bus Element13','w4'};
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

% --- 구성 정의: {태그, kd_att, filtD_att, 위치루프절단} ---
% 기준 게인은 parameters.m의 kp_attitude=-100 유지 (kp는 고정 - 강성 자체는 검증됨)
% 2차 스윕(감쇠 강화 방향): 1차에서 kd/필터 축소·위치루프 절단 전부 악화 ->
% 지터 = 자세 내루프의 ~7Hz 저감쇠 공진(모터 시정수 0.02s 가설). 감쇠를 올려본다.
% 3차 스윕(사용자 가설 판정): kp 과대 여부(양방향) + I 부재 여부. kd=-150/fD2000 고정(2차 최적).
% cfgs 열: {태그, kp_att, ki_att, kd_att, filtD_att}
cfgs = { '7 기준(kp-100,fD2000)',      -100,   0, -150, 2000; ...
         'A kp-60',                     -60,   0, -150, 2000; ...
         'B kp-140',                   -140,   0, -150, 2000; ...
         'C ki-20',                    -100, -20, -150, 2000 };
results = {};
posCutDone = false;
fprintf('\n===== roll/pitch 지터 원인 분리 (5구성 x %gs 호버) =====\n', T);
for ci = 1:size(cfgs, 1)
    tag = cfgs{ci,1};
    kp_attitude = cfgs{ci,2};
    ki_attitude = cfgs{ci,3};
    kd_attitude = cfgs{ci,4};
    filtD_attitude = cfgs{ci,5};
    needCut = false;
    if needCut && ~posCutDone
        % 위치루프 절단: PC Out1/2 -> 상수 0 (검증된 방법: diagnose_att_step.m)
        mc = [mdl '/Maneuver Controller'];
        pcB = [mc '/Position Control'];
        pph = get_param(pcB, 'PortHandles');
        for oi = 1:2
            l = get_param(pph.Outport(oi), 'Line');
            if l == -1; error('PC Out%d 라인 없음 - 실행 무효', oi); end
            dsts = get_param(l, 'DstPortHandle');
            delete_line(l);
            cbName = sprintf('Zero Cmd %d', oi);
            cb = [mc '/' cbName];
            if isempty(find_system(mc, 'SearchDepth', 1, 'Name', cbName))
                add_block('simulink/Sources/Constant', cb, 'Value', '0');
            end
            cph3 = get_param(cb, 'PortHandles');
            for dd = dsts(:)'
                add_line(mc, cph3.Outport(1), dd, 'autorouting', 'on');
            end
        end
        posCutDone = true;
        fprintf('\n[구성2 준비] 위치루프 절단 완료 (자세명령 = 0 상수)\n');
    end
    fprintf('\n--- 구성 %s: kp=%g ki=%g kd=%g filtD=%g ---\n', tag, kp_attitude, ki_attitude, kd_attitude, filtD_attitude);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        results(end+1,:) = {tag, NaN, NaN, NaN, NaN, NaN, NaN}; %#ok<SAGROW>
        continue;
    end
    t = real_roll.time(:);
    r = rad2deg(real_roll.signals.values(:));
    pch = rad2deg(real_pitch.signals.values(:));
    zv = real_z.signals.values(:);
    W = zeros(numel(t), 4);
    wvars = {w1, w2, w3, w4};
    for k = 1:4
        W(:,k) = interp1(wvars{k}.time(:), wvars{k}.signals.values(:), t, 'linear', 'extrap');
    end
    mask = t > 2;
    rmsA = sqrt(mean(r(mask).^2 + pch(mask).^2));
    maxA = max(max(abs(r(mask))), max(abs(pch(mask))));
    % FFT (roll, 100Hz 균일 리샘플, 상수 제거)
    tu = (2:0.01:T)';
    ru = interp1(t, r, tu, 'linear');
    ru = ru - mean(ru);
    nfft = numel(ru);
    fax = (0:nfft-1)' * (100/nfft);
    amp = abs(fft(ru)) * 2 / nfft;
    half = fax > 0.2 & fax < 50;
    [pkAmp, pkIdx] = max(amp(half));
    fh = fax(half);
    domF = fh(pkIdx);
    % 모터 노이즈: 회전수 절대값의 고주파 성분 (샘플간 차분 std)
    wJit = mean(std(diff(abs(W(mask,:)))));
    fprintf('  >> RMS %.3f도 / 최대 %.2f도 / 지배주파수 %.2fHz(진폭 %.3f도) / 모터지터 %.3f / z [%.2f %.2f]\n', ...
        rmsA, maxA, domF, pkAmp, wJit, min(zv), max(zv));
    results(end+1,:) = {tag, rmsA, maxA, domF, pkAmp, wJit, min(zv)}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%-26s | %7s | %6s | %9s | %9s | %8s\n', '구성', 'RMS[도]', '최대', '지배f[Hz]', '피크진폭', '모터지터');
for ci = 1:size(results,1)
    fprintf('%-26s | %7.3f | %6.2f | %9.2f | %9.3f | %8.3f\n', results{ci,1:6});
end
fprintf(['\n(해석 지침)\n' ...
    ' - 구성2(절단)에서 지터 소멸 -> 위치루프발 자세명령 지터가 원인 (위치 D/필터 손질)\n' ...
    ' - 구성3(fD100)에서 감소 -> 미분필터 과대역이 원인 (filtD_attitude 하향 채택)\n' ...
    ' - 구성4(kd-80)에서 감소하나 3에서 불변 -> kd 자체가 과함 (단 외란응답 회귀 필수)\n' ...
    ' - 전부 불변 + 지배주파수 뚜렷 -> 내루프 리미트사이클 (믹서/양자화 조사 필요)\n']);
