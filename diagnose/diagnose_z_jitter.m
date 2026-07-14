%% z축 진동 조사 + "6.87Hz 가진원 = 고도 루프" 가설 판정 (지터 수사 4차, 사용자 관찰 기반)
%% 관찰: z가 0.95~1.00 사이 상시 출렁임 + 초반 새그. 고도 PID 출력은 Z-mix로 4모터 공통 주입되므로
%%       고도 루프 링잉은 추력 리플 -> roll/pitch 지터로 전파 가능. filtD_altitude=10000(자세의 5배)도 의심.
%% 판정: 고도 게인/필터만 흔들 때 (a) z 진동과 (b) roll의 6.87Hz 피크가 함께 움직이면 가설 확정.
%% 규칙: 대상 미발견 시 error() 즉사. 구운 .slx 무수정(메모리만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;   % 오늘 확정본 (yaw 15/1.5/4, filtD_att 2000)
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

% --- 로깅 ---
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

% --- 구성: 고도 채널만 변조. {태그, kp_alt, ki_alt, kd_alt, filtD_alt} ---
cfgs = { '기준(0.5/0.1/0.3,fD10000)',  0.5, 0.1, 0.3, 10000; ...
         'kd_alt 0.6 (감쇠2배)',       0.5, 0.1, 0.6, 10000; ...
         'kd_alt 0.15 (감쇠절반)',     0.5, 0.1, 0.15, 10000; ...
         'fD_alt 1000 (필터축소)',     0.5, 0.1, 0.3, 1000 };
fprintf('\n===== z 진동/커플링 조사 (고도 채널만 변조, %d구성 x %gs) =====\n', size(cfgs,1), T);
results = {};
for ci = 1:size(cfgs, 1)
    tag = cfgs{ci,1};
    kp_altitude = cfgs{ci,2};
    ki_altitude = cfgs{ci,3};
    kd_altitude = cfgs{ci,4};
    filtD_altitude = cfgs{ci,5};
    fprintf('\n--- 구성 %s ---\n', tag);
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
    mask = t > 2;
    rmsA = sqrt(mean(r(mask).^2 + pch(mask).^2));
    % 균일 리샘플 후 FFT (roll과 z 각각)
    tu = (2:0.01:T)';
    ru = interp1(t, r, tu, 'linear'); ru = ru - mean(ru);
    zu = interp1(t, zv, tu, 'linear');
    zdet = zu - polyval(polyfit(tu, zu, 1), tu);   % z는 느린 상승 추세 제거
    nfft = numel(ru);
    fax = (0:nfft-1)' * (100/nfft);
    half = fax > 0.2 & fax < 50;
    fh = fax(half);
    ampR = abs(fft(ru)) * 2 / nfft;  ampR = ampR(half);
    ampZ = abs(fft(zdet)) * 2 / nfft; ampZ = ampZ(half);
    [pkR, iR] = max(ampR);
    [pkZ, iZ] = max(ampZ);
    % roll의 6.87Hz 대역(6.5~7.3Hz) 성분 크기 (가설의 핵심 지표)
    band = fh > 6.5 & fh < 7.3;
    r687 = max(ampR(band));
    zRms = std(zdet);
    fprintf('  >> 자세RMS %.3f도 / roll지배 %.2fHz(%.3f도) / roll@6.87Hz %.4f도 / z진동RMS %.4fm / z지배 %.2fHz(%.4fm) / z [%.3f %.3f]\n', ...
        rmsA, fh(iR), pkR, r687, zRms, fh(iZ), pkZ, min(zv), max(zv));
    results(end+1,:) = {tag, rmsA, fh(iR), r687, zRms, fh(iZ), pkZ}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%-28s | %8s | %9s | %11s | %9s | %9s\n', '구성', '자세RMS', 'roll지배f', 'roll@6.87Hz', 'z진동RMS', 'z지배f');
for ci = 1:size(results,1)
    fprintf('%-28s | %8.3f | %9.2f | %11.4f | %9.4f | %9.2f\n', results{ci,1}, results{ci,2}, results{ci,3}, results{ci,4}, results{ci,5}, results{ci,6});
end
fprintf(['\n(판정)\n' ...
    ' - 고도 게인 변조에 roll@6.87Hz가 함께 움직이면 -> 6.87Hz 가진원 = 고도 루프 확정, 고도 게인 재튜닝이 지터 처방\n' ...
    ' - z 진동만 움직이고 roll@6.87Hz 불변이면 -> z 진동과 자세 지터는 별개 문제 (모터 가설 유지)\n']);
