%% 2호기 교정 재설계판: 공진 체류(resonant dwell) 가진 (2026-07-19)
%% 표준 펄스(calib v1)가 실패한 이유: 에너지가 짐 모드 대역(~1.75Hz)에 없어
%% 유발 진폭 0.07~0.21도, f0 측정 불가. 이번엔 f0 후보 주파수로 x 기준을
%% 정현 가진해 무감쇠 진자를 공진 축적 -> 큰 응답에서 정밀 측정.
%%
%% 방법론 근거 (외력 주입 대신 궤적 가진인 이유): 2호기(counter_swing)가
%% 실제로 쓸 액추에이터가 "드론 가속"이므로, 교정도 같은 경로(가속->스윙
%% 이득 S, 위상)로 해야 상수가 그대로 이식된다. 외력 주입은 외란 응답
%% 시험용이지 액추에이션 교정용이 아님.
%%
%% 산출: control_seoungjin/output/swing_calib.json (schema 0.2)
%%   - f0_hz        : 자유 감쇠 꼬리의 영교차 (대진폭이라 v1과 달리 측정 가능)
%%   - S_deg_per_ms2: 공진 체류 중 사이클당 성장률 / 가진 가속 진폭
%%   - phase_lag_rad: 가진 사인 대비 응답 위상 (역위상 주입 타이밍 상수)
%%   - decay        : 꼬리 감쇠비 (댐핑 존재 여부 - 상쇄 지속시간 설계용)
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지, 투하 off.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01;
tHover = 3;          % 호버 안정
nCyc = 15;           % 공진 체류 사이클
tTail = 8;           % 자유 감쇠 꼬리 (f0 정밀 + 감쇠)
f0_nom = 1.75;       % §W 실측 후보 (스윕으로 정밀화)
aAmp = 0.15;         % 가진 가속 진폭 [m/s^2] - 저크 a*w=1.65 < 예산 2.0

load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 투하 비활성 (영구 기본 - 임무에 투하 없음)
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

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element3','real_pitch'};
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

mws = get_param(mdl, 'ModelWorkspace');

% f0 스윕: 공진이면 응답 최대 - 최대 응답 케이스의 꼬리에서 f0 정밀
freqs = [f0_nom - 0.1, f0_nom, f0_nom + 0.1];
res = {};
fprintf('===== 2호기 교정 v2: 공진 체류 가진 (a=%.2f m/s2, %d 사이클) =====\n', aAmp, nCyc);
for ci = 1:numel(freqs)
    fd = freqs(ci);
    w = 2*pi*fd;
    Ax = aAmp / w^2;                 % 위치 진폭 [m] (~1.2mm)
    tDrive = nCyc / fd;
    T = tHover + tDrive + tTail;
    N = round(T/dt) + 1;
    tt = (0:N-1)' * dt;
    xk = zeros(N,1);
    iD = tt >= tHover & tt < tHover + tDrive;
    xk(iD) = Ax * sin(w*(tt(iD) - tHover));
    % 체류 종료 후 x=0 복귀는 사인이 0 근처에서 끝나도록 nCyc 정수 사이클
    sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
    [okG, repG] = traj_gate(tt, sm, VMAX, AMAX, false, JMAX);
    if ~okG; error('교정 가진이 게이트 불통과 - aAmp 재산정'); end

    waypoints = [0 0 1; 1 0 1]';     % 시각화용 가짜 1m (서브미터 Spline 회피)
    mws.assignin('waypoints', waypoints);
    mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
    mws.assignin('timespot_spl', tt);
    mws.assignin('spline_data', sm);
    mws.assignin('spline_yaw', zeros(N,1));
    set_param(mdl, 'StopTime', num2str(T));
    fprintf('\n--- 가진 %.2fHz (Ax=%.2fmm) ---\n', fd, Ax*1000);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    tu = (0:0.002:T)';
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));

    % 체류 구간 성장률: 사이클별 피크 절대값의 선형 기울기
    grow = nan(nCyc,1);
    for k = 1:nCyc
        s = tHover + (k-1)/fd; e2 = tHover + k/fd;
        seg = pg(tu>=s & tu<e2);
        if ~isempty(seg); grow(k) = max(abs(seg - mean(pg(tu<tHover)))); end
    end
    kOK = find(~isnan(grow));
    P = polyfit(kOK, grow(kOK), 1);
    growth = P(1);                   % [도/사이클]

    % 꼬리: f0 정밀(영교차) + 감쇠 + 최종 진폭
    iT = tu >= tHover + tDrive + 0.5;
    ty = tu(iT); y = pg(iT) - mean(pg(iT));
    zc = find(abs(diff(sign(y)))>0);
    if numel(zc) >= 6
        f0m = 1/(2*mean(diff(ty(zc))));
    else
        f0m = NaN;
    end
    pk1 = max(abs(y(ty <= ty(1)+2/max(f0m,1))));
    pk2 = max(abs(y(ty >= ty(end)-2/max(f0m,1))));
    % 위상: 가진 구간 마지막 5사이클 사인 피팅 (가진 기준 t=tHover)
    iP = tu >= tHover + (nCyc-5)/fd & tu < tHover + tDrive;
    tpp = tu(iP); ypp = pg(iP) - mean(pg(iP));
    M = [sin(w*(tpp - tHover)), cos(w*(tpp - tHover))];
    ab = M \ ypp;
    phase = atan2(ab(2), ab(1));
    ampD = hypot(ab(1), ab(2));

    fprintf('  성장 %.3f도/사이클 | 정상진폭 %.2f도 | 꼬리 f0=%.3fHz, 감쇠 %0.2f (2주기 피크비)\n', ...
        growth, ampD, f0m, pk2/max(pk1, eps));
    fprintf('  위상(가진 사인 대비) %.2f rad | S = %.2f 도/(m/s2)\n', phase, ampD/aAmp);
    res(end+1,:) = {fd, growth, ampD, f0m, phase, ampD/aAmp, pk2/max(pk1,eps)}; %#ok<SAGROW>
end

fprintf('\n===== 교정 v2 결과 =====\n');
fprintf('%7s | %10s | %8s | %8s | %8s | %10s\n', ...
    '가진Hz','성장도/cyc','정상도','꼬리f0','위상rad','S도/(m/s2)');
for ci = 1:size(res,1)
    fprintf('%7.2f | %10.3f | %8.2f | %8.3f | %8.2f | %10.2f\n', res{ci,[1 2 3 4 5 6]});
end

% 최대 응답(공진) 케이스로 swing_calib.json 저장
if ~isempty(res)
    amps = cell2mat(res(:,3));
    [~, ib] = max(amps);
    calib = struct( ...
        'schema_version', '0.2', ...
        'method', 'resonant_dwell', ...
        'written_at', char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ss')), ...
        'f0_hz', res{ib,4}, ...
        'drive_freq_hz', res{ib,1}, ...
        'S_deg_per_ms2', res{ib,6}, ...
        'growth_deg_per_cycle', res{ib,2}, ...
        'phase_lag_rad', res{ib,5}, ...
        'decay_2cyc_ratio', res{ib,7}, ...
        'drive', struct('a_amp_ms2', aAmp, 'n_cycles', nCyc), ...
        'used', false);
    outDir = fullfile(fileparts(fileparts(modelDir)), 'output');
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    outPath = fullfile(outDir, 'swing_calib.json');
    fid = fopen(outPath, 'w');
    fprintf(fid, '%s', jsonencode(calib));
    fclose(fid);
    fprintf('\n[write] %s (공진 케이스 %.2fHz, S=%.2f)\n', outPath, res{ib,1}, res{ib,6});
    fprintf('(판정 기준: 정상진폭 > 0.5도면 교정 신뢰. 미달이면 aAmp 상향 재실행)\n');
end
