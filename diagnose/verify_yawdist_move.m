%% yaw 외란 적응 적분 검증 ④ — 이동 중 돌풍 (사용자 요청 2026-08-22)
%% 구성: x축 3 m 직선 이동(레이즈드 코사인) 도중 t=8~11s 에 z축 토크 0.3 N·m 인가.
%%       호버와 달리 '경로 이탈'이 판정 대상이다 — 창문 통과 여유를 직접 갉아먹는 양.
%% 케이스: 외란 두 크기 x [빠름 / 적분배율 / 느림]
%%   0.30 N·m — 검증 ③ 에서 yaw 권한 초과로 판명된 크기. 이동 중에도 같은지 확인
%%   0.10 N·m — 권한 이내로 추정. 여기서 '감속 vs 적분배율' 비교가 의미를 갖는다
%%   *_fast_base : 수술 없음, 8초 이동 (피크 0.59 m/s)   <- 기준선
%%   *_fast_g3   : gmax=3,   8초 이동
%%   *_slow_base : 수술 없음, 16초 이동 (피크 0.29 m/s)  <- '외란 중 spec 낮춤' 전략의 정량화
%% 판정: 최대 경로 이탈(횡방향 y + 고도 z), yaw 이탈, 종점 오차
%% 규칙: save_system 금지. 대상 미발견 시 error() 즉사.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
qc_yawdist_defaults;
mdl = 'quadcopter_package_delivery';

DX = 3.0;          % m, x축 이동 거리
Z0 = 1.0;          % m, 순항 고도
T0 = 5;            % s, 이동 시작
TG0 = 8; TGD = 3;  % s, 돌풍 t=8~11

%            tag              외란   이동시간  종료시각  gmax(NaN=수술없음)
cases = { 'm030_fast_base',  0.30,   8,        25,       NaN; ...
          'm030_fast_g3',    0.30,   8,        25,       3;   ...
          'm030_slow_base',  0.30,  16,        33,       NaN; ...
          'm010_fast_base',  0.10,   8,        25,       NaN; ...
          'm010_fast_g3',    0.10,   8,        25,       3;   ...
          'm010_slow_base',  0.10,  16,        33,       NaN };

M = struct();

for ci = 1:size(cases,1)
    tag = cases{ci,1}; TAU = cases{ci,2}; TMOVE = cases{ci,3};
    T_END = cases{ci,4}; gm = cases{ci,5};
    fprintf('\n########## %s (외란 %g N·m, 이동 %gs, 종료 %gs) ##########\n', tag, TAU, TMOVE, T_END);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);

    move_setup(mdl, T_END, DX, Z0, T0, TMOVE);
    yaw_pulse_wire(mdl, TAU, TG0, TGD);
    log_signals(mdl);

    if ~isnan(gm)
        assignin('base', 'yd_gmax', gm);
        qc_yawdist_apply(mdl);
    else
        fprintf('[%s] 수술 없음\n', tag);
    end

    tic; sim(mdl); el = toc;
    t  = real_x.time(:);
    x  = real_x.signals.values(:);
    y  = interp1(real_y.time(:), real_y.signals.values(:), t, 'linear', 'extrap');
    z  = interp1(real_z.time(:), real_z.signals.values(:), t, 'linear', 'extrap');
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), t, 'linear', 'extrap'));

    % 참조 경로 재구성 (레이즈드 코사인, move_setup 과 같은 식)
    u = min(max((t - T0) / TMOVE, 0), 1);
    xr = DX * 0.5 * (1 - cos(pi * u));

    mMove = t >= T0 & t <= T0 + TMOVE;          % 이동 구간
    mGust = t >= TG0 & t <= TG0 + TGD;          % 돌풍 구간
    mEnd  = t > T_END - 3;                      % 종점 정착

    y0  = mean(yw(t > 3 & t < TG0));            % 돌풍 전 yaw 기준
    M.(tag) = struct( ...
        't',t, 'x',x, 'y',y, 'z',z, 'yw',yw, 'xr',xr, 'sec',el, 'tmove',TMOVE, 'tau',TAU, ...
        'yawPk',   max(abs(yw(t >= TG0) - y0)), ...
        'yawEnd',  mean(yw(mEnd)) - y0, ...
        'crossPk', max(abs(y)), ...                              % 횡방향 이탈 (경로는 y=0)
        'crossGust', max(abs(y(mGust))), ...
        'zPk',     max(abs(z(t > 2) - Z0)), ...
        'trackPk', max(abs(x(mMove) - xr(mMove))), ...           % 진행축 추종 오차
        'endErr',  norm([mean(x(mEnd)) - DX, mean(y(mEnd)), mean(z(mEnd)) - Z0]), ...
        'pathRMS', sqrt(mean(y(mMove).^2 + (z(mMove) - Z0).^2)));

    s = M.(tag);
    fprintf(['>> %s | yaw피크 %7.2f도 잔차 %6.2f도 | 횡이탈 최대 %6.1f cm (돌풍중 %6.1f) ' ...
             '| 고도이탈 %5.1f cm | 진행축 추종 %5.1f cm | 경로RMS %5.1f cm | 종점오차 %5.1f cm | %.0fs\n'], ...
            tag, s.yawPk, s.yawEnd, 100*s.crossPk, 100*s.crossGust, 100*s.zPk, ...
            100*s.trackPk, 100*s.pathRMS, 100*s.endErr, el);
end

%% --- 판정 ---
fprintf('\n===== 판정 (이동 중 yaw 돌풍 3s) =====\n');
fprintf('  %-15s %9s %9s %10s %10s %10s %10s\n', ...
        '케이스','yaw피크','yaw잔차','횡이탈','고도이탈','경로RMS','종점오차');
tags = {'m030_fast_base','m030_fast_g3','m030_slow_base', ...
        'm010_fast_base','m010_fast_g3','m010_slow_base'};
for k = 1:numel(tags)
    s = M.(tags{k});
    fprintf('  %-15s %7.2f도 %7.2f도 %8.1fcm %8.1fcm %8.1fcm %8.1fcm\n', ...
            tags{k}, s.yawPk, s.yawEnd, 100*s.crossPk, 100*s.zPk, 100*s.pathRMS, 100*s.endErr);
end
lvs = {'m030','m010'};
for li = 1:numel(lvs)
    b  = M.([lvs{li} '_fast_base']);
    g3 = M.([lvs{li} '_fast_g3']);
    sl = M.([lvs{li} '_slow_base']);
    fprintf('\n  [%s] 적분 배율 (g3 vs base) : 횡이탈 %+.1f%%, 경로RMS %+.1f%%\n', ...
            lvs{li}, 100*(g3.crossPk/b.crossPk - 1), 100*(g3.pathRMS/b.pathRMS - 1));
    fprintf('  [%s] 감속 x0.5  (slow vs base): 횡이탈 %+.1f%%, 경로RMS %+.1f%%\n', ...
            lvs{li}, 100*(sl.crossPk/b.crossPk - 1), 100*(sl.pathRMS/b.pathRMS - 1));
end

save(fullfile(modelDir,'diagnose','results','verify_yawdist_move.mat'), '-struct', 'M');
fprintf('\n결과 저장: diagnose/results/verify_yawdist_move.mat\n');

%% ================= 로컬 함수 =================
function move_setup(mdl, T, dx, z0, t0, tmove)
% x축 dx m 직선 이동 (레이즈드 코사인 = 시종단 속도 0), yaw 참조 0 고정
dt = 0.01; N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
u = min(max((timespot_spl - t0) / tmove, 0), 1);
xs = dx * 0.5 * (1 - cos(pi * u));
spline_data = [xs, zeros(N,1), z0*ones(N,1)];
spline_yaw = zeros(N, 1);
waypoints = [0 0 z0; dx 0 z0]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));
fprintf('이동 궤적: %g m / %g s (피크 %.3f m/s), 고도 %g m\n', dx, tmove, dx*pi/(2*tmove), z0);
end

function yaw_pulse_wire(mdl, amp, tOn, dur)
PER = 1000;
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
armTf1 = '';
for i = 1:numel(allBlk)
    try
        if strcmp(strtrim(regexprep(get_param(allBlk{i},'Name'), '\s+', ' ')), 'Transform Arm1')
            armTf1 = allBlk{i};
        end
    catch
    end
end
if isempty(armTf1); error('Transform Arm1 못 찾음'); end
qcSys2 = get_param(armTf1, 'Parent');
p = qcSys2;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
bodyBlk = find_system(qcSys2, 'SearchDepth', 1, 'BlockType','SubSystem', 'Name','Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, qcSys2));
if isempty(bodyBlk); error('내부 Body 서브시스템 못 찾음'); end
bph0 = get_param(bodyBlk{1}, 'PortHandles');
bconn = [bph0.LConn bph0.RConn];
attPort = -1;
for ci = 1:numel(bconn)
    if get_param(bconn(ci), 'Line') ~= -1 && attPort == -1
        attPort = bconn(ci);
    end
end
if attPort == -1; error('Body 프레임 주입점 없음'); end

extB = [qcSys2 '/Disturb Torque Z'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb Torque Z'))
    add_block('sm_lib/Forces and Torques/External Force and Torque', extB);
end
set_param(extB, 'EnableTorqueZ', 'on');
plsB = [qcSys2 '/Disturb Pulse Z'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb Pulse Z'))
    add_block('simulink/Sources/Pulse Generator', plsB, ...
        'Amplitude', num2str(amp), 'Period', num2str(PER), ...
        'PulseWidth', num2str(100*dur/PER), 'PhaseDelay', num2str(tOn));
end
spsB = [qcSys2 '/Disturb SPS Z'];
if isempty(find_system(qcSys2,'SearchDepth',1,'Name','Disturb SPS Z'))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try
    set_param(spsB, 'Unit', 'N*m');
catch
    fprintf('SPS Unit 설정 실패\n');
end
pph = get_param(plsB,'PortHandles'); sph = get_param(spsB,'PortHandles');
if get_param(sph.Inport(1),'Line') == -1
    add_line(qcSys2, pph.Outport(1), sph.Inport(1), 'autorouting','on');
end
eph = get_param(extB,'PortHandles');
allC = [eph.LConn eph.RConn];
if numel(allC) ~= 2; error('conserving 포트 %d개(2 예상)', numel(allC)); end
orders = [2 1; 1 2]; wired = false;
for oi = 1:2
    added = [];
    try
        added(end+1) = add_line(qcSys2, attPort, allC(orders(oi,1)), 'autorouting','on'); %#ok<AGROW>
        added(end+1) = add_line(qcSys2, sph.RConn(1), allC(orders(oi,2)), 'autorouting','on'); %#ok<AGROW>
        feval(mdl, [], [], [], 'compile');
        feval(mdl, [], [], [], 'term');
        wired = true;
        fprintf('외란 배선 통과 (방향 %d): %g N·m, t=%g~%gs\n', oi, amp, tOn, tOn+dur);
        break;
    catch e
        fprintf('배선 방향 %d 실패: %s\n', oi, e.message);
        try
            feval(mdl, [], [], [], 'term');
        catch
        end
        for l2 = added
            try
                delete_line(l2);
            catch
            end
        end
    end
end
if ~wired; error('yaw 외란 배선 실패'); end
end

function log_signals(mdl)
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','real_x'; 'In Bus Element1','real_y'; 'In Bus Element2','real_z'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, ...
        'VariableName', sigMap{i,2}, 'SaveFormat','StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    add_line(scope, srcPh.Outport(1), get_param(twBlk,'PortHandles').Inport(1), 'autorouting','on');
end
end
