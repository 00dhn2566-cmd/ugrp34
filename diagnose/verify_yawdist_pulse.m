%% yaw 외란 적응 적분 검증 ③ — 강한 단발 돌풍 (사용자 요청 2026-08-22)
%% 구성: 25 s 제자리 호버. z축 토크 0.3 N·m 를 t=8s 부터 3초간만 인가 후 제거.
%%       (검증 ① 의 0.01 N·m 지속과 달리, 여기서는 '치고 빠지는' 외란이다.)
%% 보는 것:
%%   ① 피크 이탈 — 적분은 과도를 못 줄인다는 예상이 맞는지
%%   ② 외란 제거 후 복귀 속도 — 적분이 빨리 지우는지
%%   ③ ★와인드업 역효과 — 3초간 크게 쌓인 적분이 외란 제거 후 반대로 밀어
%%      되돌아 흔드는(overshoot) 부작용이 있는지. gmax 를 올릴수록 위험한 항목.
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

TAU_DIST = 0.3;    % N·m — 검증 ① 의 30배
T_DIST   = 8;      % s 인가 시각
DUR      = 3;      % s 지속 시간
T_END    = 25;

cases = { 'base', NaN; 'g1', 1; 'g3', 3; 'g5', 5 };
P = struct();

for ci = 1:size(cases,1)
    tag = cases{ci,1}; gm = cases{ci,2};
    fprintf('\n########## %s ##########\n', tag);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);

    yd_setup(mdl, T_END);
    yaw_pulse_wire(mdl, TAU_DIST, T_DIST, DUR);
    log_signals(mdl);

    if ~isnan(gm)
        assignin('base', 'yd_gmax', gm);
        qc_yawdist_apply(mdl);
    else
        fprintf('[base] 수술 없음\n');
    end

    tic; sim(mdl); el = toc;
    t  = real_yaw.time(:);
    yw = rad2deg(real_yaw.signals.values(:));
    r  = rad2deg(real_roll.signals.values(:));
    pc = rad2deg(real_pitch.signals.values(:));
    zv = real_z.signals.values(:);

    y0 = mean(yw(t > 4 & t < T_DIST));       % 외란 전 기준
    dv = yw - y0;
    tOff = T_DIST + DUR;

    pk    = max(abs(dv(t >= T_DIST & t <= tOff)));   % 인가 중 피크 이탈
    pkOff = dv(find(t >= tOff, 1));                  % 제거 시점 이탈
    ends  = mean(dv(t > T_END - 3));                 % 마지막 3s 잔차
    % 되돌림 오버슈트: 제거 후 부호가 반대로 넘어간 최대량 (와인드업 역효과 지표)
    sgn   = sign(pkOff);
    after = dv(t > tOff);
    back  = max(-sgn * after);
    if back < 0; back = 0; end
    % 복귀: 제거 후 |dv| < 2도 를 처음으로 계속 유지하는 시각
    tRec = NaN; ok = abs(dv) < 2.0; iOff = find(t > tOff, 1);
    for ii = iOff:numel(t)
        if all(ok(ii:end)); tRec = t(ii) - tOff; break; end
    end

    P.(tag) = struct('t',t,'dv',dv,'pk',pk,'pkOff',pkOff,'ends',ends, ...
                     'back',back,'tRec',tRec,'sec',el, ...
                     'att',sqrt(mean(r(t>2).^2 + pc(t>2).^2)),'z',[min(zv) max(zv)]);
    fprintf('>> %s | 피크 %7.2f도 | 제거시점 %7.2f도 | 되돌림 오버슈트 %6.2f도 | 복귀 %5.2fs | 잔차 %7.3f도 | 자세RMS %.3f | z[%.2f %.2f] | %.0fs\n', ...
            tag, pk, pkOff, back, tRec, ends, P.(tag).att, min(zv), max(zv), el);
end

%% --- 판정 ---
fprintf('\n===== 판정 (0.3 N·m x 3s 돌풍) =====\n');
tg = linspace(0, T_END, 4001)';
db = interp1(P.base.t, P.base.dv, tg, 'linear', 'extrap');
d1 = interp1(P.g1.t,   P.g1.dv,   tg, 'linear', 'extrap');
e1 = max(abs(d1 - db));
if e1 < 0.01
    v1 = '합격';
else
    v1 = '★불합격 (수술이 동작을 바꿈)';
end
fprintf('① 항등 회귀 (g1 vs base): 최대 yaw 차 %.5f도 -> %s\n', e1, v1);

fprintf('② 항목별 비교\n');
fprintf('   %-5s %9s %11s %11s %9s %10s\n', '구성','피크','제거시점','되돌림OS','복귀[s]','잔차');
tags = {'base','g1','g3','g5'};
for k = 1:numel(tags)
    s = P.(tags{k});
    fprintf('   %-5s %8.2f도 %10.2f도 %10.2f도 %8.2f %9.3f도\n', ...
            tags{k}, s.pk, s.pkOff, s.back, s.tRec, s.ends);
end
fprintf('③ 되돌림 오버슈트가 gmax 와 함께 커지면 = 와인드업 역효과. 채택 gmax 상한의 근거가 된다.\n');

save(fullfile(modelDir,'diagnose','results','verify_yawdist_pulse.mat'), '-struct', 'P');
fprintf('\n결과 저장: diagnose/results/verify_yawdist_pulse.mat\n');

%% ================= 로컬 함수 =================
function yd_setup(mdl, T)
dt = 0.01; N = round(T/dt) + 1;
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
end

function yaw_pulse_wire(mdl, amp, tOn, dur)
% diagnose_yaw_final.m 의 z축 외란 하네스 (검증된 배선) + 유한 폭 펄스
PER = 1000;                        % s — 한 번만 치도록 주기를 길게
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
        fprintf('외란 배선 통과 (방향 %d): %g N·m, t=%g~%gs (%gs 폭)\n', oi, amp, tOn, tOn+dur, dur);
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
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; ...
          'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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
