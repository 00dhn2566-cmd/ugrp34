%% yaw 외란 적응 적분 검증 ① — 지속 외란 (docs/YAW_DISTURBANCE_I.md §6-1,2,4)
%% 하네스: diagnose_yaw_final.m 과 동일 (호버 25s + t=8s부터 z축 토크 지속 인가).
%% 실행 구성:
%%   base : 수술 없음 (구운 모델 그대로)              -> 기준선
%%   g1   : 수술 + yd_gmax=1                          -> base 와 같아야 함 (항등 회귀)
%%   g3   : 수술 + yd_gmax=3
%%   g5   : 수술 + yd_gmax=5
%% 판정: ① g1 == base (수치 오차 내)  ② g3/g5 는 정상편차 감소, 호버 지터 무열화
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

TAU_DIST = 0.01;   % N·m, z축 지속 외란 (diagnose_yaw_final 과 동일 크기)
T_DIST   = 8;      % s, 인가 시각
T_END    = 25;

cases = { 'base', NaN; 'g1', 1; 'g3', 3; 'g5', 5 };
R = struct();

for ci = 1:size(cases,1)
    tag = cases{ci,1}; gm = cases{ci,2};
    fprintf('\n########## %s ##########\n', tag);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);

    yd_setup(mdl, T_END);
    yaw_disturb_wire(mdl, TAU_DIST, T_DIST);
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

    mPre  = t > 4 & t < T_DIST;         % 외란 전 (스핀업 제외)
    mPost = t > T_END - 3;              % 외란 정착 후
    base0 = mean(yw(mPre));
    ss    = mean(yw(mPost)) - base0;    % 정상편차 (외란이 남긴 yaw 오차)
    dev   = max(abs(yw(t >= T_DIST) - base0));
    jitPre = std(yw(mPre));             % 평시 호버 yaw 지터 (열화 감시)
    attRMS = sqrt(mean(r(t>2).^2 + pc(t>2).^2));

    R.(tag) = struct('t',t,'yw',yw,'ss',ss,'dev',dev,'jit',jitPre, ...
                     'att',attRMS,'z',[min(zv) max(zv)],'sec',el);
    fprintf('>> %s | 정상편차 %8.3f도 | 최대이탈 %7.3f도 | 평시지터 %.4f도 | 자세RMS %.3f도 | z[%.2f %.2f] | %.0fs\n', ...
            tag, ss, dev, jitPre, attRMS, min(zv), max(zv), el);
end

%% --- 판정 ---
fprintf('\n===== 판정 =====\n');
tg = linspace(0, T_END, 2001)';
yb = interp1(R.base.t, R.base.yw, tg, 'linear', 'extrap');
y1 = interp1(R.g1.t,   R.g1.yw,   tg, 'linear', 'extrap');
d1 = max(abs(y1 - yb));
if d1 < 0.01
    verdict1 = '합격';
else
    verdict1 = '★불합격 (수술이 동작을 바꿈)';
end
fprintf('① 항등 회귀 (g1 vs base): 최대 yaw 차 %.5f도 -> %s\n', d1, verdict1);

fprintf('② 정상편차 소거\n');
tags = {'base','g1','g3','g5'};
for k = 1:numel(tags)
    s = R.(tags{k});
    fprintf('   %-4s 정상편차 %8.3f도 (base 대비 %6.1f%%) | 평시지터 %.4f도 (base 대비 %+.4f)\n', ...
            tags{k}, s.ss, 100*abs(s.ss)/max(abs(R.base.ss),1e-9), s.jit, s.jit - R.base.jit);
end
fprintf('③ 평시 지터 열화 없어야 통과 (base 대비 +0.01도 이내)\n');

save(fullfile(modelDir,'diagnose','results','verify_yawdist.mat'), '-struct', 'R');
fprintf('\n결과 저장: diagnose/results/verify_yawdist.mat\n');

%% ================= 로컬 함수 =================
function yd_setup(mdl, T)
% 제자리 호버 궤적 (yaw 참조 0 고정)
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

function yaw_disturb_wire(mdl, amp, tOn)
% diagnose_yaw_final.m 의 z축 외란 하네스 (검증된 배선 그대로)
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
        'Amplitude', num2str(amp), 'Period','1000', 'PulseWidth','90', ...
        'PhaseDelay', num2str(tOn));
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
        fprintf('외란 배선 통과 (방향 %d), %g N·m @ t=%gs 지속\n', oi, amp, tOn);
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
