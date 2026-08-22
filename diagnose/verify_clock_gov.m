%% 속도 조속기 검증 ① — 항등 회귀 + 이동 중 돌풍 (SPEED_GOVERNOR.md §8 남은일 1·4)
%% 케이스 (전부 3 m / 8 s 이동, t=8~11s 에 z축 0.10 N·m 돌풍):
%%   base    : 수술 없음                      <- 기준선 (verify_yawdist_move m010_fast_base 재현)
%%   gov_off : 가상 시계 수술 + gov_on=0       <- base 와 같아야 함 (항등 회귀)
%%   gov_on  : 가상 시계 수술 + gov_on=1       <- 28 m 발산이 멎는가 = 이 설계의 판정
%% 규칙: save_system 금지. 대상 미발견 시 error() 즉사.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
qc_clock_gov_defaults;
mdl = 'quadcopter_package_delivery';

DX = 3.0; Z0 = 1.0; T0 = 5; TMOVE = 8; T_END = 25;
TG0 = 8; TGD = 3;

% 판정 창을 나눈다:
%   A상 (외란 0)      : 항등 회귀 — 발산 구간에서 항등을 재면 무의미하다.
%                       Integrator 가 연속 상태를 하나 추가해 가변스텝 솔버 배열이 바뀌고,
%                       불안정 루프가 그 1e-9 급 차이를 수십 m 로 증폭한다 (초판이 이걸로 오판).
%   B상 (0.01 N·m 지속): 조속기 효과 — 검증 ① 과 같은 '권한 이내' 조건.
%                       0.10 N·m x 3s 는 yaw 279도 이탈로 병진 루프가 이미 불안정해
%                       (yaw 잔차 ±90도 경계) 조속기로 못 막는다는 것이 실측됐다.
%          tag           외란   수술?  gov_on
cases = { 'nd_base',     0.00,  false,  0; ...
          'nd_gov_off',  0.00,  true,   0; ...
          'sd_base',     0.01,  false,  0; ...
          'sd_gov_on',   0.01,  true,   1 };
C = struct();

for ci = 1:size(cases,1)
    tag = cases{ci,1}; TAU = cases{ci,2}; doSurg = cases{ci,3}; gon = cases{ci,4};
    fprintf('\n########## %s (외란 %g N·m, 수술 %d, gov_on %d) ##########\n', tag, TAU, doSurg, gon);
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    load_system(mdl);

    move_setup(mdl, T_END, DX, Z0, T0, TMOVE);
    if TAU > 0
        yaw_pulse_wire(mdl, TAU, TG0, 900);   % 0.01 N·m 를 t=8s 부터 끝까지 지속 (검증 ① 과 동일)
    else
        fprintf('외란 없음 (항등 회귀용)\n');
    end
    log_signals(mdl);

    if doSurg
        assignin('base', 'gov_on', gon);
        qc_clock_gov_apply(mdl);
        log_gov(mdl);                     % s / tau 로깅 (수술 후에만 가능)
    else
        fprintf('[base] 수술 없음\n');
    end

    tic; sim(mdl); el = toc;
    t  = real_x.time(:);
    x  = real_x.signals.values(:);
    y  = interp1(real_y.time(:), real_y.signals.values(:), t, 'linear','extrap');
    z  = interp1(real_z.time(:), real_z.signals.values(:), t, 'linear','extrap');
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), t, 'linear','extrap'));

    % 판정 지표는 3D 로 잰다. 초판은 y(횡방향)만 봐서 x 로 13 m 날아간 것을
    % '발산 멎음' 으로 오판했다 — 축이 바뀌면 눈이 머는 지표를 쓰면 안 된다.
    tgt = [DX, 0, Z0];
    P = [x, y, z];
    devAll = max(sqrt(sum((P - tgt).^2, 2)));                 % 목표점 대비 최대 거리
    mE = t > T_END - 3;
    endErr = norm([mean(x(mE))-DX, mean(y(mE)), mean(z(mE))-Z0]);
    rec = struct('t',t,'x',x,'y',y,'z',z,'yw',yw,'sec',el, ...
                 'crossPk', max(abs(y)), 'zPk', max(abs(z(t>2)-Z0)), ...
                 'devAll', devAll, 'endErr', endErr, ...
                 'xEnd', x(end), 'yEnd', y(end), 'zEnd', z(end));
    if doSurg && exist('gov_s','var')
        gs = interp1(gov_s.time(:), gov_s.signals.values(:), t, 'linear','extrap');
        gt = interp1(gov_tau.time(:), gov_tau.signals.values(:), t, 'linear','extrap');
        rec.s = gs; rec.tau = gt;
        rec.sMin = min(gs); rec.tauEnd = gt(end);
        fprintf('   s: 최소 %.4f  (t=%.2fs)   tau(끝) %.2f s   [t=%gs 라면 %+.2f s 지연]\n', ...
                min(gs), t(find(gs==min(gs),1)), gt(end), T_END, gt(end)-T_END);
    else
        rec.sMin = NaN; rec.tauEnd = NaN;
    end
    C.(tag) = rec;

    fprintf(['>> %s | 최대이탈(3D) %8.1f cm | 종점오차 %8.1f cm | 끝 (%.2f, %.2f, %.2f) m ' ...
             '| s_min %6.4f | %.0fs\n'], tag, 100*rec.devAll, 100*rec.endErr, ...
            rec.xEnd, rec.yEnd, rec.zEnd, rec.sMin, el);
end

%% --- 판정 ---
fprintf('\n===== 판정 =====\n');
tg = linspace(0, T_END, 4001)';
gp = @(tag, f) interp1(C.(tag).t, C.(tag).(f), tg, 'linear','extrap');
d0 = max([max(abs(gp('nd_gov_off','x') - gp('nd_base','x'))), ...
          max(abs(gp('nd_gov_off','y') - gp('nd_base','y'))), ...
          max(abs(gp('nd_gov_off','z') - gp('nd_base','z')))]);
if d0 < 1e-3
    v0 = '합격';
else
    v0 = '★불합격 (수술이 동작을 바꿈)';
end
fprintf('① 항등 회귀 (외란 0, nd_gov_off vs nd_base): 최대 위치 차 %.3e m -> %s\n', d0, v0);

fprintf('② 조속기 효과 (0.01 N·m 지속)\n');
tags = {'nd_base','nd_gov_off','sd_base','sd_gov_on'};
for k = 1:numel(tags)
    r = C.(tags{k});
    fprintf('   %-11s 최대이탈 %8.1f cm | 종점오차 %8.1f cm | s_min %6.4f\n', ...
            tags{k}, 100*r.devAll, 100*r.endErr, r.sMin);
end
fprintf('   -> 종점오차 %+.1f%% (sd_gov_on vs sd_base)\n', ...
        100*(C.sd_gov_on.endErr / C.sd_base.endErr - 1));
fprintf(['   ※ 0.10 N·m x 3s (yaw 279도 이탈) 는 병진 루프가 이미 불안정한 영역이라\n' ...
         '     조속기로 못 막는다는 것이 별도 실측됨 — yaw 이탈 상한이 선행 조건.\n']);

save(fullfile(modelDir,'diagnose','results','verify_clock_gov.mat'), '-struct', 'C');
fprintf('\n결과 저장: diagnose/results/verify_clock_gov.mat\n');

%% ================= 로컬 함수 =================
function move_setup(mdl, T, dx, z0, t0, tmove)
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
fprintf('이동 궤적: %g m / %g s (피크 %.3f m/s)\n', dx, tmove, dx*pi/(2*tmove));
end

function log_gov(mdl)
% 조속기 내부 신호 s / tau 를 To Workspace 로 뽑는다 (수술 후에만 존재)
ref = get_param(find_system_one(mdl, 'QC S'), 'Parent');
pairs = {'QC S','gov_s'; 'QC Clock Int','gov_tau'};
for i = 1:size(pairs,1)
    src = [ref '/' pairs{i,1}];
    twName = ['To Workspace ' pairs{i,2}];
    old = find_system(ref, 'SearchDepth',1, 'Name', twName);
    if ~isempty(old); delete_block(old{1}); end
    tw = [ref '/' twName];
    add_block('simulink/Sinks/To Workspace', tw, ...
        'VariableName', pairs{i,2}, 'SaveFormat','StructureWithTime');
    add_line(ref, get_param(src,'PortHandles').Outport(1), ...
             get_param(tw,'PortHandles').Inport(1), 'autorouting','on');
end
end

function b = find_system_one(mdl, nm)
r = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', nm);
if numel(r) ~= 1; error('find_system_one: %s %d개', nm, numel(r)); end
b = r{1};
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
