%% x/y 오차 성형 체인 실험 (사용자 설계): e -> Sat(±C) -> Rate Limiter(±R) -> 위치 PID
%% 램프 종착점 = min(|원래 오차|, C)·부호 (Sat 출력을 RL이 추종) / 미분 킥 상한 = kd x R.
%% 발산 확정 조건(1m 준계단 0.67s) 고정, C x R 5조합. yaw 동시 로깅(폭주 킥 범인 수사 겸용).
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지. 게인은 parameters.m 기본.

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

% --- Position Control 체인 링크 해제 + Rate Limiter 삽입 (Sat X/Y 직후) ---
pc = [mdl '/Maneuver Controller/Position Control'];
p = pc;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
satX = [pc '/PosErr Sat X'];
satY = [pc '/PosErr Sat Y'];
muxB = [pc '/PosErr Mux'];
hit = find_system(pc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', 'PosErr Sat X');
if isempty(hit)
    lst = find_system(pc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
    fprintf('Position Control 하위 블록 목록:\n');
    for ii = 1:numel(lst)
        fprintf('  %s\n', strtrim(regexprep(lst{ii}, '\s+', ' ')));
    end
    error('PosErr Sat X 없음 - 위 목록 확인. 실행 무효');
end
rlX = [pc '/PosErr RL X'];
rlY = [pc '/PosErr RL Y'];
muxPh = get_param(muxB, 'PortHandles');
pairs = {satX, rlX, 1; satY, rlY, 2};
for k = 1:2
    sB = pairs{k,1}; rB = pairs{k,2}; mi = pairs{k,3};
    sph = get_param(sB, 'PortHandles');
    l = get_param(sph.Outport(1), 'Line');
    if l == -1; error('%s 출력 라인 없음 - 배선 예상 불일치', sB); end
    delete_line(l);
    if isempty(find_system(pc, 'SearchDepth', 1, 'Name', get_param_name(rB)))
        add_block('simulink/Discontinuities/Rate Limiter', rB, ...
            'RisingSlewLimit', '1e6', 'FallingSlewLimit', '-1e6');
    end
    rph = get_param(rB, 'PortHandles');
    add_line(pc, sph.Outport(1), rph.Inport(1), 'autorouting', 'on');
    add_line(pc, rph.Outport(1), muxPh.Inport(mi), 'autorouting', 'on');
end
fprintf('오차 성형 체인 삽입 완료: Sat -> Rate Limiter -> PID (x,y)\n');

% --- 궤적: 1m 준계단 (0.67s 최소저크) - 발산 확정 조건 ---
dt = 0.01; T = 12; tStep = 3; A = 1.0; Tm = 0.67;
N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
tau = min(max((timespot_spl - tStep) / Tm, 0), 1);
xref = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
spline_data = [xref, zeros(N,1), ones(N,1)];
spline_yaw = zeros(N, 1);
waypoints = [0 0 1; A 0 3]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));

% --- 로깅: 위치 + 자세 + yaw + 모터 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
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

% --- 케이스: {C(클램프), R(램프 기울기 m/s)} ---
% 2차: 항력 종단속도 계산 기반 강클램프 (v_t = sqrt(m·a/k_drag): C=0.03->3.2, 0.02->2.6, 0.01->1.9 m/s)
cases = [ 0.03 0.3;
          0.02 0.25;
          0.01 0.15 ];
fprintf('\n===== 오차 성형 (Sat x RL) 스윕: 1m 준계단 0.67s (발산 조건) =====\n');
summ = {};
for ci = 1:size(cases,1)
    C = cases(ci,1); R = cases(ci,2);
    set_param(satX, 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));
    set_param(satY, 'UpperLimit', num2str(C), 'LowerLimit', num2str(-C));
    set_param(rlX, 'RisingSlewLimit', num2str(R), 'FallingSlewLimit', num2str(-R));
    set_param(rlY, 'RisingSlewLimit', num2str(R), 'FallingSlewLimit', num2str(-R));
    fprintf('\n--- 케이스 %d: C=±%.2fm, R=%.2g m/s ---\n', ci, C, R);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        summ(end+1,:) = {C, R, NaN, NaN, NaN, NaN, NaN, false}; %#ok<SAGROW>
        continue;
    end
    t = px.time(:);
    x = px.signals.values(:);
    zv = interp1(pz.time(:), pz.signals.values(:), t, 'linear', 'extrap');
    r = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), t, 'linear', 'extrap'));
    pch = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear', 'extrap'));
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), t, 'linear', 'extrap'));
    tu = (0:0.01:T)';
    xu = interp1(t, x, tu, 'linear', 'extrap');
    vu = gradient(xu, 0.01);
    for ct = [3 3.5 4 5 6 7 8 9 10 12]
        [~, i1] = min(abs(tu - ct));
        [~, i2] = min(abs(t - ct));
        fprintf('  t=%5.1f | x %7.3f v %6.2f | P %6.1f R %6.1f Y %6.1f | z %5.2f\n', ...
            tu(i1), xu(i1), vu(i1), pch(i2), r(i2), yw(i2), zv(i2));
    end
    xMax = max(abs(xu));
    xEnd = abs(xu(end) - A);
    tSet = NaN;
    okm = abs(xu - A) < 0.05;
    iS = find(tu >= tStep, 1);
    for ii = iS:numel(tu)
        if all(okm(ii:end)); tSet = tu(ii) - tStep; break; end
    end
    attPk = max(max(abs(r)), max(abs(pch)));
    ywPk = max(abs(yw));
    stab = xMax < 2 && attPk < 20 && min(zv) > 0.5;
    fprintf('  >> 최대|x| %.2f / 종점오차 %.3f / 정착 %.2fs / 자세피크 %.1f도 / yaw피크 %.2f도 / %s\n', ...
        xMax, xEnd, tSet, attPk, ywPk, tern(stab, '안정', '발산'));
    summ(end+1,:) = {C, R, xMax, xEnd, tSet, attPk, ywPk, stab}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%6s | %8s | %8s | %8s | %8s | %8s | %8s | %s\n', 'C[m]','R[m/s]','최대|x|','종점오차','정착[s]','자세[도]','yaw[도]','판정');
for ci = 1:size(summ,1)
    fprintf('%6.2f | %8.2g | %8.2f | %8.3f | %8.2f | %8.1f | %8.2f | %s\n', summ{ci,1:7}, tern(summ{ci,8},'안정','발산'));
end
fprintf('(케이스 A의 yaw피크가 크면 = 폭주 킥의 yaw 좌표변환 가설 지지 증거)\n');

function s = tern(c, a, b)
    if c; s = a; else; s = b; end
end

function nm = get_param_name(blkPath)
    ix = find(blkPath == '/', 1, 'last');
    nm = blkPath(ix+1:end);
end
