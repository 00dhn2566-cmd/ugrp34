%% 팔 4개 장착 Transform에 +X90 직렬 삽입 + CG 직접 측정
%% 배경(TUNING_STATUS (L)절): CAD가 roll축으로 90도 돌아간 채 저장 -> 팔 솔리드의
%% 장축(로컬 Z)이 수직으로 서 있음. 다리(Transform Leg)에는 이미 +X90 보정이 있으므로
%% 그 블록을 템플릿으로 복사해 각 팔 Transform과 솔리드 사이에 직렬 삽입한다.
%% 판정:
%%  - Inertia Sensor로 팔 회전 전/후 전체 CoM을 측정 -> 팔이 CG 치우침의 주범인지 정량 판정
%%  - 비행 재실행: 가속 전복(-22.8 @0.24s -> -79.3 @0.6s)이 줄어드는지
%% 규칙: 수정 대상 개수 무조건 출력, 기대와 다르면 error()로 즉사 (무효 실행 위장 금지)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.5;  ki_altitude = 0.1;  kd_altitude = 0.3;
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 프로펠러 물리 보정 ---
propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

% --- 추력 경로 재스케일 ---
sub = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem'];
set_param([sub '/Bias Chassis'], 'Bias', '56.5');
set_param([sub '/Bias Load'], 'Gain', '44.4*pkgSize(1)^3*pkgDensity');

% 고도 PID 출력(cmd 인포트 신호)에 ±30 클램프 삽입
cmdBlk = [sub '/cmd'];
bcBlk = [sub '/Bias Chassis'];
cph = get_param(cmdBlk, 'PortHandles');
bph = get_param(bcBlk, 'PortHandles');
oldLine = get_param(cph.Outport(1), 'Line');
if oldLine ~= -1; delete_line(oldLine); end
satBlk = [sub '/Alt Cmd Sat'];
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Alt Cmd Sat'))
    add_block('simulink/Discontinuities/Saturation', satBlk, 'UpperLimit', '30', 'LowerLimit', '-30');
end
sph = get_param(satBlk, 'PortHandles');
add_line(sub, cph.Outport(1), sph.Inport(1), 'autorouting', 'on');
add_line(sub, sph.Outport(1), bph.Inport(1), 'autorouting', 'on');

% --- 프로펠러 direction: 공장 상태(전부 Positive) 유지 ---
for p = 1:4
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    fprintf('Propeller %d direction(공장값 유지) = %s\n', p, get_param(blk, 'direction'));
end

mixer = [mdl '/Maneuver Controller/Motor Mixer'];
fprintf('Mixer 원래 부호 유지: Add4=%s Add5=%s Add6=%s Add7=%s\n', ...
    get_param([mixer '/Add4'],'Inputs'), get_param([mixer '/Add5'],'Inputs'), ...
    get_param([mixer '/Add6'],'Inputs'), get_param([mixer '/Add7'],'Inputs'));

% --- x,y만 ±0.15m saturation ---
pc = [mdl '/Maneuver Controller/Position Control'];
subBlk = [pc '/Subtract2'];
pidBlk = [pc '/PID Controller'];
subPh = get_param(subBlk, 'PortHandles');
pidPh = get_param(pidBlk, 'PortHandles');
oldLine = get_param(subPh.Outport(1), 'Line');
if oldLine ~= -1; delete_line(oldLine); end
demuxBlk = [pc '/PosErr Demux'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Demux'))
    add_block('simulink/Signal Routing/Demux', demuxBlk, 'Outputs', '3');
end
satXBlk = [pc '/PosErr Sat X'];
satYBlk = [pc '/PosErr Sat Y'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat X'))
    add_block('simulink/Discontinuities/Saturation', satXBlk, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
end
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat Y'))
    add_block('simulink/Discontinuities/Saturation', satYBlk, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
end
muxBlk = [pc '/PosErr Mux'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Mux'))
    add_block('simulink/Signal Routing/Mux', muxBlk, 'Inputs', '3');
end
demuxPh = get_param(demuxBlk, 'PortHandles');
satXPh = get_param(satXBlk, 'PortHandles');
satYPh = get_param(satYBlk, 'PortHandles');
muxPh = get_param(muxBlk, 'PortHandles');
add_line(pc, subPh.Outport(1), demuxPh.Inport(1), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(1), satXPh.Inport(1), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(2), satYPh.Inport(1), 'autorouting', 'on');
add_line(pc, satXPh.Outport(1), muxPh.Inport(1), 'autorouting', 'on');
add_line(pc, satYPh.Outport(1), muxPh.Inport(2), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(3), muxPh.Inport(3), 'autorouting', 'on');
add_line(pc, muxPh.Outport(1), pidPh.Inport(1), 'autorouting', 'on');
% Scope 분기 재연결
scopeBlk = [pc '/Scope'];
scopePh = get_param(scopeBlk, 'PortHandles');
if get_param(scopePh.Inport(2), 'Line') == -1
    add_line(pc, subPh.Outport(1), scopePh.Inport(2), 'autorouting', 'on');
end

% --- 궤적/워크스페이스 ---
dt = 0.01;
T = 1.0;
N = round(T/dt) + 1;
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

% --- 로깅 To Workspace ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'; ...
          'In Bus Element11','W1'; 'In Bus Element10','W2'; 'In Bus Element12','W3'; 'In Bus Element13','W4'; ...
          'In Bus Element6','T1'; 'In Bus Element7','T2'; 'In Bus Element8','T3'; 'In Bus Element9','T4'};
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

%% ================= (0) 팔/다리 Transform 탐색 =================
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
armTf = {}; legTf = {};
for i = 1:numel(allBlk)
    try
        nm1 = regexprep(get_param(allBlk{i}, 'Name'), '\s+', ' ');
    catch
        continue;
    end
    if ~isempty(regexp(nm1, '^Transform Arm\s*\d+$', 'once'))
        armTf{end+1} = allBlk{i}; %#ok<AGROW>
    elseif ~isempty(regexp(nm1, '^Transform Leg', 'once'))
        legTf{end+1} = allBlk{i}; %#ok<AGROW>
    end
end
fprintf('\n=== Transform Arm 블록: %d개 / Transform Leg 블록(+X90 템플릿): %d개 ===\n', numel(armTf), numel(legTf));
for i = 1:numel(armTf); fprintf('  arm[%d] %s\n', i, strrep(armTf{i}, newline, '|')); end
for i = 1:numel(legTf); fprintf('  leg[%d] %s\n', i, strrep(legTf{i}, newline, '|')); end
if numel(armTf) ~= 4 || isempty(legTf)
    error('팔 Transform 4개 + 다리 템플릿 1개 이상을 못 찾음 - 실행 무효');
end

% 부모 체인 라이브러리 링크 비활성화 (수정 가능하게)
doneLinks = {};
for k = 1:4
    p = get_param(armTf{k}, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        if ~any(strcmp(doneLinks, p))
            doneLinks{end+1} = p; %#ok<AGROW>
            try
                if strcmp(get_param(p, 'LinkStatus'), 'resolved')
                    set_param(p, 'LinkStatus', 'inactive');
                    fprintf('  라이브러리 링크 비활성화: %s\n', strrep(p, newline, '|'));
                end
            catch
            end
        end
        p = get_param(p, 'Parent');
    end
end

% 회전/평행이동 파라미터 덤프 (템플릿 다리 1개 + 팔 4개)
dumpList = [legTf(1), armTf];
for i = 1:numel(dumpList)
    fprintf('  --- 파라미터: %s\n', strrep(dumpList{i}, newline, '|'));
    dpd = get_param(dumpList{i}, 'DialogParameters');
    fnd = fieldnames(dpd);
    for k = 1:numel(fnd)
        lk = lower(fnd{k});
        if ~isempty(strfind(lk, 'rot')) || ~isempty(strfind(lk, 'trans'))
            try
                v = get_param(dumpList{i}, fnd{k});
                if ischar(v); fprintf('      %s = %s\n', fnd{k}, v); end
            catch
            end
        end
    end
end

%% ================= (1) 팔 Transform 양쪽 포트 조사 =================
armSolidPort = zeros(1,4); armSolidLine = zeros(1,4); armTfPort = zeros(1,4); armBodyPort = zeros(1,4);
for k = 1:4
    ph = get_param(armTf{k}, 'PortHandles');
    connPorts = [ph.LConn ph.RConn];
    if numel(connPorts) ~= 2
        error('Transform Arm%d: 물리포트 %d개(2개여야 함) - 실행 무효', k, numel(connPorts));
    end
    nHit = 0;
    for ci = 1:2
        cp = connPorts(ci);
        l = get_param(cp, 'Line');
        if l == -1
            fprintf('  Arm%d 포트%d: 연결 없음\n', k, ci);
            continue;
        end
        lc = get_param(l, 'LineChildren');
        sp = get_param(l, 'SrcPortHandle'); dpp = get_param(l, 'DstPortHandle');
        othPort = sp; if sp == cp; othPort = dpp; end
        othBlkPath = get_param(othPort, 'Parent');
        othName = regexprep(get_param(othBlkPath, 'Name'), '\s+', ' ');
        fprintf('  Arm%d 포트%d 상대 = %s (분기 %d개)\n', k, ci, othName, numel(lc));
        if ~isempty(regexp(othName, 'Arm', 'once')) && isempty(regexp(othName, '^Transform', 'once'))
            if ~isempty(lc)
                error('Arm%d: 솔리드 쪽 라인에 분기가 있음 - 수동 검사 필요, 실행 무효', k);
            end
            armSolidPort(k) = othPort; armSolidLine(k) = l; armTfPort(k) = cp;
            nHit = nHit + 1;
        else
            armBodyPort(k) = cp;
        end
    end
    if nHit ~= 1
        error('Arm%d: 솔리드 쪽 포트 식별 실패(%d개 매칭) - 위 상대 목록 참조, 실행 무효', k, nHit);
    end
end
fprintf('팔 4개 모두 [Transform]-[솔리드] 라인 식별 완료\n');

%% ================= (2) Inertia Sensor로 CG 측정 준비 (실패해도 계속) =================
haveSensor = false;
try
    qcSys = get_param(armTf{1}, 'Parent');
    senBlk = [qcSys '/CG Sensor'];
    if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', 'CG Sensor'))
        added = false;
        for cand = {'sm_lib/Body Elements/Inertia Sensor', 'sm_lib/Sensors/Inertia Sensor'}
            try
                add_block(cand{1}, senBlk);
                added = true;
                fprintf('Inertia Sensor 추가: %s\n', cand{1});
                break;
            catch
            end
        end
        if ~added; error('Inertia Sensor 블록을 라이브러리에서 못 찾음'); end
    end
    % 파라미터 덤프(체크박스 이름 확인용)
    dps = get_param(senBlk, 'DialogParameters');
    fns = fieldnames(dps);
    for k = 1:numel(fns)
        try
            v = get_param(senBlk, fns{k});
            if ischar(v); fprintf('    [센서파라미터] %s = %s\n', fns{k}, v); end
        catch
        end
    end
    comOn = false;
    for cand = {'SenseCenterOfMass','SenseCoM','SenseCOM','com'}
        try
            set_param(senBlk, cand{1}, 'on');
            comOn = true;
            fprintf('  CoM 측정 활성: %s=on\n', cand{1});
            break;
        catch
        end
    end
    if ~comOn; error('CoM 체크박스 이름을 못 맞춤 - 위 덤프 참조'); end
    % PS-Simulink 변환 + To Workspace
    cvBlk = [qcSys '/CG PS Conv'];
    if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', 'CG PS Conv'))
        add_block('nesl_utility/PS-Simulink Converter', cvBlk);
    end
    twcBlk = [qcSys '/To Workspace com_meas'];
    if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', 'To Workspace com_meas'))
        add_block('simulink/Sinks/To Workspace', twcBlk, 'VariableName', 'com_meas', 'SaveFormat', 'StructureWithTime');
    end
    sphh = get_param(senBlk, 'PortHandles');
    cvph = get_param(cvBlk, 'PortHandles');
    twcph = get_param(twcBlk, 'PortHandles');
    % 센서 프레임 포트(LConn1)를 Arm1 몸체쪽 포트에 분기 연결
    add_line(qcSys, armBodyPort(1), sphh.LConn(1), 'autorouting', 'on');
    % 측정 PS 출력(RConn1) -> 변환기 -> To Workspace
    add_line(qcSys, sphh.RConn(1), cvph.LConn(1), 'autorouting', 'on');
    add_line(qcSys, cvph.Outport(1), twcph.Inport(1), 'autorouting', 'on');
    haveSensor = true;
    fprintf('CG Sensor 배선 완료 (측정 프레임 = Arm1 몸체쪽 장착 프레임)\n');
catch eSen
    fprintf('CG Sensor 설정 실패(계속 진행): %s\n', eSen.message(1:min(200,end)));
end

%% ================= (3) Run A: 팔 회전 전 CG 측정 (0.02s) =================
comBefore = [NaN NaN NaN];
if haveSensor
    try
        set_param(mdl, 'StopTime', '0.02');
        sim(mdl);
        comBefore = com_meas.signals.values(1, :);
        fprintf('\n[CG 전] 팔 회전 전 CoM = [%+.4f %+.4f %+.4f] m (Arm1 프레임)\n', comBefore(1), comBefore(2), comBefore(3));
    catch eA
        fprintf('Run A 실패(계속 진행): %s\n', eA.message(1:min(200,end)));
    end
    set_param(mdl, 'StopTime', num2str(T));
end

%% ================= (4) 팔 4개에 +X90 삽입 (다리 템플릿 복사) =================
nIns = 0;
for k = 1:4
    par = get_param(armTf{k}, 'Parent');
    newName = sprintf('X90 Arm%d', k);
    newBlk = [par '/' newName];
    if isempty(find_system(par, 'SearchDepth', 1, 'Name', newName))
        add_block(legTf{1}, newBlk);
    end
    set_param(newBlk, 'Orientation', 'right');
    posA = get_param(armTf{k}, 'Position');
    set_param(newBlk, 'Position', posA + [0 70 0 70]);
    % 평행이동 제거(회전 +X90만 남김)
    trOk = false;
    for cand = {'TranslationMethod','TranslationSpecification','Method'}
        try
            set_param(newBlk, cand{1}, 'None');
            trOk = true;
            fprintf('  %s: %s=None (평행이동 제거)\n', newName, cand{1});
            break;
        catch
        end
    end
    if ~trOk
        error('%s: 평행이동 제거 실패 - 위 파라미터 덤프에서 이름 확인 필요, 실행 무효', newName);
    end
    try
        fprintf('  %s: RotationMethod=%s\n', newName, get_param(newBlk, 'RotationMethod'));
    catch
    end
    % 재배선: [Transform Arm k]-(기존라인 삭제)-[X90]-[솔리드]
    delete_line(armSolidLine(k));
    nph = get_param(newBlk, 'PortHandles');
    if numel(nph.RConn) >= 1
        pB = nph.LConn(1); pF = nph.RConn(1);
    else
        pB = nph.LConn(1); pF = nph.LConn(2);
    end
    add_line(par, armTfPort(k), pB, 'autorouting', 'on');
    add_line(par, pF, armSolidPort(k), 'autorouting', 'on');
    nIns = nIns + 1;
    fprintf('  %s 삽입 완료\n', newName);
end
if nIns ~= 4
    error('X90 삽입이 4개가 아님(%d개) - 실행 무효', nIns);
end
fprintf('=== 팔 4개 +X90 삽입 완료 ===\n');

%% ================= (5) Run B: 팔 눕힌 상태 비행 =================
fprintf('\n=== 팔 +X90 비행 테스트: 가속 전복이 줄면 팔 질량 방향이 원인 지분 보유 ===\n');
try
    simOut = sim(mdl);
    t = real_z.time(:);
    z = real_z.signals.values(:);
    r = rad2deg(real_roll.signals.values(:));
    pit = rad2deg(real_pitch.signals.values(:));
    rrad = real_roll.signals.values(:);
    t1 = T1.signals.values(:); t2 = T2.signals.values(:); t3 = T3.signals.values(:); t4 = T4.signals.values(:);

    if haveSensor
        try
            comAfter = com_meas.signals.values(1, :);
            fprintf('[CG 후] 팔 회전 후 CoM = [%+.4f %+.4f %+.4f] m (Arm1 프레임)\n', comAfter(1), comAfter(2), comAfter(3));
            if ~any(isnan(comBefore))
                dcom = comAfter - comBefore;
                fprintf('[CG 변화] delta = [%+.4f %+.4f %+.4f] m\n', dcom(1), dcom(2), dcom(3));
            end
        catch
        end
    end

    Larm = 0.14;
    dT = (t1+t3) - (t2+t4);
    Mthr = dT * Larm;

    fprintf('\n  %5s | %7s %7s %7s %7s | %8s %9s | %8s %8s %7s\n', 't','T1','T2','T3','T4','dT_roll','M_thrust','roll(deg)','pitch','z');
    for ct = 0:0.05:T
        [~, idx] = min(abs(t - ct));
        fprintf('  %5.2f | %7.3f %7.3f %7.3f %7.3f | %+8.3f %+9.4f | %8.2f %8.2f %7.3f\n', ...
            t(idx), t1(idx), t2(idx), t3(idx), t4(idx), dT(idx), Mthr(idx), r(idx), pit(idx), z(idx));
    end

    rollRate = gradient(rrad, t);
    rollAcc = gradient(rollRate, t);
    Ireq = 0.03;
    fprintf('\n=== 필요 모멘트(=I*alpha, I=0.03 가정) vs 추력 모멘트 ===\n');
    fprintf('  %5s | %9s | %9s | %s\n', 't', 'M_needed', 'M_thrust', '해석');
    for ct = [0.05 0.1 0.15 0.2 0.25 0.3 0.4 0.6 0.8 1.0]
        [~, idx] = min(abs(t - ct));
        mn_ = Ireq*rollAcc(idx); mt_ = Mthr(idx);
        if abs(mt_) > 0.5*abs(mn_) && sign(mt_)==sign(mn_)
            verdict = '추력차가 주 원인';
        else
            verdict = '추력차로 설명 불가(다른 토크원)';
        end
        fprintf('  %5.2f | %+9.4f | %+9.4f | %s\n', t(idx), mn_, mt_, verdict);
    end

    fprintf('\n[비교] 중력ON 기준선 roll: -22.8 @0.24s / -50.0 @0.42s / -79.3 @0.60s (가속 전복)\n');
    fprintf('[비교] 무중력       roll: -11.9 @0.24s / -26.9 @0.42s / -41.3 @0.60s (등속 드리프트)\n');
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(300,end)));
end
