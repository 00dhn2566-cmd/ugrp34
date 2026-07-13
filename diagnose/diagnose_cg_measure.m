%% 전체 드론 CoM 직접 측정 (Inertia Sensor)
%% 발견(10차): Transform Arm1~4는 이미 ZXZ [th 90 0]으로 X90 보정 내장 -> 팔 수직 가설 철회.
%% 남은 질문: 전체 CoM이 실제로 몇 mm 치우쳐 있는가 (추력중심 대비 5~10mm Y 치우침 예상).
%% 이 스크립트는 모델 수정 없이 Inertia Sensor만 붙여 CoM을 찍는다.
%% 규칙: 대상 미발견 시 error()로 즉사.

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

propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

% --- 궤적/워크스페이스 (모델 초기화용 최소) ---
dt = 0.01;
T = 0.05;
N = 101;
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

%% --- Transform Arm1 찾기 (센서 부착 프레임용) ---
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
armTf = {};
for i = 1:numel(allBlk)
    try
        nm1 = regexprep(get_param(allBlk{i}, 'Name'), '\s+', ' ');
    catch
        continue;
    end
    if ~isempty(regexp(nm1, '^Transform Arm\s*\d+$', 'once'))
        armTf{end+1} = allBlk{i}; %#ok<AGROW>
    end
end
fprintf('Transform Arm 블록: %d개\n', numel(armTf));
if isempty(armTf)
    error('Transform Arm을 못 찾음 - 실행 무효');
end

% 부모 체인 라이브러리 링크 비활성화
p = get_param(armTf{1}, 'Parent');
qcSys = p;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
            fprintf('  라이브러리 링크 비활성화: %s\n', strrep(p, newline, '|'));
        end
    catch
    end
    p = get_param(p, 'Parent');
end

% Arm1의 두 물리포트 조사: 몸체 쪽(솔리드 아닌 쪽) 포트를 찾는다
ph = get_param(armTf{1}, 'PortHandles');
connPorts = [ph.LConn ph.RConn];
fprintf('Arm1 물리포트 %d개\n', numel(connPorts));
bodyPort = -1; anyPort = -1;
for ci = 1:numel(connPorts)
    cp = connPorts(ci);
    l = get_param(cp, 'Line');
    if l == -1
        fprintf('  포트%d: 연결 없음\n', ci);
        continue;
    end
    cands = [get_param(l, 'SrcPortHandle'), get_param(l, 'DstPortHandle')];
    othPort = -1;
    for c2 = cands
        if c2 > 0 && c2 ~= cp
            othPort = c2;
        end
    end
    if othPort == -1
        fprintf('  포트%d: 상대 포트 핸들 식별 불가(-1)\n', ci);
        anyPort = cp;
        continue;
    end
    othBlkPath = get_param(othPort, 'Parent');
    othName = regexprep(get_param(othBlkPath, 'Name'), '\s+', ' ');
    fprintf('  포트%d 상대 = %s\n', ci, strrep(othBlkPath, newline, '|'));
    anyPort = cp;
    if isempty(regexp(othName, '^Arm', 'once'))
        bodyPort = cp;   % 상대가 솔리드(Arm*)가 아니면 몸체 쪽으로 판단
    end
end
if bodyPort == -1
    fprintf('몸체 쪽 포트 식별 실패 - 아무 포트나 사용(해석 시 프레임 주의)\n');
    bodyPort = anyPort;
end
if bodyPort == -1
    error('Arm1에 연결된 포트가 없음 - 실행 무효');
end

%% --- Inertia Sensor 부착 ---
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
    if ~added
        error('Inertia Sensor 블록을 라이브러리에서 못 찾음 - 실행 무효');
    end
end
% 파라미터 덤프 (측정 체크박스 이름 기록)
dps = get_param(senBlk, 'DialogParameters');
fns = fieldnames(dps);
for k = 1:numel(fns)
    try
        v = get_param(senBlk, fns{k});
        if ischar(v); fprintf('    [센서파라미터] %s = %s\n', fns{k}, v); end
    catch
    end
end
set_param(senBlk, 'SenseCenterOfMass', 'on');
set_param(senBlk, 'SenseMass', 'on');
fprintf('CoM/질량 측정 활성\n');
% 패키지(Weld Joint 뒤 1kg)까지 포함해서 측정
set_param(senBlk, 'SpanWeldJoints', 'on');
fprintf('SpanWeldJoints=on (패키지 포함)\n');
% 측정 프레임을 World로 (t=0 수평 상태에서 월드축=몸체축이라 해석 명확)
try
    set_param(senBlk, 'MeasurementFrame', 'World');
    fprintf('MeasurementFrame=World\n');
catch
    fprintf('MeasurementFrame=World 실패 - Attached 유지(프레임 해석 주의)\n');
end
% 가능하면 측정 범위를 전체 기계로 확대
for cand = {'Machine','WholeMachine','AllBodies','EntireMachine'}
    try
        set_param(senBlk, 'SensorExtent', cand{1});
        fprintf('SensorExtent=%s\n', cand{1});
        break;
    catch
    end
end
fprintf('SensorExtent(최종) = %s\n', get_param(senBlk, 'SensorExtent'));

sphh = get_param(senBlk, 'PortHandles');
fprintf('센서 포트: LConn %d개 / RConn %d개\n', numel(sphh.LConn), numel(sphh.RConn));

% 프레임 포트(LConn1)를 Arm1 몸체쪽 포트에 분기 연결
add_line(qcSys, bodyPort, sphh.LConn(1), 'autorouting', 'on');
fprintf('센서 프레임 연결 완료\n');

% 측정 PS 출력 -> PS-Simulink 변환 -> To Workspace (RConn 각각)
sphh = get_param(senBlk, 'PortHandles');   % 측정 활성 후 포트 갱신
nOut = numel(sphh.RConn);
fprintf('센서 측정 포트(RConn): %d개\n', nOut);
for k = 1:nOut
    cvBlk = sprintf('%s/CG PS Conv %d', qcSys, k);
    if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', sprintf('CG PS Conv %d', k)))
        add_block('nesl_utility/PS-Simulink Converter', cvBlk);
    end
    twcBlk = sprintf('%s/To Workspace cg_out%d', qcSys, k);
    if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', sprintf('To Workspace cg_out%d', k)))
        add_block('simulink/Sinks/To Workspace', twcBlk, 'VariableName', sprintf('cg_out%d', k), 'SaveFormat', 'StructureWithTime');
    end
    cvph = get_param(cvBlk, 'PortHandles');
    twcph = get_param(twcBlk, 'PortHandles');
    add_line(qcSys, sphh.RConn(k), cvph.LConn(1), 'autorouting', 'on');
    add_line(qcSys, cvph.Outport(1), twcph.Inport(1), 'autorouting', 'on');
end

%% --- 실행 + CoM 출력 (2회: 패키지 포함 / 섀시만) ---
mm = [NaN NaN];       % [질량_포함, 질량_섀시]
cc = nan(3, 2);       % CoM 열별
for run_i = 1:2
    if run_i == 1
        set_param(senBlk, 'SpanWeldJoints', 'on');
        lbl = '섀시+패키지 (SpanWeldJoints=on)';
    else
        set_param(senBlk, 'SpanWeldJoints', 'off');
        lbl = '섀시만 (SpanWeldJoints=off)';
    end
    fprintf('\n=== CoM 측정 %d: %s ===\n', run_i, lbl);
    sim(mdl);
    for k = 1:nOut
        vn = sprintf('cg_out%d', k);
        if evalin('base', sprintf('exist(''%s'',''var'')', vn))
            vv = evalin('base', vn);
            vals = squeeze(vv.signals.values);
            if isvector(vals)
                fprintf('  %s(스칼라) = %+.5f\n', vn, vals(end));
                mm(run_i) = vals(end);
            else
                lastcol = vals(:, end);
                fprintf('  %s(벡터 %d성분) = [', vn, size(vals,1));
                fprintf(' %+.5f', lastcol);
                fprintf(' ]\n');
                cc(:, run_i) = lastcol(:);
            end
        end
    end
end
if ~any(isnan(mm)) && ~any(isnan(cc(:)))
    mPkg = mm(1) - mm(2);
    cPkg = (mm(1)*cc(:,1) - mm(2)*cc(:,2)) / mPkg;
    fprintf('\n=== 뺄셈으로 구한 패키지 단독 ===\n');
    fprintf('  질량 = %.4f kg\n', mPkg);
    fprintf('  CoM  = [ %+.5f %+.5f %+.5f ] m (World)\n', cPkg(1), cPkg(2), cPkg(3));
    fprintf('  섀시만 CoM = [ %+.5f %+.5f %+.5f ] m\n', cc(1,2), cc(2,2), cc(3,2));
    fprintf('  -> y 치우침 기여: 섀시 %.1f mm·kg vs 패키지 %.1f mm·kg\n', ...
        1000*mm(2)*cc(2,2), 1000*mPkg*cPkg(2));
end
fprintf('\n(해석) World 프레임, t=0 드론 원점 수평 - CoM x,y가 곧 기하중심 대비 치우침.\n');
