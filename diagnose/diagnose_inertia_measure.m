%% 전체 드론 관성 텐서 실측 (Inertia Sensor) - 게인 물성 정규화(17차)용 앵커 채취
%% 목적: parameters.m의 게인을 K_thrust/K_drag/질량/관성모멘트 의존식으로 묶기 위해
%%       튜닝 당시(현재) 기체의 Ixx/Iyy/Izz 기준값을 실측한다.
%% 방법: diagnose_cg_measure.m과 동일한 부착 방식 + 관성행렬 측정 체크 추가.
%%       World 프레임, t=0 수평 -> 월드축=몸체축, CoM 기준 관성으로 해석.
%% 측정 2회: SpanWeldJoints=on (섀시+패키지 1kg = 비행 구성) / off (섀시만).
%% 규칙: 대상 미발견 시 error()로 즉사.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

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
        end
    catch
    end
    p = get_param(p, 'Parent');
end

% Arm1의 몸체 쪽 물리포트 식별
ph = get_param(armTf{1}, 'PortHandles');
connPorts = [ph.LConn ph.RConn];
bodyPort = -1; anyPort = -1;
for ci = 1:numel(connPorts)
    cp = connPorts(ci);
    l = get_param(cp, 'Line');
    if l == -1; continue; end
    cands = [get_param(l, 'SrcPortHandle'), get_param(l, 'DstPortHandle')];
    othPort = -1;
    for c2 = cands
        if c2 > 0 && c2 ~= cp; othPort = c2; end
    end
    if othPort == -1; anyPort = cp; continue; end
    othBlkPath = get_param(othPort, 'Parent');
    othName = regexprep(get_param(othBlkPath, 'Name'), '\s+', ' ');
    anyPort = cp;
    if isempty(regexp(othName, '^Arm', 'once'))
        bodyPort = cp;
    end
end
if bodyPort == -1; bodyPort = anyPort; end
if bodyPort == -1
    error('Arm1에 연결된 포트가 없음 - 실행 무효');
end

%% --- Inertia Sensor 부착 (관성행렬 측정 활성) ---
senBlk = [qcSys '/Inertia Meas Sensor'];
added = false;
for cand = {'sm_lib/Body Elements/Inertia Sensor', 'sm_lib/Sensors/Inertia Sensor'}
    try
        add_block(cand{1}, senBlk);
        added = true;
        break;
    catch
    end
end
if ~added
    error('Inertia Sensor 블록을 라이브러리에서 못 찾음 - 실행 무효');
end

% 파라미터 덤프 (측정 체크박스 이름 파악용 기록)
dps = get_param(senBlk, 'DialogParameters');
fns = fieldnames(dps);
fprintf('=== Inertia Sensor 다이얼로그 파라미터 ===\n');
for k = 1:numel(fns)
    try
        v = get_param(senBlk, fns{k});
        if ischar(v); fprintf('    %s = %s\n', fns{k}, v); end
    catch
    end
end

set_param(senBlk, 'SenseMass', 'on');
set_param(senBlk, 'SenseCenterOfMass', 'on');
% 관성 측정 체크박스: 이름 후보를 순회하며 켜지는 것을 모두 활성
inertiaParamHit = {};
for cand = {'SenseInertiaMatrix','SenseMomentsOfInertia','SenseProductsOfInertia','SenseInertia'}
    try
        set_param(senBlk, cand{1}, 'on');
        inertiaParamHit{end+1} = cand{1}; %#ok<AGROW>
    catch
    end
end
if isempty(inertiaParamHit)
    error('관성 측정 체크박스를 못 찾음 (후보 4종 전부 실패) - 위 파라미터 덤프 확인');
end
fprintf('관성 측정 활성: %s\n', strjoin(inertiaParamHit, ', '));

try
    set_param(senBlk, 'MeasurementFrame', 'World');
    fprintf('MeasurementFrame=World\n');
catch
    fprintf('MeasurementFrame=World 실패 - Attached 유지(프레임 해석 주의)\n');
end

% 프레임 포트 연결
sphh = get_param(senBlk, 'PortHandles');
add_line(qcSys, bodyPort, sphh.LConn(1), 'autorouting', 'on');

% 측정 PS 출력 -> PS-Simulink -> To Workspace
sphh = get_param(senBlk, 'PortHandles');
nOut = numel(sphh.RConn);
fprintf('센서 측정 포트(RConn): %d개\n', nOut);
for k = 1:nOut
    cvBlk = sprintf('%s/IM PS Conv %d', qcSys, k);
    add_block('nesl_utility/PS-Simulink Converter', cvBlk);
    twcBlk = sprintf('%s/To Workspace im_out%d', qcSys, k);
    add_block('simulink/Sinks/To Workspace', twcBlk, 'VariableName', sprintf('im_out%d', k), 'SaveFormat', 'StructureWithTime');
    cvph = get_param(cvBlk, 'PortHandles');
    twcph = get_param(twcBlk, 'PortHandles');
    add_line(qcSys, sphh.RConn(k), cvph.LConn(1), 'autorouting', 'on');
    add_line(qcSys, cvph.Outport(1), twcph.Inport(1), 'autorouting', 'on');
end

%% --- 실행 2회: 비행 구성(패키지 포함) / 섀시만 ---
for run_i = 1:2
    if run_i == 1
        set_param(senBlk, 'SpanWeldJoints', 'on');
        lbl = '섀시+패키지 (비행 구성, SpanWeldJoints=on)';
    else
        set_param(senBlk, 'SpanWeldJoints', 'off');
        lbl = '섀시만 (SpanWeldJoints=off)';
    end
    fprintf('\n=== 관성 측정 %d: %s ===\n', run_i, lbl);
    sim(mdl);
    for k = 1:nOut
        vn = sprintf('im_out%d', k);
        if evalin('base', sprintf('exist(''%s'',''var'')', vn))
            vv = evalin('base', vn);
            vals = squeeze(vv.signals.values);
            if isvector(vals) && numel(vals) <= 2
                fprintf('  %s(스칼라) = %+.6f\n', vn, vals(end));
            elseif ndims(vals) == 3
                M = vals(:, :, end);
                fprintf('  %s(행렬 %dx%d, 마지막 시점):\n', vn, size(M,1), size(M,2));
                for r = 1:size(M,1)
                    fprintf('    [');
                    fprintf(' %+.6e', M(r,:));
                    fprintf(' ]\n');
                end
            else
                lastcol = vals(:, end);
                fprintf('  %s(벡터 %d성분) = [', vn, size(vals,1));
                fprintf(' %+.6e', lastcol);
                fprintf(' ]\n');
            end
        end
    end
end
fprintf('\n(해석) World 프레임, t=0 수평 -> 월드축=몸체축. 관성은 CoM 기준으로 해석.\n');
fprintf('(용도) run1(비행 구성)의 Ixx/Iyy/Izz -> parameters.m 물성 앵커 I*_ref로 기입.\n');
