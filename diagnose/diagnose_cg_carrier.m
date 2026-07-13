%% 섀시 CoM y=-29.25mm의 캐리어 확정: 파라메트릭 부품 밀도 제로화 스윕
%% 방법: Inertia Sensor(World, SpanWeldJoints=off=섀시만) 기준 측정 후,
%% 후보 그룹(다리 2 / Flight Computer / 프롭 베이스 4)을 하나씩 밀도 1e-9로 꺼서 재측정.
%% 뺄셈으로 각 그룹의 실제 질량과 CoM이 그대로 나온다.
%% 규칙: 대상 미발견/설정 실패 시 error()로 즉사.

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

% drone_leg 파라미터 덤프 (압출 데이터 구조 확인용)
fprintf('=== drone_leg 구조 ===\n');
try
    fn = fieldnames(drone_leg);
    for k = 1:numel(fn)
        v = drone_leg.(fn{k});
        if isnumeric(v) && numel(v) <= 6
            fprintf('  drone_leg.%s = %s\n', fn{k}, mat2str(v, 5));
        else
            fprintf('  drone_leg.%s = [%s]\n', fn{k}, num2str(size(v)));
        end
    end
catch
    fprintf('  drone_leg 없음\n');
end

% --- 궤적/워크스페이스 ---
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

% --- 후보 그룹 정의 ---
grp = struct();
grp(1).name = '다리 2개';
grp(1).blks = { [mdl '/Quadcopter/Body/Legs/Leg Left'], [mdl '/Quadcopter/Body/Legs/Leg Right'] };
grp(2).name = 'Flight Computer';
grp(2).blks = { [mdl '/Quadcopter/Body/Body/Flight Computer'] };
grp(3).name = '프롭 베이스 4개';
grp(3).blks = { [mdl '/Quadcopter/Propeller 1/Base'], [mdl '/Quadcopter/Propeller 2/Base'], ...
                [mdl '/Quadcopter/Propeller 3/Base'], [mdl '/Quadcopter/Propeller 4/Base'] };

% 블록 존재/밀도 접근 확인 + 원래 밀도 저장
for g = 1:numel(grp)
    for b = 1:numel(grp(g).blks)
        d0 = get_param(grp(g).blks{b}, 'Density');   % 실패 시 에러로 즉사
        grp(g).dens0{b} = d0;
    end
    fprintf('그룹 [%s]: 블록 %d개 확인, Density=%s\n', grp(g).name, numel(grp(g).blks), grp(g).dens0{1});
end

% Flight Computer 지오메트리 파라미터 덤프
fprintf('\n=== Flight Computer 전체 파라미터 ===\n');
fcB = [mdl '/Quadcopter/Body/Body/Flight Computer'];
dpf = get_param(fcB, 'DialogParameters');
fnf = fieldnames(dpf);
for k = 1:numel(fnf)
    try
        v = get_param(fcB, fnf{k});
        if ischar(v) && ~isempty(v) && isempty(regexp(fnf{k}, 'Graphic|_conf', 'once'))
            fprintf('  %s = %s\n', fnf{k}, v);
        end
    catch
    end
end

% --- 링크 비활성화 + 센서 설치 (cg_measure와 동일 방식) ---
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
if isempty(armTf); error('Transform Arm 못 찾음'); end
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
ph = get_param(armTf{1}, 'PortHandles');
connPorts = [ph.LConn ph.RConn];
bodyPort = -1;
for ci = 1:numel(connPorts)
    cp = connPorts(ci);
    l = get_param(cp, 'Line');
    if l ~= -1; bodyPort = cp; end
end
if bodyPort == -1; error('Arm1 연결 포트 없음'); end

senBlk = [qcSys '/CG Sensor'];
if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', 'CG Sensor'))
    add_block('sm_lib/Body Elements/Inertia Sensor', senBlk);
end
set_param(senBlk, 'SenseCenterOfMass', 'on');
set_param(senBlk, 'SenseMass', 'on');
set_param(senBlk, 'SpanWeldJoints', 'off');   % 섀시만
set_param(senBlk, 'MeasurementFrame', 'World');
sphh = get_param(senBlk, 'PortHandles');
if get_param(sphh.LConn(1), 'Line') == -1
    add_line(qcSys, bodyPort, sphh.LConn(1), 'autorouting', 'on');
end
sphh = get_param(senBlk, 'PortHandles');
nOut = numel(sphh.RConn);
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
    if get_param(cvph.LConn(1), 'Line') == -1
        add_line(qcSys, sphh.RConn(k), cvph.LConn(1), 'autorouting', 'on');
        add_line(qcSys, cvph.Outport(1), twcph.Inport(1), 'autorouting', 'on');
    end
end

% --- 측정 함수 역할 (스크립트라 인라인 반복) ---
% 결과 저장
res_m = nan(1, numel(grp)+1);
res_c = nan(3, numel(grp)+1);

for step = 0:numel(grp)
    if step == 0
        lbl = '기준선 (전부 켬)';
    else
        lbl = sprintf('[%s] 밀도 제로화', grp(step).name);
        for b = 1:numel(grp(step).blks)
            set_param(grp(step).blks{b}, 'Density', '1e-9');
        end
    end
    fprintf('\n=== 측정 %d: %s ===\n', step, lbl);
    sim(mdl);
    for k = 1:nOut
        vn = sprintf('cg_out%d', k);
        vv = evalin('base', vn);
        vals = squeeze(vv.signals.values);
        if isvector(vals)
            res_m(step+1) = vals(end);
            fprintf('  질량 = %.5f kg\n', vals(end));
        else
            res_c(:, step+1) = vals(:, end);
            fprintf('  CoM = [ %+.5f %+.5f %+.5f ] m\n', vals(1,end), vals(2,end), vals(3,end));
        end
    end
    if step > 0
        % 복원
        for b = 1:numel(grp(step).blks)
            set_param(grp(step).blks{b}, 'Density', grp(step).dens0{b});
        end
        % 뺄셈으로 해당 그룹의 질량/CoM
        m0 = res_m(1); c0 = res_c(:,1);
        m1 = res_m(step+1); c1 = res_c(:,step+1);
        mg = m0 - m1;
        if mg > 1e-6
            cgP = (m0*c0 - m1*c1) / mg;
            fprintf('  >> [%s] 질량 = %.4f kg, CoM = [ %+.5f %+.5f %+.5f ] m\n', ...
                grp(step).name, mg, cgP(1), cgP(2), cgP(3));
            fprintf('  >> y 모멘트 기여 = %+.1f mm·kg (전체 필요량 -28.2)\n', 1000*mg*cgP(2));
        else
            fprintf('  >> [%s] 질량 변화 없음(%.2e) - 제로화가 안 먹었거나 원래 질량 미미\n', grp(step).name, mg);
        end
    end
end

fprintf('\n=== 스윕 완료 ===\n');
fprintf('기준선 섀시: m=%.4f, CoM y=%+.2f mm (목표: 범인 제거 시 y가 0쪽으로 이동)\n', res_m(1), 1000*res_c(2,1));
