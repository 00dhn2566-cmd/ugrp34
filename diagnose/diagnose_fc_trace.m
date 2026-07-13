%% Flight Computer(638g) 장착 체인 추적 + 질량 제로화 검증
%% 확정 사항: 섀시 CoM y=-29.25mm의 캐리어는 FC(638g) -> FC가 y=-44mm에 있어야 수지가 맞음.
%% (1) FC와 Body/Body 내 Rigid Transform들의 포트 연결 전수 추적 - FC가 어느 프레임을 타는지
%% (2) FC Mass=1e-6로 끄고 섀시 CoM 재측정 - 나머지가 y=0 부근이면 최종 확정

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

% --- (1) Body/Body 내부 전 블록 포트 연결 추적 ---
bb = [mdl '/Quadcopter/Body/Body'];
inner = find_system(bb, 'LookUnderMasks','all', 'FollowLinks','on');
fprintf('=== Body/Body 내부 연결 지도 ===\n');
for i = 1:numel(inner)
    b = inner{i};
    if strcmp(b, bb); continue; end
    try
        ph = get_param(b, 'PortHandles');
    catch
        continue;
    end
    conn = [ph.LConn ph.RConn];
    if isempty(conn); continue; end
    fprintf('--- %s (물리포트 %d개)\n', strrep(b, newline, '|'), numel(conn));
    for ci = 1:numel(conn)
        cp = conn(ci);
        l = get_param(cp, 'Line');
        if l == -1
            fprintf('    포트%d: (미연결)\n', ci);
            continue;
        end
        lns = l;
        try
            ch = get_param(l, 'LineChildren');
            if ~isempty(ch); lns = [lns; ch(:)]; end
        catch
        end
        prts = [];
        for li = 1:numel(lns)
            try
                prts = [prts get_param(lns(li),'SrcPortHandle') get_param(lns(li),'DstPortHandle')]; %#ok<AGROW>
            catch
            end
        end
        prts = unique(prts(prts > 0 & prts ~= cp));
        for pi = 1:numel(prts)
            try
                fprintf('    포트%d <-> %s\n', ci, strrep(get_param(prts(pi),'Parent'), newline, '|'));
            catch
            end
        end
    end
end

% --- (2) FC 질량 제로화 후 섀시 CoM ---
% 궤적/워크스페이스
dt = 0.01; T = 0.05; N = 101;
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

% 링크 비활성화 + 센서 (기존 방식)
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
    if get_param(cp, 'Line') ~= -1; bodyPort = cp; end
end
senBlk = [qcSys '/CG Sensor'];
if isempty(find_system(qcSys, 'SearchDepth', 1, 'Name', 'CG Sensor'))
    add_block('sm_lib/Body Elements/Inertia Sensor', senBlk);
end
set_param(senBlk, 'SenseCenterOfMass', 'on');
set_param(senBlk, 'SenseMass', 'on');
set_param(senBlk, 'SpanWeldJoints', 'off');
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

fcB = [bb '/Flight Computer'];
m0 = 638;
fprintf('\n=== FC Mass 638g -> 1e-6g 제로화 후 섀시 CoM ===\n');
set_param(fcB, 'Mass', '1e-6');
sim(mdl);
for k = 1:nOut
    vn = sprintf('cg_out%d', k);
    vv = evalin('base', vn);
    vals = squeeze(vv.signals.values);
    if isvector(vals)
        mz = vals(end);
        fprintf('  질량 = %.5f kg (기준선 0.96503)\n', mz);
    else
        cz = vals(:, end);
        fprintf('  CoM = [ %+.5f %+.5f %+.5f ] m (기준선 y=-0.02925)\n', cz(1), cz(2), cz(3));
    end
end
set_param(fcB, 'Mass', num2str(m0));
% FC 단독 역산
try
    mFC = 0.96503 - mz;
    cFC = (0.96503*[0;-0.02925;0.97276] - mz*cz) / mFC;
    fprintf('\n>> FC 단독: 질량 = %.4f kg, CoM = [ %+.5f %+.5f %+.5f ] m (World)\n', mFC, cFC(1), cFC(2), cFC(3));
    fprintf('>> 예측 y=-44mm와 비교할 것. 나머지 섀시 y=%+.2f mm (0 부근이면 FC 단독범 확정)\n', 1000*cz(2));
catch
end
