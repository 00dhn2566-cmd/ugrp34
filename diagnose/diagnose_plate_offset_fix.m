%% 회전된 FX450 하판의 평면 내 앵커 오차 보정 (B'안)
%% 현재 상태(회전 플레이트): 섀시 CoM = [+29.25, -29.25, +1001.97] mm
%% 목표(원본 플레이트 검증값): [-0.64, 0.00, +1002.73] mm
%% 방법: B <-> plate_bottom 연결에 보정 Rigid Transform 삽입.
%%   1) 프로브 이동 [10 20 5]mm -> CoM 변화로 로컬축<->월드축 대응(부호 포함) 역산
%%   2) 필요한 보정값 계산해 적용 -> 재측정으로 검증
%% 규칙: 대상 미발견 시 error() 즉사.

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

% --- 링크 비활성화 (Body 체인) ---
bb = [mdl '/Quadcopter/Body/Body'];
p = bb;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
            fprintf('링크 비활성화: %s\n', strrep(p, newline, '|'));
        end
    catch
    end
    p = get_param(p, 'Parent');
end

% --- 센서 설치 (Arm1 프레임, World, 섀시만) ---
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
    if get_param(connPorts(ci), 'Line') ~= -1; bodyPort = connPorts(ci); end
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

% --- plate_bottom의 B쪽 라인 찾기 ---
pbBlk = [bb '/plate_bottom'];
pbPh = get_param(pbBlk, 'PortHandles');
pbConn = [pbPh.LConn pbPh.RConn];
anchorPort = -1; bNodePort = -1; anchorLine = -1;
for ci = 1:numel(pbConn)
    cp = pbConn(ci);
    l = get_param(cp, 'Line');
    if l == -1; continue; end
    cands = [get_param(l,'SrcPortHandle'), get_param(l,'DstPortHandle')];
    othPort = -1;
    for c2 = cands
        if c2 > 0 && c2 ~= cp; othPort = c2; end
    end
    if othPort == -1; continue; end
    othName = strtrim(regexprep(get_param(get_param(othPort,'Parent'), 'Name'), '\s+', ' '));
    fprintf('plate_bottom 포트%d 상대 = %s\n', ci, othName);
    if strcmp(othName, 'B')
        anchorPort = cp; bNodePort = othPort; anchorLine = l;
    end
end
if anchorPort == -1
    error('plate_bottom <-> B 라인을 못 찾음 - 실행 무효');
end

% --- 보정 Rigid Transform 삽입 (초기값 = 프로브) ---
compName = 'Plate Anchor Comp';
compBlk = [bb '/' compName];
if isempty(find_system(bb, 'SearchDepth', 1, 'Name', compName))
    add_block('sm_lib/Frames and Transforms/Rigid Transform', compBlk);
end
set_param(compBlk, 'Orientation', 'right');
pbPos = get_param(pbBlk, 'Position');
set_param(compBlk, 'Position', pbPos + [-100 80 -100 80]);
set_param(compBlk, 'TranslationMethod', 'Cartesian');
set_param(compBlk, 'TranslationCartesianOffsetUnits', 'mm');

probe = [10 20 5];   % mm, 축별 구분 가능한 크기
set_param(compBlk, 'TranslationCartesianOffset', mat2str(probe));
delete_line(anchorLine);
cph2 = get_param(compBlk, 'PortHandles');
if numel(cph2.RConn) >= 1
    pB = cph2.LConn(1); pF = cph2.RConn(1);
else
    pB = cph2.LConn(1); pF = cph2.LConn(2);
end
add_line(bb, bNodePort, pB, 'autorouting', 'on');
add_line(bb, pF, anchorPort, 'autorouting', 'on');
fprintf('보정 Transform 삽입 완료 (프로브 %s mm)\n', mat2str(probe));

% --- 측정 1: 프로브 ---
c0 = [29.25, -29.25, 1001.97];        % 현재(보정 없음) 섀시 CoM, mm
target = [-0.64, 0.00, 1002.73];      % 원본 플레이트 검증값, mm
fprintf('\n=== 측정 1 (프로브 %s mm) ===\n', mat2str(probe));
sim(mdl);
c1 = [NaN NaN NaN];
for k = 1:nOut
    vv = evalin('base', sprintf('cg_out%d', k));
    vals = squeeze(vv.signals.values);
    if ~isvector(vals); c1 = 1000*vals(:, end)'; end
end
fprintf('  섀시 CoM = [ %+.2f %+.2f %+.2f ] mm\n', c1(1), c1(2), c1(3));
d = c1 - c0;
fprintf('  프로브에 의한 이동 = [ %+.2f %+.2f %+.2f ] mm\n', d(1), d(2), d(3));

% --- 축 대응(부호 포함 순열) 역산 ---
P = zeros(3);
for i = 1:3
    [~, j] = min(abs(abs(d(i)) - abs(probe)));
    P(i, j) = sign(d(i)) * sign(probe(j));
end
fprintf('  축 매핑 행렬 P (월드이동 = P * 로컬이동):\n');
disp(P);
if abs(abs(det(P)) - 1) > 0.01
    error('축 매핑이 순열이 아님(45도 회전 등) - 프로브 결과를 보고 수동 해석 필요');
end

% --- 필요 보정 계산 + 적용 ---
% 이득 보정: 스택(937g)/섀시(965g) 질량비 때문에 보정 t당 CoM은 ~0.971t만 이동
g = mean(abs(d) ./ abs(probe));
fprintf('  프로브 이득(질량비) = %.4f\n', g);
w = (target - c0)';        % 필요한 월드 이동
tReq = (P' * w)' / g;      % 로컬 좌표 변환 + 이득 보정
fprintf('  필요 월드 이동 = %s mm -> 로컬 보정값(이득보정) = %s mm\n', mat2str(round(w',2)), mat2str(round(tReq,2)));
set_param(compBlk, 'TranslationCartesianOffset', mat2str(tReq, 6));

fprintf('\n=== 측정 2 (보정 적용) ===\n');
sim(mdl);
c2 = [NaN NaN NaN];
for k = 1:nOut
    vv = evalin('base', sprintf('cg_out%d', k));
    vals = squeeze(vv.signals.values);
    if ~isvector(vals); c2 = 1000*vals(:, end)'; end
end
fprintf('  섀시 CoM = [ %+.2f %+.2f %+.2f ] mm (목표 [%+.2f %+.2f %+.2f])\n', ...
    c2(1), c2(2), c2(3), target(1), target(2), target(3));
resid = norm(c2 - target);
fprintf('  잔차 = %.2f mm ', resid);
if resid < 2
    fprintf('-> 보정 성공. 최종 보정값 = %s mm (Plate Anchor Comp, B->plate_bottom)\n', mat2str(tReq, 6));
else
    fprintf('-> 잔차 큼. P 행렬/프로브 재검토 필요\n');
end
