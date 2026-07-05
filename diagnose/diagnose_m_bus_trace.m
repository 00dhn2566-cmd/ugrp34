%% "m"(Chassis 상태 버스)가 최상위에서 어디서 만들어져서 Maneuver Controller까지
%% 전달되는지, 그리고 Chassis.yaw가 실제로 6DOF/센서 출력과 제대로 연결돼
%% 있는지 추적.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== 최상위 Maneuver Controller의 "m" 입력 소스 ===\n');
mc = [mdl '/Maneuver Controller'];
ph = get_param(mc, 'PortHandles');
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  Inport %d <- %s\n', i, get_param(srcPortH, 'Parent'));
        end
    end
end

fprintf('\n=== Quadcopter 서브시스템 출력(Out Bus Element들)이 뭘 담고 있는지 ===\n');
qc = [mdl '/Quadcopter'];
outs = find_system(qc, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Outport');
for i = 1:numel(outs)
    try
        fprintf('  %s (Port %s): Element=%s\n', get_param(outs{i},'Name'), get_param(outs{i},'Port'), get_param(outs{i},'Element'));
    catch
    end
end

fprintf('\n=== Chassis.yaw를 만드는 원본 센서/6DOF 블록 찾기 ===\n');
% "yaw"라는 이름의 Element를 만드는 모든 Out Bus Element 검색
allOutBus = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'Outport');
for i = 1:numel(allOutBus)
    try
        el = get_param(allOutBus{i}, 'Element');
        if contains(lower(el), 'yaw') || strcmp(el, 'yaw')
            fprintf('  %s : Element=%s\n', allOutBus{i}, el);
            ph2 = get_param(allOutBus{i}, 'PortHandles');
            lineH = get_param(ph2.Inport(1), 'Line');
            if lineH ~= -1
                srcPortH = get_param(lineH, 'SrcPortHandle');
                if srcPortH ~= -1
                    fprintf('    소스: %s\n', get_param(srcPortH, 'Parent'));
                end
            end
        end
    catch
    end
end

fprintf('\n=== "Position and Orientation" 센서 블록 파라미터 (Chassis 자세 측정) ===\n');
posOri = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Position and');
for i = 1:numel(posOri)
    fprintf('  %s (%s)\n', posOri{i}, get_param(posOri{i}, 'BlockType'));
    try
        fprintf('    SenseRotationSequence=%s RotationSequence=%s\n', ...
            get_param(posOri{i},'SenseRotationSequence'), get_param(posOri{i},'RotationSequence'));
    catch
    end
end
