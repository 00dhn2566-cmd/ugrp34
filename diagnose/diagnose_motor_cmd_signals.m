%% Motor Mixer의 Out Bus Element(모터별 명령)가 어디서 오고 어디로 가는지 추적.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

mixer = [mdl '/Maneuver Controller/Motor Mixer'];

fprintf('=== Motor Mixer/Out Bus Element* 소스 (Add4~7의 결과) ===\n');
outBusBlocks = {'Out Bus Element','Out Bus Element1','Out Bus Element2','Out Bus Element3'};
for i = 1:numel(outBusBlocks)
    blk = [mixer '/' outBusBlocks{i}];
    fprintf('--- %s ---\n', blk);
    ph = get_param(blk, 'PortHandles');
    lineH = get_param(ph.Inport(1), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  source: %s\n', get_param(srcPortH, 'Parent'));
        end
    end
end

fprintf('\n=== Motor Mixer/Goto1~4 태그 ===\n');
for i = 1:4
    name = 'Goto';
    if i > 1; name = sprintf('Goto%d', i); end
    blk = [mixer '/' name];
    try
        fprintf('  %s tag=%s\n', name, get_param(blk, 'GotoTag'));
    catch e
        fprintf('  %s: %s\n', name, e.message);
    end
end

fprintf('\n=== 전체 모델에서 Motor Mixer 태그와 매칭되는 From 블록 ===\n');
froms = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'From');
for i = 1:numel(froms)
    fprintf('  %s tag=%s\n', froms{i}, get_param(froms{i}, 'GotoTag'));
end

fprintf('\n=== Quadcopter/Electrical/Motor 1 입력 소스 ===\n');
blk = [mdl '/Quadcopter/Electrical/Motor 1'];
ph = get_param(blk, 'PortHandles');
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  In%d <- %s\n', i, get_param(srcPortH, 'Parent'));
        end
    else
        fprintf('  In%d: line -1\n', i);
    end
end

fprintf('\n=== Quadcopter/Electrical 내부 블록 ===\n');
b = find_system([mdl '/Quadcopter/Electrical'], 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== Quadcopter/Electrical 서브시스템 In1 소스 (top-level) ===\n');
blk = [mdl '/Quadcopter/Electrical'];
ph = get_param(blk, 'PortHandles');
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  In%d <- %s\n', i, get_param(srcPortH, 'Parent'));
        end
    end
end

fprintf('\n=== 기존 To Workspace 블록 (act_x1 등 로그 소스 확인) ===\n');
tws = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'ToWorkspace');
for i = 1:numel(tws)
    fprintf('  %s  VariableName=%s\n', tws{i}, get_param(tws{i}, 'VariableName'));
end
