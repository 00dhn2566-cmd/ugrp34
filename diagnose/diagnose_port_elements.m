modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Position Control 외부 Inport 1,2 실제 소스의 Element ===\n');
blks = {
    'quadcopter_package_delivery/Maneuver Controller/In Bus Element13'
    'quadcopter_package_delivery/Maneuver Controller/In Bus Element3'
};
for i = 1:numel(blks)
    try
        fprintf('  %s : Element=%s\n', blks{i}, get_param(blks{i}, 'Element'));
    catch e
        fprintf('  %s : %s\n', blks{i}, e.message);
    end
end

fprintf('\n=== Position Control 내부 In Bus Element* 들의 Element(무엇을 뽑는지) ===\n');
blk = [mdl '/Maneuver Controller/Position Control'];
inBlocks = find_system(blk, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(inBlocks)
    try
        fprintf('  %s (Port %s) : Element=%s\n', get_param(inBlocks{i},'Name'), get_param(inBlocks{i},'Port'), get_param(inBlocks{i},'Element'));
    catch e
        fprintf('  %s : %s\n', inBlocks{i}, e.message);
    end
end

fprintf('\n=== Position Control 출력 Pitch Cmd(Port1) 소스 (내부) ===\n');
outBlk = [blk '/Pitch Cmd'];
ph = get_param(outBlk, 'PortHandles');
lineH = get_param(ph.Inport(1), 'Line');
if lineH ~= -1
    srcPortH = get_param(lineH, 'SrcPortHandle');
    if srcPortH ~= -1
        fprintf('  source block: %s\n', get_param(srcPortH, 'Parent'));
    end
else
    fprintf('  line -1 (연결 없음?)\n');
end
