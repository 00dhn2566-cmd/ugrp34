%% "Traj" / "Pitch Cmd" 등이 실제로 몇 번 포트인지(diagram 배치 순서가 아니라
%% 진짜 PortHandles 인덱스) 확인. linio(blk, N, ...)에서 N을 잘못 짚었을 가능성 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = [mdl '/Maneuver Controller/Position Control'];

fprintf('=== Position Control 내부 boundary Inport 블록: Port번호 <-> Name ===\n');
inBlocks = find_system(blk, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(inBlocks)
    fprintf('  Port %s : Name=%s\n', get_param(inBlocks{i}, 'Port'), get_param(inBlocks{i}, 'Name'));
end

fprintf('\n=== Position Control 내부 boundary Outport 블록: Port번호 <-> Name ===\n');
outBlocks = find_system(blk, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Outport');
for i = 1:numel(outBlocks)
    fprintf('  Port %s : Name=%s\n', get_param(outBlocks{i}, 'Port'), get_param(outBlocks{i}, 'Name'));
end

fprintf('\n=== 외부에서 본 PortHandles 순서(linio에서 쓰는 번호) ===\n');
ph = get_param(blk, 'PortHandles');
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    srcName = '(no line)';
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            srcName = get_param(srcPortH, 'Parent');
        end
    end
    fprintf('  Inport %d  <- %s\n', i, srcName);
end
for i = 1:numel(ph.Outport)
    fprintf('  Outport %d\n', i);
end
