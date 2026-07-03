modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

elec = [mdl '/Quadcopter/Electrical'];

fprintf('=== In Bus Element 0-3 Element names ===\n');
inBusBlocks = {'In Bus Element','In Bus Element1','In Bus Element2','In Bus Element3'};
for i = 1:numel(inBusBlocks)
    blk = [elec '/' inBusBlocks{i}];
    fprintf('  %s Element=%s\n', inBusBlocks{i}, get_param(blk, 'Element'));
    ph = get_param(blk, 'PortHandles');
    lineH = get_param(ph.Outport(1), 'Line');
    if lineH ~= -1
        dstPorts = get_param(lineH, 'DstPortHandle');
        for j = 1:numel(dstPorts)
            fprintf('    -> %s\n', get_param(dstPorts(j), 'Parent'));
        end
    end
end

fprintf('\n=== Control1 internal blocks ===\n');
b = find_system([elec '/Control1'], 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== Control1/ref -> where used ===\n');
refBlk = [elec '/Control1/ref'];
ph = get_param(refBlk, 'PortHandles');
lineH = get_param(ph.Outport(1), 'Line');
if lineH ~= -1
    dstPorts = get_param(lineH, 'DstPortHandle');
    for j = 1:numel(dstPorts)
        fprintf('  ref -> %s\n', get_param(dstPorts(j), 'Parent'));
    end
end

fprintf('\n=== Control1/Add block (ref vs meas diff) ===\n');
addBlk = [elec '/Control1/Add'];
fprintf('  Inputs param: %s\n', get_param(addBlk, 'Inputs'));

fprintf('\n=== Control1/Control block type ===\n');
ctrlBlk = [elec '/Control1/Control'];
fprintf('  BlockType=%s MaskType=%s\n', get_param(ctrlBlk,'BlockType'), get_param(ctrlBlk,'MaskType'));

fprintf('\n=== Sqrt blocks anywhere in Quadcopter/Electrical or Motor Mixer ===\n');
b = find_system(elec, 'LookUnderMasks', 'all', 'BlockType', 'Sqrt');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end
b = find_system([mdl '/Maneuver Controller/Motor Mixer'], 'LookUnderMasks', 'all', 'BlockType', 'Sqrt');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== Motor 1 (Simscape) block parameters ===\n');
m1 = [mdl '/Quadcopter/Electrical/Motor 1'];
dp = get_param(m1, 'DialogParameters');
disp(fieldnames(dp));
