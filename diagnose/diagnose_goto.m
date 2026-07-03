modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

gotoBlk = 'quadcopter_package_delivery/Goto';
fprintf('Goto tag: %s\n', get_param(gotoBlk, 'GotoTag'));

froms = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'From');
for i = 1:numel(froms)
    fprintf('From block: %s  tag=%s\n', froms{i}, get_param(froms{i}, 'GotoTag'));
end

fprintf('\n=== Motor Mixer inport sources ===\n');
blk = 'quadcopter_package_delivery/Maneuver Controller/Motor Mixer';
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

fprintf('\n=== Motor Mixer subsystem internal blocks ===\n');
b = find_system(blk, 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end
