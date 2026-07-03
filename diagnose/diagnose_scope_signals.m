modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

scope = [mdl '/Scope'];

tws = find_system(scope, 'LookUnderMasks', 'all', 'BlockType', 'ToWorkspace');
for i = 1:numel(tws)
    varName = get_param(tws{i}, 'VariableName');
    ph = get_param(tws{i}, 'PortHandles');
    lineH = get_param(ph.Inport(1), 'Line');
    fprintf('--- %s (var=%s) ---\n', tws{i}, varName);
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            srcBlk = get_param(srcPortH, 'Parent');
            fprintf('  source block: %s (%s)\n', srcBlk, get_param(srcBlk,'BlockType'));
        end
    end
end

fprintf('\n=== Scope subsystem full block list (SearchDepth 1) ===\n');
b = find_system(scope, 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end
