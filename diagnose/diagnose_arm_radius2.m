modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Propeller 1 전체 블록 + ReferenceBlock ===\n');
b = find_system([mdl '/Quadcopter/Propeller 1'], 'LookUnderMasks', 'all');
for i = 1:numel(b)
    try
        rb = get_param(b{i}, 'ReferenceBlock');
    catch
        rb = '';
    end
    fprintf('  %s  (ref: %s)\n', b{i}, rb);
end

fprintf('\n=== Rigid Transform 블록 찾기 (ReferenceBlock 기준) ===\n');
allBlocks = find_system(mdl, 'LookUnderMasks', 'all');
for i = 1:numel(allBlocks)
    blk = allBlocks{i};
    try
        rb = get_param(blk, 'ReferenceBlock');
    catch
        rb = '';
    end
    if contains(rb, 'Rigid Transform') && (contains(blk,'Propeller') || contains(blk,'Body'))
        try
            trans = get_param(blk, 'TranslationCartesianOffset');
        catch
            trans = '?';
        end
        fprintf('  %s : Translation = %s\n', blk, trans);
    end
end
