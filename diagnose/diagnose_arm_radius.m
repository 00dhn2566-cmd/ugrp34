modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Body 서브시스템 안의 Rigid Transform 블록들 (Translation) ===\n');
tf = find_system([mdl '/Quadcopter/Body'], 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Transform|Rigid');
for i = 1:numel(tf)
    blk = tf{i};
    try
        rb = get_param(blk, 'ReferenceBlock');
        if contains(rb, 'Rigid Transform')
            trans = get_param(blk, 'TranslationCartesianOffset');
            fprintf('  %s : Translation = %s\n', blk, trans);
        end
    catch
    end
end

fprintf('\n=== Propeller 1~4 서브시스템 Rigid Transform (모터 위치) ===\n');
for p = 1:4
    tf2 = find_system(sprintf('%s/Quadcopter/Propeller %d', mdl, p), 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Transform|Rigid');
    for i = 1:numel(tf2)
        blk = tf2{i};
        try
            rb = get_param(blk, 'ReferenceBlock');
            if contains(rb, 'Rigid Transform')
                trans = get_param(blk, 'TranslationCartesianOffset');
                fprintf('  %s : Translation = %s\n', blk, trans);
            end
        catch
        end
    end
end
