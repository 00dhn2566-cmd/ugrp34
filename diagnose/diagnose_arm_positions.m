modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Body 안 Arm1~4 / 각 Arm에 연결된 Propeller 몇 번인지 ===\n');
b = find_system([mdl '/Quadcopter/Body'], 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', '^Arm');
for i = 1:numel(b)
    blk = b{i};
    try
        rb = get_param(blk, 'ReferenceBlock');
        if contains(rb, 'Rigid Transform')
            fprintf('  %s : Translation=%s\n', blk, get_param(blk, 'TranslationCartesianOffset'));
        end
    catch
    end
end

fprintf('\n=== Connection Label(Arm-Propeller 연결) 이름들 ===\n');
b2 = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Connection Label');
for i = 1:numel(b2)
    if contains(b2{i}, 'Body') || contains(b2{i}, 'Propeller')
        fprintf('  %s (Label=%s)\n', b2{i}, get_param(b2{i}, 'Label'));
    end
end
