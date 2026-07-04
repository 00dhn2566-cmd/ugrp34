modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

fprintf('=== quadcopter_package_parameters.m 변수 중 mass/inertia 관련 ===\n');
vars = who;
for i = 1:numel(vars)
    name = vars{i};
    if contains(lower(name), 'mass') || contains(lower(name), 'inert') || contains(lower(name), 'density')
        v = eval(name);
        if isnumeric(v)
            fprintf('  %s = %s\n', name, mat2str(v));
        end
    end
end

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('\n=== Body(File Solid) 블록들의 Density/Mass 파라미터 ===\n');
bodyBlocks = find_system([mdl '/Quadcopter/Body'], 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', '.*');
for i = 1:numel(bodyBlocks)
    blk = bodyBlocks{i};
    bt = get_param(blk, 'BlockType');
    if strcmp(bt, 'SimscapeBlock') || strcmp(bt,'Reference')
        try
            dp = get_param(blk, 'DialogParameters');
            fn = fieldnames(dp);
            if any(contains(fn, 'Density')) || any(contains(fn,'Mass'))
                fprintf('  %s\n', blk);
                if isfield(dp,'Density')
                    fprintf('    Density = %s %s\n', get_param(blk,'Density'), get_param(blk,'Density_unit'));
                end
                if isfield(dp,'ExtGeomUnits')
                    fprintf('    ExtGeomUnits = %s\n', get_param(blk,'ExtGeomUnits'));
                end
            end
        catch
        end
    end
end
