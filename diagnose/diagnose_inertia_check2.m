modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== find_system for File Solid / Rigid Body / Inertia blocks (whole model) ===\n');
allBlocks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
for i = 1:numel(allBlocks)
    blk = allBlocks{i};
    try
        rb = get_param(blk, 'ReferenceBlock');
    catch
        rb = '';
    end
    if contains(rb, 'File Solid') || contains(rb, 'Rigid Body') || contains(rb, 'Inertia') || contains(rb, 'Solid')
        fprintf('  %s   (ref: %s)\n', blk, rb);
    end
end

fprintf('\n=== File Solid 블록 ExtGeomUnits/Mass/Density 값 ===\n');
for i = 1:numel(allBlocks)
    blk = allBlocks{i};
    try
        rb = get_param(blk, 'ReferenceBlock');
    catch
        rb = '';
    end
    if contains(rb, 'File Solid')
        fprintf('--- %s ---\n', blk);
        dp = get_param(blk, 'DialogParameters');
        fn = fieldnames(dp);
        for j = 1:numel(fn)
            if contains(lower(fn{j}), 'unit') || contains(lower(fn{j}), 'density') || contains(lower(fn{j}), 'mass') || contains(lower(fn{j}), 'geomfilename') || contains(lower(fn{j}), 'scal')
                try
                    fprintf('    %s = %s\n', fn{j}, get_param(blk, fn{j}));
                catch
                end
            end
        end
    end
end
