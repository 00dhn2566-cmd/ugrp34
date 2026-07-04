modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

fprintf('=== quadcopter_package_parameters.m 안의 inertia/geometry 관련 변수 ===\n');
vars = who;
for i = 1:numel(vars)
    name = vars{i};
    if contains(lower(name), 'inert') || contains(lower(name), 'arm') || ...
       contains(lower(name), 'length') || contains(lower(name), 'geom') || ...
       contains(lower(name), 'radius')
        v = eval(name);
        if isnumeric(v)
            fprintf('  %s = %s\n', name, mat2str(v));
        end
    end
end

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('\n=== Body 서브시스템 File Solid 블록의 ExtGeomUnits ===\n');
bodyBlocks = find_system([mdl '/Quadcopter/Body'], 'LookUnderMasks', 'all');
for i = 1:numel(bodyBlocks)
    blk = bodyBlocks{i};
    try
        units = get_param(blk, 'ExtGeomUnits');
        fprintf('  %s : ExtGeomUnits = %s\n', blk, units);
    catch
    end
end

fprintf('\n=== Chassis(Rigid Body) 관성 텐서 직접 조회 시도 ===\n');
try
    chassisBlk = find_system(mdl, 'LookUnderMasks','all','Name','Chassis');
    for i = 1:numel(chassisBlk)
        fprintf('  found: %s (%s)\n', chassisBlk{i}, get_param(chassisBlk{i},'BlockType'));
    end
catch e
    fprintf('  error: %s\n', e.message);
end
