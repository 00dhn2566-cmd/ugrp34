modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

mixer = [mdl '/Maneuver Controller/Motor Mixer'];
outBusBlocks = {'Out Bus Element','Out Bus Element1','Out Bus Element2','Out Bus Element3'};
for i = 1:numel(outBusBlocks)
    blk = [mixer '/' outBusBlocks{i}];
    dp = get_param(blk, 'DialogParameters');
    fn = fieldnames(dp);
    fprintf('--- %s available params: %s\n', outBusBlocks{i}, strjoin(fn, ', '));
end

fprintf('\n=== Element param values ===\n');
for i = 1:numel(outBusBlocks)
    blk = [mixer '/' outBusBlocks{i}];
    try
        fprintf('  %s Element=%s\n', outBusBlocks{i}, get_param(blk,'Element'));
    catch e
        fprintf('  %s: %s\n', outBusBlocks{i}, e.message);
    end
end

fprintf('\n=== Quadcopter/In Bus Element1 (top level) ===\n');
try
    fprintf('  Element=%s\n', get_param([mdl '/Quadcopter/In Bus Element1'], 'Element'));
catch e
    fprintf('  %s\n', e.message);
end

fprintf('\n=== Quadcopter/Electrical 내부 (LookUnderMasks all) ===\n');
b = find_system([mdl '/Quadcopter/Electrical'], 'LookUnderMasks', 'all');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end
