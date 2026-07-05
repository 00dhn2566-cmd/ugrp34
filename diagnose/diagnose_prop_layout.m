%% Propeller 1~4의 실제 배치(대각선 페어 파악)를 확인 -> 어느 2개를 반대 방향으로
%% 돌려야 하는지 결정하기 위함.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

for p = 1:4
    propBlk = sprintf('%s/Quadcopter/Propeller %d', mdl, p);
    fprintf('=== Propeller %d ===\n', p);
    b = find_system(propBlk, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', 'Transform');
    for i = 1:numel(b)
        try
            fprintf('  %s : TranslationCartesianOffset=%s\n', b{i}, get_param(b{i}, 'TranslationCartesianOffset'));
        catch e
            fprintf('  %s : %s\n', b{i}, e.message);
        end
    end
end
