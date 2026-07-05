%% Aerodynamic Propeller의 direction 파라미터가 yaw 토크에 영향 없었으므로,
%% Motor(Simscape 전기모터)나 회전 조인트 쪽에 별도의 회전방향/부호 설정이
%% 있는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Propeller 1 안의 모든 블록과 회전/방향 관련 파라미터 ===\n');
b = find_system([mdl '/Quadcopter/Propeller 1'], 'LookUnderMasks', 'all', 'FollowLinks', 'on');
for i = 1:numel(b)
    blk = b{i};
    try
        dp = get_param(blk, 'DialogParameters');
        fn = fieldnames(dp);
        match = fn(contains(lower(fn), 'direction') | contains(lower(fn), 'sense') | ...
                    contains(lower(fn), 'rotat') | contains(lower(fn), 'reverse') | contains(lower(fn), 'sign'));
        if ~isempty(match)
            fprintf('  %s (%s):\n', blk, get_param(blk,'BlockType'));
            for m = 1:numel(match)
                try
                    fprintf('    %s = %s\n', match{m}, get_param(blk, match{m}));
                catch
                end
            end
        end
    catch
    end
end

fprintf('\n=== Motor Mixer의 w1~w4(Add4/5/6/7) 실제 계수(Gain) 확인 ===\n');
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
gains = find_system(mixer, 'LookUnderMasks', 'all', 'BlockType', 'Gain');
for i = 1:numel(gains)
    try
        fprintf('  %s : Gain=%s\n', gains{i}, get_param(gains{i}, 'Gain'));
    catch
    end
end
