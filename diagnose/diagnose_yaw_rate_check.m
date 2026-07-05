%% 요 "위치"가 아니라 실제 순간 각속도(6 DOF의 각속도 출력)를 직접 로깅해서,
%% 진짜로 물리적으로 초당 수백~수천 rad/s급으로 돌고 있는지, 아니면 각속도
%% 자체는 작은데 위치 표현(wrap/unwrap)만 이상한 건지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== 6 DOF 블록의 각속도 관련 Out Bus Element 찾기 ===\n');
b = find_system([mdl '/Quadcopter/6 DOF'], 'LookUnderMasks', 'all', 'BlockType', 'Outport');
for i = 1:numel(b)
    try
        el = get_param(b{i}, 'Element');
        fprintf('  %s : Element=%s\n', b{i}, el);
    catch
    end
end
