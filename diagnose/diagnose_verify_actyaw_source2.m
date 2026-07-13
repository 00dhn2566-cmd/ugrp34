%% diagnose_verify_actyaw_source.m 에서 regex 검색이 아무것도 못 찾아서(출력 0줄)
%% 이름/위치를 더 넓게 뒤져서 디버깅.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== 모델 전체에서 이름에 act_ 포함된 블록 (SearchDepth 무제한) ===\n');
b = find_system(mdl, 'RegExp', 'on', 'Name', 'act_');
for i = 1:numel(b)
    fprintf('  %s (%s)\n', b{i}, get_param(b{i}, 'BlockType'));
end

fprintf('\n=== 모델 전체에서 이름에 To Workspace 포함된 블록 개수 ===\n');
b2 = find_system(mdl, 'RegExp', 'on', 'Name', 'To Workspace');
fprintf('  총 %d개\n', numel(b2));
for i = 1:numel(b2)
    fprintf('  %s\n', b2{i});
end
