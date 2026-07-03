%% 모델 안에서 "모터에 먹이는 입력"에 해당하는 신호/블록을 찾는다.
% 목표: 궤적(reference)을 넣었을 때 4개 모터 각각에 들어가는 명령(추력/PWM/rpm 등)을
% 어디서 뽑아낼 수 있는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== "Motor" 포함 블록 ===\n');
b = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Motor');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== "Propeller" 포함 블록 ===\n');
b = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Propeller');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== "PWM" 포함 블록 ===\n');
b = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'PWM');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== "Mixer" 포함 블록 ===\n');
b = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Mixer|Mixing');
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== "Maneuver Controller" 바로 아래 블록 목록 ===\n');
b = find_system([mdl '/Maneuver Controller'], 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

fprintf('\n=== "Quadcopter" 서브시스템 바로 아래 블록 목록 ===\n');
b = find_system([mdl '/Quadcopter'], 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end
