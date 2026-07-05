modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

ctrlBlk = [mdl '/Quadcopter/Electrical/Control1/Control'];
fprintf('=== Control1/Control 블록 정보 ===\n');
fprintf('BlockType=%s\n', get_param(ctrlBlk, 'BlockType'));
try
    fprintf('MaskType=%s\n', get_param(ctrlBlk, 'MaskType'));
catch
end
try
    fprintf('P=%s\n', get_param(ctrlBlk, 'P'));
    fprintf('I=%s\n', get_param(ctrlBlk, 'I'));
    fprintf('D=%s\n', get_param(ctrlBlk, 'D'));
catch e
    fprintf('P/I/D 조회 실패: %s\n', e.message);
end

fprintf('\n=== 부모(Control1) 마스크 파라미터 ===\n');
parent = [mdl '/Quadcopter/Electrical/Control1'];
fprintf('Mask=%s\n', get_param(parent, 'Mask'));
if strcmp(get_param(parent, 'Mask'), 'on')
    fprintf('MaskNames: %s\n', strjoin(get_param(parent,'MaskNames'), ', '));
    fprintf('MaskValues: %s\n', strjoin(get_param(parent,'MaskValues'), ', '));
end

fprintf('\n=== 조부모(Electrical) 마스크 파라미터 (있다면) ===\n');
gp = [mdl '/Quadcopter/Electrical'];
fprintf('Mask=%s\n', get_param(gp, 'Mask'));
if strcmp(get_param(gp, 'Mask'), 'on')
    fprintf('MaskNames: %s\n', strjoin(get_param(gp,'MaskNames'), ', '));
    fprintf('MaskValues: %s\n', strjoin(get_param(gp,'MaskValues'), ', '));
end
