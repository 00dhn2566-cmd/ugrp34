modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

m1 = [mdl '/Quadcopter/Electrical/Motor 1'];
fprintf('설정 전 w_t = %s\n', get_param(m1, 'w_t'));

try
    set_param(m1, 'w_t', '[0 544.875 1089.75 1162.4]');
    fprintf('set_param 호출 완료 (에러 없음)\n');
catch e
    fprintf('set_param 에러: %s\n', e.message);
end

fprintf('설정 직후 다시 읽은 w_t = %s\n', get_param(m1, 'w_t'));

fprintf('\nLinkStatus = %s\n', get_param(m1, 'LinkStatus'));
try
    fprintf('ReferenceBlock = %s\n', get_param(m1, 'ReferenceBlock'));
catch
end
