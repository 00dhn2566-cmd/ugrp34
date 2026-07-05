%% Motor 1의 w_t=[0,3750,7500,8000] 파라미터가 어디서 설정되는지(변수 참조인지
%% 블록에 박힌 리터럴인지) 확인.

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
fprintf('=== Motor 1 관련 파라미터 원본 표현식 ===\n');
fields = {'w_t','T_t','T_t_intermittent','torque_max','power_max','w_eff_vec','T_eff_vec','torque_max_intermittent','power_max_intermittent'};
for i = 1:numel(fields)
    try
        fprintf('  %s = %s\n', fields{i}, get_param(m1, fields{i}));
    catch
    end
end

fprintf('\n=== Motor 2/3/4도 동일한지 확인 ===\n');
for p = 2:4
    mp = sprintf('%s/Quadcopter/Electrical/Motor %d', mdl, p);
    fprintf('  Motor %d w_t = %s\n', p, get_param(mp, 'w_t'));
end
