%% 모터 속도(meas)에 직접 걸린 Saturation 블록이 있는지, 그리고 Motor
%% Simscape 블록 자체의 정격속도/토크 파라미터 중 7420 근처 값이 있는지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Control1 내부 Saturation 블록 ===\n');
sats = find_system([mdl '/Quadcopter/Electrical/Control1'], 'LookUnderMasks', 'all', 'BlockType', 'Saturate');
for i = 1:numel(sats)
    fprintf('  %s: Upper=%s Lower=%s\n', sats{i}, get_param(sats{i},'UpperLimit'), get_param(sats{i},'LowerLimit'));
end

fprintf('\n=== Motor 1 Simscape 블록의 속도/토크 관련 파라미터 값 ===\n');
m1 = [mdl '/Quadcopter/Electrical/Motor 1'];
fields = {'w_t','w_eff','torque_max','power_max','speed0','torque_speed_param', 'w_eff_vec','T_eff_vec'};
for i = 1:numel(fields)
    try
        v = get_param(m1, fields{i});
        fprintf('  %s = %s\n', fields{i}, v);
    catch
    end
end

fprintf('\n=== quadcopter_package_parameters.m 안의 qc_motor 구조체 전체 ===\n');
disp(qc_motor);

fprintf('\n=== "Va" 관련 계산 (전압->속도 관계, back-EMF 등) 확인용 Constant/Gain 블록 ===\n');
b = find_system([mdl '/Quadcopter/Electrical'], 'LookUnderMasks', 'all', 'BlockType', 'Constant');
for i = 1:numel(b)
    try
        v = get_param(b{i}, 'Value');
        vn = str2double(v);
        if ~isnan(vn) && abs(vn) > 100
            fprintf('  %s = %s\n', b{i}, v);
        end
    catch
    end
end
