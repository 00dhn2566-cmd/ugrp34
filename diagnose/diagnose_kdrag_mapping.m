%% Kdrag가 Aerodynamic Propeller 블록의 정확히 어느 마스크 파라미터에 매핑되는지
%% 확인 (Kthrust를 잘못 짚었던 실수를 반복하지 않기 위해, 추측 없이 직접 확인).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = [mdl '/Quadcopter/Propeller 1/Thrust and Drag/Aerodynamic Propeller'];
fprintf('=== Aerodynamic Propeller 전체 마스크 파라미터 ===\n');
mn = get_param(blk, 'MaskNames');
mv = get_param(blk, 'MaskValues');
for i = 1:numel(mn)
    fprintf('  %s = %s\n', mn{i}, mv{i});
end
