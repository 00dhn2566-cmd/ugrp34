%% PID Controller의 ExternalReset 설정 확인 + 실제 리셋 신호 값을 로깅해서
%% 시뮬레이션 내내 리셋 상태(출력 고정)인지 직접 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

posPid = [mdl '/Maneuver Controller/Position Control/PID Controller'];
fprintf('=== Position Control/PID Controller ExternalReset 설정 ===\n');
fprintf('  ExternalReset = %s\n', get_param(posPid, 'ExternalReset'));

fprintf('\n=== Reset Signal 서브시스템 내부 (있으면) ===\n');
resetSub = find_system(posPid, 'LookUnderMasks', 'all', 'Name', 'Reset Signal');
for i = 1:numel(resetSub)
    fprintf('  %s\n', resetSub{i});
    b = find_system(resetSub{i}, 'LookUnderMasks', 'all');
    for j = 1:numel(b)
        fprintf('    %s (%s)\n', b{j}, get_param(b{j}, 'BlockType'));
    end
end

fprintf('\n=== IgnoreLimit / TrackingMode 등 PID 관련 플래그 ===\n');
fields = {'IgnoreLimit', 'TrackingMode', 'LimitOutput', 'InitialConditionSource', ...
          'AntiWindupMode', 'ExternalResetSignal'};
for i = 1:numel(fields)
    try
        fprintf('  %s = %s\n', fields{i}, get_param(posPid, fields{i}));
    catch
        fprintf('  %s = (해당없음)\n', fields{i});
    end
end
