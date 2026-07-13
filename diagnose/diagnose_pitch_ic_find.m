%% pitch에 초기 각도/각속도 외란을 직접 주입하기 위해, 6 DOF/Roll Pitch 안의
%% Joint 블록들이 노출하는 초기조건(IC) 관련 마스크 파라미터를 찾는다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

dofDir = [mdl '/Quadcopter/6 DOF'];
fprintf('=== 6 DOF 안의 Joint 계열 블록 ===\n');
blks = find_system(dofDir, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', 'Joint');
for i = 1:numel(blks)
    fprintf('  %s\n', strrep(blks{i}, [dofDir '/'], ''));
end

fprintf('\n=== Joints 서브시스템 전체 블록 ===\n');
blks2 = find_system([dofDir '/Joints'], 'LookUnderMasks', 'all', 'FollowLinks', 'on');
for i = 1:numel(blks2)
    fprintf('  %s (%s)\n', strrep(blks2{i}, [dofDir '/Joints/'], ''), get_param(blks2{i}, 'BlockType'));
end

fprintf('\n=== 각 Joint 블록의 마스크 파라미터 (IC/각도/각속도 관련) ===\n');
for i = 1:numel(blks)
    blk = blks{i};
    fprintf('--- %s ---\n', strrep(blk, [dofDir '/'], ''));
    try
        mn = get_param(blk, 'MaskNames');
        mv = get_param(blk, 'MaskValues');
        for j = 1:numel(mn)
            fprintf('  %s = %s\n', mn{j}, mv{j});
        end
    catch e
        fprintf('  (마스크 조회 실패: %s)\n', e.message);
    end
end
