%% arm CAD 파일(quadcopter_drone_arm.stp) 하나가 4개 Propeller 서브시스템에
%% 어떻게 배치돼 있는지 확인: 단순 회전(90도씩)만 됐는지, 아니면 미러링(반사)까지
%% 적용됐는지. Rigid Transform의 Rotation/Translation, File Solid의 Orientation 등을
%% 전부 덤프해서 4개 arm 배치가 진짜 회전대칭인지 비교.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

for p = 1:4
    sub = sprintf('%s/Quadcopter/Propeller %d', mdl, p);
    fprintf('=== Propeller %d 내부 전체 블록 ===\n', p);
    blks = find_system(sub, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
    for i = 1:numel(blks)
        if strcmp(blks{i}, sub); continue; end
        bt = get_param(blks{i}, 'BlockType');
        fprintf('  %s (%s)\n', strrep(blks{i}, [sub '/'], ''), bt);
    end
end

fprintf('\n=== File Solid / Rigid Transform 상세 파라미터 (arm 관련) ===\n');
for p = 1:4
    sub = sprintf('%s/Quadcopter/Propeller %d', mdl, p);
    fs = find_system(sub, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', 'Solid|Arm|arm');
    rt = find_system(sub, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', 'Transform|Rigid');
    fprintf('--- Propeller %d ---\n', p);
    for i = 1:numel(fs)
        blk = fs{i};
        fprintf('  [Solid] %s\n', strrep(blk, [sub '/'], ''));
        try
            fprintf('    Filename=%s\n', get_param(blk, 'GeometryFileName'));
        catch
        end
        try
            fprintf('    Orientation=%s\n', get_param(blk, 'GeomOrientationMode'));
        catch
        end
        try
            fprintf('    RotationAngles=%s\n', get_param(blk, 'GeomRotationAngles'));
        catch
        end
        try
            fprintf('    Scale=%s\n', get_param(blk, 'GeomScale'));
        catch
        end
    end
    for i = 1:numel(rt)
        blk = rt{i};
        fprintf('  [Transform] %s\n', strrep(blk, [sub '/'], ''));
        try
            fprintf('    RotationAngles=%s\n', get_param(blk, 'RotationAngles'));
        catch
        end
        try
            fprintf('    Offset=%s\n', get_param(blk, 'Offset'));
        catch
        end
        try
            fprintf('    RotationMethod=%s\n', get_param(blk, 'RotationMethod'));
        catch
        end
    end
end

% Propeller 서브시스템 자체가 Quadcopter 상위에서 어떤 Rigid Transform으로
% 배치되는지도 확인 (4개 arm의 중심축 배치 각도)
fprintf('\n=== Quadcopter 레벨에서 Propeller 1~4를 배치하는 Rigid Transform ===\n');
qc = [mdl '/Quadcopter'];
rt2 = find_system(qc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', 'Transform|Rigid');
for i = 1:numel(rt2)
    blk = rt2{i};
    fprintf('  %s\n', strrep(blk, [qc '/'], ''));
    try
        fprintf('    RotationAngles=%s\n', get_param(blk, 'RotationAngles'));
    catch
    end
    try
        fprintf('    Offset=%s\n', get_param(blk, 'Offset'));
    catch
    end
end
