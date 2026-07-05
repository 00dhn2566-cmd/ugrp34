%% "R to X-Y-Z Extrinsic" 블록(Chassis.yaw를 만드는 원본)의 파라미터와
%% 입력(어떤 회전행렬/좌표계를 받는지) 확인. Quadcopter -> m 로 이어지는
%% yaw 계산 로직 자체를 확인하기 위함.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = [mdl '/Quadcopter/6 DOF/R to X-Y-Z Extrinsic'];
fprintf('=== R to X-Y-Z Extrinsic 블록 정보 ===\n');
fprintf('BlockType=%s\n', get_param(blk, 'BlockType'));
try
    fprintf('MaskType=%s\n', get_param(blk, 'MaskType'));
catch
end
try
    dp = get_param(blk, 'DialogParameters');
    fn = fieldnames(dp);
    for i = 1:numel(fn)
        try
            fprintf('  %s = %s\n', fn{i}, get_param(blk, fn{i}));
        catch
        end
    end
catch e
    fprintf('DialogParameters 조회 실패: %s\n', e.message);
end

fprintf('\n=== 이 블록의 입력 소스 ===\n');
ph = get_param(blk, 'PortHandles');
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  In%d <- %s\n', i, get_param(srcPortH, 'Parent'));
        end
    end
end

fprintf('\n=== 이 블록의 출력 포트들이 어디로 가는지 ===\n');
for i = 1:numel(ph.Outport)
    lineH = get_param(ph.Outport(i), 'Line');
    if lineH ~= -1
        dstPorts = get_param(lineH, 'DstPortHandle');
        for j = 1:numel(dstPorts)
            fprintf('  Out%d -> %s\n', i, get_param(dstPorts(j), 'Parent'));
        end
    end
end

fprintf('\n=== 6 DOF 서브시스템 전체 블록 목록 (구조 파악용) ===\n');
b = find_system([mdl '/Quadcopter/6 DOF'], 'LookUnderMasks', 'all', 'SearchDepth', 1);
for i = 1:numel(b)
    fprintf('  %s (%s)\n', b{i}, get_param(b{i}, 'BlockType'));
end
