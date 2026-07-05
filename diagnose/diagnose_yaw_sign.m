%% 요(yaw) 축 폭주 원인 확인: (1) 프로펠러 회전방향 설정, (2) Motor Mixer의
%% yaw 배분 부호, (3) Control Yaw의 오차 계산 부호(Add 블록 Inputs).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

fprintf('=== Propeller 1~4 Aerodynamic Propeller 블록의 회전방향 파라미터 ===\n');
for p = 1:4
    propBlk = sprintf('%s/Quadcopter/Propeller %d', mdl, p);
    b = find_system(propBlk, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', 'Name', 'Aero|Propeller$');
    for i = 1:numel(b)
        blk = b{i};
        try
            dp = get_param(blk, 'DialogParameters');
            fn = fieldnames(dp);
            match = fn(contains(lower(fn), 'direction') | contains(lower(fn), 'rotat') | contains(lower(fn), 'sense'));
            if ~isempty(match)
                fprintf('  %s:\n', blk);
                for m = 1:numel(match)
                    try
                        fprintf('    %s = %s\n', match{m}, get_param(blk, match{m}));
                    catch
                    end
                end
            end
        catch
        end
    end
end

fprintf('\n=== Motor Mixer 내부 Add4~7 (w1~w4)의 Yaw 항 부호 ===\n');
mixer = [mdl '/Maneuver Controller/Motor Mixer'];
addBlocks = {'Add4','Add5','Add6','Add7'};
for i = 1:numel(addBlocks)
    blk = [mixer '/' addBlocks{i}];
    fprintf('  %s: Inputs=%s\n', addBlocks{i}, get_param(blk, 'Inputs'));
    ph = get_param(blk, 'PortHandles');
    for j = 1:numel(ph.Inport)
        lineH = get_param(ph.Inport(j), 'Line');
        if lineH ~= -1
            srcPortH = get_param(lineH, 'SrcPortHandle');
            if srcPortH ~= -1
                srcBlk = get_param(srcPortH, 'Parent');
                tag = '';
                try
                    tag = get_param(srcBlk, 'GotoTag');
                catch
                end
                fprintf('    In%d <- %s (tag=%s)\n', j, srcBlk, tag);
            end
        end
    end
end

fprintf('\n=== Control Yaw 내부 오차 계산(Sum/Add) 부호 ===\n');
yawBlk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Control Yaw'];
sumBlocks = find_system(yawBlk, 'LookUnderMasks', 'all', 'BlockType', 'Sum');
for i = 1:numel(sumBlocks)
    try
        fprintf('  %s: Inputs=%s\n', sumBlocks{i}, get_param(sumBlocks{i}, 'Inputs'));
    catch
    end
end
