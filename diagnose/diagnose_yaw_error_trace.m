%% Control Yaw의 오차(e) 입력을 만드는 Sum/Add 블록을 찾아서, 그 두 입력
%% (ref, meas)이 각각 어디서 오는지 추적. 오차가 항상 0으로 나오는 이유 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
yawBlk = [ypr '/Control Yaw'];

fprintf('=== Control Yaw 입력의 소스 블록 ===\n');
yawPh = get_param(yawBlk, 'PortHandles');
lineH = get_param(yawPh.Inport(1), 'Line');
srcPortH = get_param(lineH, 'SrcPortHandle');
srcBlk = get_param(srcPortH, 'Parent');
fprintf('Control Yaw 입력 소스: %s (%s)\n', srcBlk, get_param(srcBlk, 'BlockType'));

if strcmp(get_param(srcBlk, 'BlockType'), 'Sum')
    fprintf('  Inputs=%s\n', get_param(srcBlk, 'Inputs'));
    sumPh = get_param(srcBlk, 'PortHandles');
    for i = 1:numel(sumPh.Inport)
        l2 = get_param(sumPh.Inport(i), 'Line');
        if l2 ~= -1
            sp2 = get_param(l2, 'SrcPortHandle');
            if sp2 ~= -1
                sb2 = get_param(sp2, 'Parent');
                tag = '';
                try
                    tag = get_param(sb2, 'GotoTag');
                catch
                end
                fprintf('    In%d <- %s (tag=%s)\n', i, sb2, tag);
            end
        end
    end
end

fprintf('\n=== YPR 서브시스템 자체의 Ref/m 입력이 무엇을 나르는지 (In Bus Element 매핑) ===\n');
inBusBlocks = find_system(ypr, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(inBusBlocks)
    try
        fprintf('  %s (Port %s): Element=%s\n', get_param(inBusBlocks{i},'Name'), get_param(inBusBlocks{i},'Port'), get_param(inBusBlocks{i}, 'Element'));
    catch
    end
end
