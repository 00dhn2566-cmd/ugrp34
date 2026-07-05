%% Position Control 안에서 "Yaw Cmd" 출력이 어떻게 계산되는지(어느 블록에서
%% 나오는지, 그 소스를 계속 거슬러 올라가며) 추적. spline_yaw(항상 0)가 실제로
%% 반영되는지, 아니면 실제 측정 yaw를 그대로 베끼는 로직인지 확인.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

posCtrl = [mdl '/Maneuver Controller/Position Control'];

fprintf('=== Position Control/Yaw Cmd (Outport) 소스 ===\n');
yawCmdBlk = [posCtrl '/Yaw Cmd'];
ph = get_param(yawCmdBlk, 'PortHandles');
lineH = get_param(ph.Inport(1), 'Line');
srcPortH = get_param(lineH, 'SrcPortHandle');
srcBlk = get_param(srcPortH, 'Parent');
fprintf('소스: %s (%s)\n', srcBlk, get_param(srcBlk, 'BlockType'));

% 소스를 계속 거슬러 올라가며 추적 (최대 6단계)
curBlk = srcBlk;
for depth = 1:6
    bt = get_param(curBlk, 'BlockType');
    fprintf('  [%d] %s (%s)', depth, curBlk, bt);
    if strcmp(bt, 'Inport')
        try
            fprintf(' Element=%s', get_param(curBlk, 'Element'));
        catch
        end
        fprintf('\n');
        break
    end
    fprintf('\n');
    try
        ph2 = get_param(curBlk, 'PortHandles');
        if isempty(ph2.Inport)
            break
        end
        l2 = get_param(ph2.Inport(1), 'Line');
        if l2 == -1
            break
        end
        sp2 = get_param(l2, 'SrcPortHandle');
        if sp2 == -1
            break
        end
        curBlk = get_param(sp2, 'Parent');
    catch
        break
    end
end

fprintf('\n=== Position Control 내부 "Yaw" 관련 블록 전체 ===\n');
b = find_system(posCtrl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Yaw');
for i = 1:numel(b)
    fprintf('  %s (%s)\n', b{i}, get_param(b{i}, 'BlockType'));
end
