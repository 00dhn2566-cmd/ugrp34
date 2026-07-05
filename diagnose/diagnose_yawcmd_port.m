%% Position Control/Yaw Cmd로 이어지는 "In Bus Element"(Element=yaw)가
%% Port1(m, 측정값) vs Port2(Traj, 목표값) 중 어느 쪽에서 오는지 확정.

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
yawCmdBlk = [posCtrl '/Yaw Cmd'];
ph = get_param(yawCmdBlk, 'PortHandles');
lineH = get_param(ph.Inport(1), 'Line');
srcPortH = get_param(lineH, 'SrcPortHandle');
srcBlk = get_param(srcPortH, 'Parent');

fprintf('Yaw Cmd 소스 블록: %s\n', srcBlk);
fprintf('  Port 번호(내부 In Bus Element의 Port 파라미터, 어느 외부 입력에서 왔는지) = %s\n', get_param(srcBlk, 'Port'));
fprintf('  Element = %s\n', get_param(srcBlk, 'Element'));

fprintf('\n=== 비교: Position Control 외부 Inport 1(m)/2(Traj) 각각에 연결된 내부 In Bus Element들 ===\n');
allIn = find_system(posCtrl, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(allIn)
    fprintf('  %s : Port=%s, Element=%s\n', get_param(allIn{i},'Name'), get_param(allIn{i},'Port'), get_param(allIn{i},'Element'));
end

fprintf('\n=== Position Control 외부에서 본 Inport 1,2가 각각 무엇에 연결되는지 ===\n');
ph2 = get_param(posCtrl, 'PortHandles');
for i = 1:numel(ph2.Inport)
    l = get_param(ph2.Inport(i), 'Line');
    if l ~= -1
        sp = get_param(l, 'SrcPortHandle');
        if sp ~= -1
            fprintf('  외부 Inport %d <- %s\n', i, get_param(sp, 'Parent'));
        end
    end
end
