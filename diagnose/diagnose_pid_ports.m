%% Control Pitch/PID Compensator Formula 블록의 입력 포트 연결 상태 진단
% "has one or more input signal ports with no explicit line connections"
% 에러 원인을 찾기 위해 포트/라인 연결 상태를 직접 조회한다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

blk = [mdl '/Maneuver Controller/Altitude and  YPR Control/Control Pitch/PID Compensator Formula'];

outerBlk = get_param(blk, 'Parent');   % 'Control Pitch' itself
fprintf('=== Outer block: %s ===\n', outerBlk);
fprintf('BlockType: %s, Mask: %s, MaskType: %s\n', ...
    get_param(outerBlk, 'BlockType'), get_param(outerBlk, 'Mask'), get_param(outerBlk, 'MaskType'));
phOuter = get_param(outerBlk, 'PortHandles');
fprintf('Outer Inport count: %d, Outport count: %d\n', numel(phOuter.Inport), numel(phOuter.Outport));

fprintf('\n=== Inner block (PID Compensator Formula variant): %s ===\n', blk);
fprintf('BlockType: %s\n', get_param(blk, 'BlockType'));
ph = get_param(blk, 'PortHandles');
fprintf('Inport count: %d, Outport count: %d\n', numel(ph.Inport), numel(ph.Outport));

fprintf('\n=== Is this a Variant Subsystem? ===\n');
try
    fprintf('Variant: %s\n', get_param(blk, 'Variant'));
catch e
    fprintf('(no Variant param: %s)\n', e.message);
end

for i = 1:numel(ph.Inport)
    portH = ph.Inport(i);
    lineH = get_param(portH, 'Line');
    fprintf('--- Inport %d (handle %g) ---\n', i, portH);
    if lineH == -1
        fprintf('  Line: NONE (unconnected)\n');
    else
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH == -1
            fprintf('  Line handle %g exists but SrcPortHandle = -1 (no source!)\n', lineH);
        else
            srcBlock = get_param(srcPortH, 'Parent');
            fprintf('  Line handle %g, source block: %s\n', lineH, srcBlock);
        end
    end
end

fprintf('\n=== Outer block ports (should be the real PID I/O) ===\n');
for i = 1:numel(phOuter.Inport)
    fprintf('  Outer Inport %d name: %s\n', i, get_param(phOuter.Inport(i), 'Name'));
end
for i = 1:numel(phOuter.Outport)
    fprintf('  Outer Outport %d name: %s\n', i, get_param(phOuter.Outport(i), 'Name'));
end
