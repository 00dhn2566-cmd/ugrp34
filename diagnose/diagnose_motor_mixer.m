modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

describe_ports('quadcopter_package_delivery/Maneuver Controller/Motor Mixer');
fprintf('\n');
describe_ports('quadcopter_package_delivery/Maneuver Controller');
fprintf('\n');
describe_ports('quadcopter_package_delivery/Quadcopter/Electrical/Motor 1');
fprintf('\n');
describe_ports('quadcopter_package_delivery/Quadcopter/Electrical');
fprintf('\n');
describe_ports('quadcopter_package_delivery/Quadcopter/Propeller 1');

fprintf('\n=== Top-level lines from Maneuver Controller outports ===\n');
blk = 'quadcopter_package_delivery/Maneuver Controller';
ph = get_param(blk, 'PortHandles');
for i = 1:numel(ph.Outport)
    lineH = get_param(ph.Outport(i), 'Line');
    if lineH ~= -1
        dstPorts = get_param(lineH, 'DstPortHandle');
        for j = 1:numel(dstPorts)
            dstBlk = get_param(dstPorts(j), 'Parent');
            fprintf('  Out%d (%s) -> %s\n', i, get_param(ph.Outport(i),'Name'), dstBlk);
        end
    end
end

fprintf('\n=== Electrical subsystem contents ===\n');
b = find_system('quadcopter_package_delivery/Quadcopter/Electrical', 'SearchDepth', 1);
for i = 1:numel(b); fprintf('  %s\n', b{i}); end

function describe_ports(blk)
    fprintf('--- %s (%s) ---\n', blk, get_param(blk, 'BlockType'));
    ph = get_param(blk, 'PortHandles');
    for i = 1:numel(ph.Inport)
        fprintf('  In%d name=%s\n', i, get_param(ph.Inport(i), 'Name'));
    end
    for i = 1:numel(ph.Outport)
        fprintf('  Out%d name=%s\n', i, get_param(ph.Outport(i), 'Name'));
    end
end
