modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

demux = [mdl '/Scope/Demux'];
fprintf('Outputs param: %s\n', get_param(demux, 'Outputs'));
ph = get_param(demux, 'PortHandles');
fprintf('numel(Outport) = %d\n', numel(ph.Outport));

lineH = get_param(ph.Inport(1), 'Line');
srcPortH = get_param(lineH, 'SrcPortHandle');
fprintf('Demux input source: %s\n', get_param(srcPortH, 'Parent'));

fprintf('\n=== which Demux outport feeds which To Workspace ===\n');
tws = {'To Workspace6','To Workspace7','To Workspace8'};
vars = {'des_x1','des_y1','des_z1'};
for i = 1:numel(tws)
    twBlk = [mdl '/Scope/' tws{i}];
    twPh = get_param(twBlk, 'PortHandles');
    lineH = get_param(twPh.Inport(1), 'Line');
    srcPortH = get_param(lineH, 'SrcPortHandle');
    portNum = get_param(srcPortH, 'PortNumber');
    fprintf('  %s (%s) <- Demux outport #%d\n', tws{i}, vars{i}, portNum);
end

fprintf('\n=== In Bus Element/1/2 Element names (act_x/y/z source) ===\n');
for i = 0:2
    name = 'In Bus Element';
    if i > 0; name = sprintf('In Bus Element%d', i); end
    blk = [mdl '/Scope/' name];
    fprintf('  %s Element=%s\n', name, get_param(blk, 'Element'));
end
