modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

names = {'filtM_position','kp_position','ki_position','kd_position','filtD_position','pos2attitude'};
for i = 1:numel(names)
    if exist(names{i}, 'var')
        v = eval(names{i});
        fprintf('%s = %s  (class=%s, size=%s)\n', names{i}, mat2str(v), class(v), mat2str(size(v)));
    else
        fprintf('%s: NOT DEFINED\n', names{i});
    end
end

mdl = 'quadcopter_package_delivery';
load_system(mdl);
pidBlk = [mdl '/Maneuver Controller/Position Control/PID Controller'];
ph = get_param(pidBlk, 'PortHandles');
fprintf('\nPID Controller ports: In=%d Out=%d\n', numel(ph.Inport), numel(ph.Outport));
fprintf('CompiledPortDimensions (need update diagram):\n');
try
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('  In: %s\n', mat2str(get_param(ph.Inport(1),'CompiledPortDimensions')));
    fprintf('  Out: %s\n', mat2str(get_param(ph.Outport(1),'CompiledPortDimensions')));
catch e
    fprintf('  error: %s\n', e.message);
end
