modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

cbFields = {'PreLoadFcn','PostLoadFcn','InitFcn','StartFcn','PauseFcn', ...
            'ContinueFcn','StopFcn','CloseFcn'};
for i = 1:numel(cbFields)
    v = get_param(mdl, cbFields{i});
    fprintf('--- %s ---\n%s\n\n', cbFields{i}, v);
end

fprintf('=== grep attitude_k in any .m file under Scripts_Data/Models ===\n');
d1 = dir(fullfile(modelDir, 'Scripts_Data', '*.m'));
d2 = dir(fullfile(modelDir, 'Models', '*.m'));
allFiles = [d1; d2];
for i = 1:numel(allFiles)
    fp = fullfile(allFiles(i).folder, allFiles(i).name);
    txt = fileread(fp);
    if contains(txt, 'attitude_k')
        fprintf('  FOUND in %s\n', fp);
    end
end
