%% File Solid의 CAD 파일 참조 방식 전수 확인 (읽기 전용, 시뮬 없음)
%% - Arm1 / plate_top / plate_bottom 블록의 전체 DialogParameters 덤프
%% - which()로 실제 어느 파일이 해석되는지 확인

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

targets = { ...
    [mdl '/Quadcopter/Body/Arm1'], ...
    [mdl '/Quadcopter/Body/Body/plate_bottom'] ...
};
% plate_top은 이름에 개행이 있어서 검색으로 찾는다
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
for i = 1:numel(allBlk)
    try
        nm1 = regexprep(get_param(allBlk{i}, 'Name'), '\s+', ' ');
    catch
        continue;
    end
    if strcmp(strtrim(nm1), 'plate_top')
        targets{end+1} = allBlk{i}; %#ok<AGROW>
    end
end

for t = 1:numel(targets)
    b = targets{t};
    fprintf('\n=============== %s ===============\n', strrep(b, newline, '|'));
    try
        dp = get_param(b, 'DialogParameters');
    catch e
        fprintf('  블록 접근 실패: %s\n', e.message(1:min(120,end)));
        continue;
    end
    fn = fieldnames(dp);
    for k = 1:numel(fn)
        try
            v = get_param(b, fn{k});
            if ischar(v) && ~isempty(v)
                fprintf('  %s = %s\n', fn{k}, v);
            end
        catch
        end
    end
end

fprintf('\n=============== which()로 실제 해석 경로 ===============\n');
for f = {'quadcopter_drone_arm.stp','quadcopter_drone_plate_top.stp','quadcopter_drone_plate_bottom.stp'}
    w = which(f{1});
    if isempty(w); w = '(경로에서 못 찾음)'; end
    fprintf('  %s -> %s\n', f{1}, w);
end
