%% roll이 pitch와 달리 -90도 근처까지 크게 떨어지는 이유를 찾기 위해,
%% Control Pitch vs Control Roll, Filter Pitch vs Filter Roll의 파라미터를
%% 직접 나란히 비교. 로직/게인이 완전히 대칭이면 physical(CAD 무게중심/관성)
%% 쪽 문제로 좁혀지고, 비대칭이면 그 자체가 원인 후보.

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

function dumpMask(blk, label)
    fprintf('  --- %s (%s) ---\n', label, blk);
    try
        mn = get_param(blk, 'MaskNames');
        mv = get_param(blk, 'MaskValues');
        for i = 1:numel(mn)
            fprintf('    %s = %s\n', mn{i}, mv{i});
        end
    catch e
        fprintf('    (마스크 없음/조회 실패: %s)\n', e.message);
    end
end

fprintf('=== Control Pitch vs Control Roll ===\n');
dumpMask([ypr '/Control Pitch'], 'Control Pitch');
dumpMask([ypr '/Control Roll'], 'Control Roll');

fprintf('\n=== Filter Pitch vs Filter Roll ===\n');
dumpMask([ypr '/Filter Pitch'], 'Filter Pitch');
dumpMask([ypr '/Filter Roll'], 'Filter Roll');

fprintf('\n=== YPR Control 자체의 마스크 파라미터(부모, attitude_kp 등 실제 활성값) ===\n');
dumpMask(ypr, 'Altitude and YPR Control');

% Sum 블록 부호 비교 (Control Pitch/Roll 앞단 에러 계산)
fprintf('\n=== 에러 계산 Sum 블록 (Pitch Cmd/Roll Cmd 쪽) 부호 ===\n');
for name = {'Add', 'Add2', 'Add3', 'Add7'}
    blk = [ypr '/' name{1}];
    try
        fprintf('  %s: Inputs=%s\n', name{1}, get_param(blk, 'Inputs'));
    catch
    end
end

% CAD 물리 파라미터: 관성모멘트, 암 위치 등 roll/pitch축 비교
fprintf('\n=== 관성모멘트/질량 관련 변수 (base workspace, roll=X축 vs pitch=Y축 가정) ===\n');
varsToCheck = {'Ixx','Iyy','Izz','inertia','drone_mass','arm_length','arm_radius'};
for i = 1:numel(varsToCheck)
    vn = varsToCheck{i};
    if evalin('base', sprintf('exist(''%s'',''var'')', vn))
        v = evalin('base', vn);
        fprintf('  %s = %s\n', vn, mat2str(v));
    end
end

% CAD Rigid Transform에서 프로펠러 위치 좌표 확인 (roll/pitch 축 대칭성)
fprintf('\n=== 프로펠러 CAD 장착 위치 (Rigid Transform Offset, roll=Y거리 vs pitch=X거리 확인) ===\n');
for p = 1:4
    sub = sprintf('%s/Quadcopter/Propeller %d', mdl, p);
    rt = find_system(sub, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', 'Rigid Transform');
    for i = 1:numel(rt)
        try
            off = get_param(rt{i}, 'Offset');
            fprintf('  Propeller %d - %s: Offset=%s\n', p, strrep(rt{i}, [sub '/'], ''), off);
        catch
        end
    end
end
