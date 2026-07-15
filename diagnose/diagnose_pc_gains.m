%% Position Control 명령 사슬 게인/한계 덤프 (시뮬 없음): MM 출력 -> 자세명령[rad] 환산 확정
%% ff_tap 미스터리: PC 출력 전성분 소형인데 pitch 15° - Err2P 스케일 환산이 선결 과제.
modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

pc = [mdl '/Maneuver Controller/Position Control'];
gainBlks = {'Dir P', 'Dir R', 'Err2P', 'Err2R'};
for i = 1:numel(gainBlks)
    b = [pc '/' gainBlks{i}];
    v = get_param(b, 'Gain');
    fprintf('%-8s Gain = %s', gainBlks{i}, v);
    try
        fprintf('   (수치: %g)', evalin('base', v));
    catch
    end
    fprintf('\n');
end
satBlks = {'Pitch Limit', 'Roll Limit', 'PosErr Sat X', 'PosErr Sat Y'};
for i = 1:numel(satBlks)
    b = [pc '/' satBlks{i}];
    fprintf('%-13s Upper=%s Lower=%s\n', satBlks{i}, get_param(b,'UpperLimit'), get_param(b,'LowerLimit'));
end
% PID Controller 서브시스템 내부 게인도 (P/I/D가 무슨 변수인지)
pid = [pc '/PID Controller'];
inner = find_system(pid, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Gain');
fprintf('\nPID Controller 내부 Gain 블록:\n');
for i = 1:numel(inner)
    fprintf('  %-40s Gain = %s\n', strtrim(regexprep(inner{i}(numel(pid)+2:end), '\s+', ' ')), get_param(inner{i}, 'Gain'));
end
% Filter 서브시스템 정체
flt = [pc '/Filter'];
finner = find_system(flt, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
fprintf('\nFilter 내부 블록:\n');
for i = 1:numel(finner)
    if strcmp(finner{i}, flt); continue; end
    try; bt = get_param(finner{i}, 'BlockType'); catch; bt = '?'; end
    fprintf('  %-40s [%s]\n', strtrim(regexprep(finner{i}(numel(flt)+2:end), '\s+', ' ')), bt);
end
