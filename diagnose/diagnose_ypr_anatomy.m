%% Altitude and YPR Control + Motor Mixer 1층 해부 (시뮬 없음): 태핑 지점 선정용
%% ff_tap 미스터리: PC 명령 0.6°인데 실제 pitch 15~29° 유지 - 자세 루프가 지는 이유
%% (모터 포화? 다른 기준?)를 밝히기 위한 내부 구조 파악.
modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 블록 이름 개행 함정 대응: Maneuver Controller 자식들을 정규화 이름으로 매칭
mc = [mdl '/Maneuver Controller'];
mcKids = find_system(mc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
resolve = @(want) mcKids{cellfun(@(b) strcmp(strtrim(regexprep(get_param(b,'Name'),'\s+',' ')), want), mcKids)};
for tgtC = {'Altitude and YPR Control', 'Motor Mixer'}
    tgt = resolve(tgtC{1});
    if isempty(tgt); error('%s 미발견 - 이름 목록 확인 필요', tgtC{1}); end
    kids = find_system(tgt, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
    kids = kids(~strcmp(kids, tgt));
    n = numel(kids);
    fprintf('\n===== %s 1층 %d블록 =====\n', tgtC{1}, n);
    h2i = containers.Map('KeyType','double','ValueType','double');
    for i = 1:n
        h2i(get_param(kids{i}, 'Handle')) = i;
    end
    for i = 1:n
        try; bt = get_param(kids{i}, 'BlockType'); catch; bt = '?'; end
        nm = strtrim(regexprep(get_param(kids{i}, 'Name'), '\s+', ' '));
        extra = '';
        if strcmp(bt, 'Gain')
            extra = [' Gain=' get_param(kids{i}, 'Gain')];
        elseif strcmp(bt, 'Saturate')
            extra = [' [' get_param(kids{i}, 'LowerLimit') ',' get_param(kids{i}, 'UpperLimit') ']'];
        end
        fprintf('[%2d] %-40s [%s]%s\n', i, nm, bt, extra);
        try
            ph = get_param(kids{i}, 'PortHandles');
            for oi = 1:numel(ph.Outport)
                l = get_param(ph.Outport(oi), 'Line');
                if l == -1; continue; end
                dsts = get_param(l, 'DstBlockHandle');
                for di = 1:numel(dsts)
                    if dsts(di) == -1 || ~isKey(h2i, dsts(di)); continue; end
                    fprintf('     out%d -> [%2d] %s\n', oi, h2i(dsts(di)), ...
                        strtrim(regexprep(get_param(dsts(di), 'Name'), '\s+', ' ')));
                end
            end
        catch
        end
    end
end
