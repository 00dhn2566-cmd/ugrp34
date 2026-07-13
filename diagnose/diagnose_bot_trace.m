%% 하판 Bot/Top 프레임 + 패키지 Weld Joint 연결 추적 (읽기 전용, 시뮬 없음)
%% 목적: CoM y=-14.3mm의 캐리어 확정 - 패키지가 하판의 어느 프레임에 붙는지,
%% Load 서브시스템 안에 뭐가 있는지 전부 나열.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% 추적 대상 블록 수집 (이름에 개행 있을 수 있어 검색으로)
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
targets = {};
labels = {};
for i = 1:numel(allBlk)
    try
        nm1 = strtrim(regexprep(get_param(allBlk{i}, 'Name'), '\s+', ' '));
    catch
        continue;
    end
    if any(strcmp(nm1, {'plate_bottom','plate_top','Weld Joint'})) || ...
       (strcmp(nm1,'Package') && ~isempty(regexp(allBlk{i}, 'Package/Package', 'once')))
        targets{end+1} = allBlk{i}; %#ok<AGROW>
        labels{end+1} = nm1; %#ok<AGROW>
    end
end
fprintf('추적 대상 %d개\n', numel(targets));

for t = 1:numel(targets)
    b = targets{t};
    fprintf('\n=============== [%s] %s ===============\n', labels{t}, strrep(b, newline, '|'));
    try
        ph = get_param(b, 'PortHandles');
    catch
        fprintf('  포트 접근 실패\n');
        continue;
    end
    conn = [ph.LConn ph.RConn];
    fprintf('  물리포트 %d개 (LConn %d / RConn %d)\n', numel(conn), numel(ph.LConn), numel(ph.RConn));
    for ci = 1:numel(conn)
        cp = conn(ci);
        l = get_param(cp, 'Line');
        if l == -1
            fprintf('  포트%d: 연결 없음\n', ci);
            continue;
        end
        % 라인 + 분기 라인의 모든 포트 수집
        lns = l;
        try
            ch = get_param(l, 'LineChildren');
            if ~isempty(ch); lns = [lns; ch(:)]; end
        catch
        end
        prts = [];
        for li = 1:numel(lns)
            try
                sp = get_param(lns(li), 'SrcPortHandle');
                dpp = get_param(lns(li), 'DstPortHandle');
                prts = [prts sp dpp]; %#ok<AGROW>
            catch
            end
        end
        prts = unique(prts(prts > 0 & prts ~= cp));
        if isempty(prts)
            fprintf('  포트%d: 상대 포트 식별 불가\n', ci);
            continue;
        end
        for pi = 1:numel(prts)
            try
                ob = get_param(prts(pi), 'Parent');
                fprintf('  포트%d <-> %s\n', ci, strrep(ob, newline, '|'));
            catch
            end
        end
    end
end

% Load 서브시스템 전체 블록 나열
fprintf('\n=============== Load 서브시스템 내용물 ===============\n');
loadBlk = find_system([mdl '/Quadcopter/Load'], 'LookUnderMasks','all', 'FollowLinks','on');
for i = 1:numel(loadBlk)
    b = loadBlk{i};
    bt = '';
    try bt = get_param(b, 'BlockType'); catch; end
    fprintf('  %s  [%s]\n', strrep(b, newline, '|'), bt);
end

% Weld Joint 파라미터 + Package 솔리드 지오메트리 파라미터
fprintf('\n=============== 파라미터 상세 ===============\n');
for t = 1:numel(targets)
    if ~any(strcmp(labels{t}, {'Weld Joint','Package'})); continue; end
    b = targets{t};
    fprintf('--- %s\n', strrep(b, newline, '|'));
    try
        dp = get_param(b, 'DialogParameters');
        fn = fieldnames(dp);
        for k = 1:numel(fn)
            try
                v = get_param(b, fn{k});
                if ischar(v) && ~isempty(v) && isempty(regexp(fn{k}, 'Graphic|_conf', 'once'))
                    fprintf('    %s = %s\n', fn{k}, v);
                end
            catch
            end
        end
    catch
    end
end
fprintf('\n추적 완료\n');
