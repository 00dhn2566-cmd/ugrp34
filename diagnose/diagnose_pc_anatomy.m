%% Position Control 내부 해부 (시뮬 없음): 오차 클램프(PosErr Sat)를 우회하는 기준 경로 수색
%% 확정 사실(ff_trace): 기준의 동적 소비처는 Reference/Interpolate Spline Points 한 곳,
%%   YPR 입력은 전부 Position Control 경유 -> 옆문은 Position Control 내부에 있어야 함.
%% 방법: PC 1층 블록 인접 그래프 전부 출력 + Inport->Outport 경로 중 Sat 미경유 경로 탐지.

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
kids = find_system(pc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
kids = kids(~strcmp(kids, pc));
n = numel(kids);
fprintf('===== Position Control 1층 %d블록 인접 그래프 =====\n', n);

% 이름/타입 출력 + 핸들 매핑
h2i = containers.Map('KeyType','double','ValueType','double');
for i = 1:n
    h = get_param(kids{i}, 'Handle');
    h2i(h) = i;
end
adj = false(n);
for i = 1:n
    try; bt = get_param(kids{i}, 'BlockType'); catch; bt = '?'; end
    nm = strtrim(regexprep(get_param(kids{i}, 'Name'), '\s+', ' '));
    fprintf('\n[%2d] %-42s [%s]\n', i, nm, bt);
    try
        ph = get_param(kids{i}, 'PortHandles');
        for oi = 1:numel(ph.Outport)
            l = get_param(ph.Outport(oi), 'Line');
            if l == -1; continue; end
            dsts = get_param(l, 'DstBlockHandle');
            for di = 1:numel(dsts)
                if dsts(di) == -1; continue; end
                if isKey(h2i, dsts(di))
                    j = h2i(dsts(di));
                    adj(i, j) = true;
                    fprintf('     out%d -> [%2d] %s\n', oi, j, ...
                        strtrim(regexprep(get_param(dsts(di), 'Name'), '\s+', ' ')));
                end
            end
        end
    catch e
        fprintf('     (라인 조사 실패: %s)\n', e.message);
    end
end

% Inport -> Outport 경로 중 PosErr Sat 미경유 탐지
fprintf('\n===== Sat 우회 경로 탐지 =====\n');
isIn = false(1,n); isOut = false(1,n); isSat = false(1,n);
for i = 1:n
    bt = '';
    try; bt = get_param(kids{i}, 'BlockType'); catch; end
    nm = strtrim(regexprep(get_param(kids{i}, 'Name'), '\s+', ' '));
    isIn(i) = strcmp(bt, 'Inport');
    isOut(i) = strcmp(bt, 'Outport');
    isSat(i) = contains(nm, 'PosErr Sat');
end
% Sat 노드 제거한 그래프에서 도달성
adj2 = adj;
adj2(isSat, :) = false;
adj2(:, isSat) = false;
for s = find(isIn)
    reach = false(1,n); stack = s;
    while ~isempty(stack)
        c = stack(end); stack(end) = [];
        nxt = find(adj2(c,:) & ~reach);
        reach(nxt) = true;
        stack = [stack nxt]; %#ok<AGROW>
    end
    hitOut = find(reach & isOut);
    if ~isempty(hitOut)
        fprintf('  [우회!] Inport [%d] %s -> Sat 안 거치고 Outport 도달: ', s, ...
            strtrim(regexprep(get_param(kids{s},'Name'), '\s+', ' ')));
        for o = hitOut
            fprintf('[%d] %s  ', o, strtrim(regexprep(get_param(kids{o},'Name'), '\s+', ' ')));
        end
        fprintf('\n');
    else
        fprintf('  Inport [%d] %s -> Sat 우회 경로 없음\n', s, ...
            strtrim(regexprep(get_param(kids{s},'Name'), '\s+', ' ')));
    end
end
fprintf('(우회 경로에 낀 중간 블록이 옆문 후보. 서브시스템이면 다음 단계로 그 내부 해부)\n');
