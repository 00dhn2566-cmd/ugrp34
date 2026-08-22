%% 궤적 참조 시계 정찰 — 가상 시계 조속기(SPEED_GOVERNOR.md §1) 배선점 확정용
%% 찾을 것:
%%   Q1. timespot_spl / spline_data / spline_yaw 를 다이얼로그에 쓰는 블록 전부
%%   Q2. 그 블록들의 '시간' 입력을 무엇이 만드는가 (Clock? Ramp? 공유되나?)
%%   Q3. 시계 소스가 하나로 모이는지 (하나면 수술 1점, 갈라지면 여러 점)
%% 규칙: 읽기만. save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

vars = {'timespot_spl', 'spline_data', 'spline_yaw'};
all = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
fprintf('=== Q1: 궤적 변수를 참조하는 블록 (전체 %d개 중) ===\n', numel(all));

hits = {};
for i = 1:numel(all)
    b = all{i};
    try
        dp = get_param(b, 'DialogParameters');
    catch
        continue;
    end
    if isempty(dp); continue; end
    names = fieldnames(dp);
    used = {};
    for k = 1:numel(names)
        try
            v = get_param(b, names{k});
        catch
            continue;
        end
        if ~ischar(v) && ~isstring(v); continue; end
        v = char(v);
        for q = 1:numel(vars)
            if contains(v, vars{q})
                used{end+1} = sprintf('%s=%s', names{k}, v); %#ok<SAGROW>
            end
        end
    end
    if ~isempty(used)
        hits{end+1} = b; %#ok<SAGROW>
        fprintf('\n  [%d] %s\n      BlockType=%s\n', numel(hits), ...
                strtrim(regexprep(b, '\s+', ' ')), get_param(b,'BlockType'));
        for k = 1:numel(used)
            fprintf('      %s\n', used{k});
        end
    end
end

fprintf('\n=== Q2: 각 블록의 입력 소스 추적 ===\n');
srcSet = {};
for i = 1:numel(hits)
    b = hits{i};
    ph = get_param(b, 'PortHandles');
    fprintf('\n  [%d] %s  (Inport %d개)\n', i, get_param(b,'Name'), numel(ph.Inport));
    for k = 1:numel(ph.Inport)
        l = get_param(ph.Inport(k), 'Line');
        if l == -1
            fprintf('      In%d : (미연결)\n', k);
            continue;
        end
        sp = get_param(l, 'SrcPortHandle');
        sb = get_param(sp, 'Parent');
        bt = get_param(sb, 'BlockType');
        tag = '';
        if strcmp(bt, 'From')
            tag = sprintf(' tag=%s', get_param(sb, 'GotoTag'));
        end
        fprintf('      In%d <- %s (%s)%s\n', k, strtrim(regexprep(sb,'\s+',' ')), bt, tag);
        srcSet{end+1} = sb; %#ok<SAGROW>
    end
end

fprintf('\n=== Q3: 모델 안의 Clock 계열 블록 ===\n');
for bt = {'Clock', 'DigitalClock'}
    cs = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'BlockType', bt{1});
    for i = 1:numel(cs)
        ph = get_param(cs{i}, 'PortHandles');
        fprintf('  %s (%s)\n', strtrim(regexprep(cs{i},'\s+',' ')), bt{1});
        l = get_param(ph.Outport(1), 'Line');
        if l ~= -1
            d = get_param(l, 'DstPortHandle');
            for k = 1:numel(d)
                fprintf('      -> %s (%s)\n', strtrim(regexprep(get_param(d(k),'Parent'),'\s+',' ')), ...
                        get_param(get_param(d(k),'Parent'),'BlockType'));
            end
        end
    end
end

fprintf('\n=== 결론 ===\n');
u = unique(srcSet);
fprintf('  궤적 블록의 입력 소스 종류 %d개:\n', numel(u));
for i = 1:numel(u)
    fprintf('    %s (%s)\n', strtrim(regexprep(u{i},'\s+',' ')), get_param(u{i},'BlockType'));
end
fprintf('  -> 소스가 1개면 그 선 하나만 Integrator(∫s dt)로 갈아끼우면 된다.\n');
fprintf('  ※ save_system 하지 않음.\n');
