%% 피드포워드(둘째 경로) 해부 1단계 (시뮬 없음): 기준 궤적 소비처 전수 수색
%% 배경(§U/§V): C=0.01 강클램프에서 위치루프 최대 명령 ~0.5°인데 pitch 5.7~8.4° 실측
%%   -> 기준 궤적이 Position Control 밖에서 자세 명령으로 직행하는 배선 존재.
%% 방법: 모델 전 블록의 다이얼로그 파라미터에서 spline_data/timespot_spl/spline_yaw/
%%   waypoints 참조를 수색 -> 각 소비 블록의 출력이 어디로 흐르는지 1홉 추적.
%%   + YPR/자세 제어 서브시스템의 입력 포트 소스 목록 (2단계 태핑 지점 선정용).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

vars = {'spline_data', 'timespot_spl', 'spline_yaw', 'waypoints'};
blks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
fprintf('===== 1) 기준 궤적 변수 소비 블록 전수 (전체 %d블록 수색) =====\n', numel(blks));
nHit = 0;
for i = 1:numel(blks)
    blk = blks{i};
    try
        dp = get_param(blk, 'DialogParameters');
    catch
        continue;
    end
    if isempty(dp); continue; end
    fn = fieldnames(dp);
    for k = 1:numel(fn)
        try
            v = get_param(blk, fn{k});
        catch
            continue;
        end
        if ~ischar(v); continue; end
        for vi = 1:numel(vars)
            if contains(v, vars{vi})
                nHit = nHit + 1;
                nm = strtrim(regexprep(blk, '\s+', ' '));
                fprintf('\n[소비 %d] %s\n   파라미터 %s = %s\n', nHit, nm, fn{k}, strtrim(v));
                % 출력 1홉 추적
                try
                    ph = get_param(blk, 'PortHandles');
                    for oi = 1:numel(ph.Outport)
                        l = get_param(ph.Outport(oi), 'Line');
                        if l == -1; continue; end
                        dsts = get_param(l, 'DstBlockHandle');
                        for di = 1:numel(dsts)
                            if dsts(di) == -1; continue; end
                            fprintf('   출력%d -> %s\n', oi, strtrim(regexprep(getfullname(dsts(di)), '\s+', ' ')));
                        end
                    end
                catch
                end
                break;
            end
        end
    end
end
fprintf('\n(총 %d개 소비처. Position Control 밖 소비처가 옆문 용의자)\n', nHit);

fprintf('\n===== 2) 자세(YPR) 제어 서브시스템 입력 소스 (태핑 지점 후보) =====\n');
cand = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
for i = 1:numel(cand)
    nm1 = strtrim(regexprep(get_param(cand{i}, 'Name'), '\s+', ' '));
    if isempty(regexpi(nm1, 'ypr|attitude', 'once')); continue; end
    fprintf('\n[서브시스템] %s\n', strtrim(regexprep(cand{i}, '\s+', ' ')));
    try
        pc = get_param(cand{i}, 'PortConnectivity');
        for pi2 = 1:numel(pc)
            if isempty(pc(pi2).SrcBlock) || pc(pi2).SrcBlock == -1; continue; end
            fprintf('   입력포트 %s <- %s\n', pc(pi2).Type, ...
                strtrim(regexprep(getfullname(pc(pi2).SrcBlock), '\s+', ' ')));
        end
    catch e
        fprintf('   (조사 실패: %s)\n', e.message);
    end
end

fprintf('\n===== 3) Maneuver Controller 1층 구조 (참고) =====\n');
mc = [mdl '/Maneuver Controller'];
kids = find_system(mc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
for i = 1:numel(kids)
    if strcmp(kids{i}, mc); continue; end
    try; bt = get_param(kids{i}, 'BlockType'); catch; bt = '?'; end
    fprintf('   %-50s [%s]\n', strtrim(regexprep(get_param(kids{i},'Name'), '\s+', ' ')), bt);
end
