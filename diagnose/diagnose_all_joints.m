%% 모델 전체 조인트/힘 요소 전수 조사 (시뮬 없음): 짐-기체 결합의 실제 위치와 자유도
%% Load 하위엔 Weld + 지면접촉뿐 -> 분리가능 결합(투하)은 상위 계층에 있어야 함.
%% 목표: 진자 피벗(회전 자유도 + 앵커 위치)을 찾아 L=8.1cm 예측과 대조.
modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

blks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
fprintf('===== 모델 전체 %d블록 중 조인트/힘/스프링 요소 =====\n', numel(blks));
for i = 1:numel(blks)
    nm = strtrim(regexprep(blks{i}, '\s+', ' '));
    rb = ''; mt = '';
    try; rb = get_param(blks{i}, 'ReferenceBlock'); catch; end
    try; mt = get_param(blks{i}, 'MaskType'); catch; end
    rb1 = strtrim(regexprep(rb, '\s+', ' '));
    key = lower([rb1 ' ' mt]);
    isJoint = contains(key, 'joint');
    isForce = contains(key, 'force') || contains(key, 'spring') || contains(key, 'bushing');
    if ~(isJoint || isForce); continue; end
    fprintf('[%s]\n   ref: %s\n', nm, rb1);
    % 조인트 스프링/댐퍼/타입 파라미터 덤프
    try
        dp = get_param(blks{i}, 'DialogParameters');
        fn = fieldnames(dp);
        for k = 1:numel(fn)
            if contains(lower(fn{k}), 'stiffness') || contains(lower(fn{k}), 'damping') || ...
               contains(lower(fn{k}), 'dampercoefficient') || contains(lower(fn{k}), 'springstiffness')
                try
                    v = get_param(blks{i}, fn{k});
                    if ischar(v) && ~strcmp(v, '0')
                        fprintf('   %s = %s\n', fn{k}, v);
                    end
                catch
                end
            end
        end
    catch
    end
end
fprintf('\n===== Disengage Logic 신호 소비처 (홀딩/분리 메커니즘) =====\n');
dis = [mdl '/Quadcopter/Load/Disengage Logic'];
try
    ph = get_param(dis, 'PortHandles');
    for oi = 1:numel(ph.Outport)
        l = get_param(ph.Outport(oi), 'Line');
        if l == -1; continue; end
        dsts = get_param(l, 'DstBlockHandle');
        for di = 1:numel(dsts)
            fprintf('  출력%d -> %s\n', oi, strtrim(regexprep(getfullname(dsts(di)), '\s+', ' ')));
        end
    end
catch e
    fprintf('  (조사 실패: %s)\n', e.message);
end
