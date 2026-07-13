%% Body 전체 감사 (시뮬 없음, 파라미터 읽기 전용 - 빠름)
%% 목적: FX450 파일 교체(커밋 156cd3b: arm, plate_top, plate_bottom)로 인한
%% 부품별 [파일 프레임 x 관성 설정 x 장착 Transform] 정합 여부를 한 번에 나열.
%% - 모든 Solid: InertiaType, (Custom이면) Mass/CoM/MoI/PoI, (FromGeometry면) Density, 형상 파일
%% - 모든 Rigid Transform: 활성 RotationMethod에 해당하는 값 + Translation
%% - 패키지/조인트 체인: Weld/Revolute/Spherical 조인트와 패키지 관련 블록

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
fprintf('전체 블록 수: %d\n', numel(allBlk));

solids = {}; transforms = {}; joints = {};
for i = 1:numel(allBlk)
    try
        dp = get_param(allBlk{i}, 'DialogParameters');
    catch
        continue;
    end
    if ~isstruct(dp); continue; end
    fn = fieldnames(dp);
    if any(strcmp(fn, 'InertiaType')) || any(strcmp(fn, 'ExtGeomFileName'))
        solids{end+1} = allBlk{i}; %#ok<AGROW>
    elseif any(strcmp(fn, 'RotationMethod')) && any(strcmp(fn, 'TranslationMethod'))
        transforms{end+1} = allBlk{i}; %#ok<AGROW>
    elseif any(strcmp(fn, 'PositionTargetValue')) || ~isempty(regexp(get_param(allBlk{i},'Name'), 'Joint', 'once'))
        joints{end+1} = allBlk{i}; %#ok<AGROW>
    end
end
fprintf('솔리드 후보 %d개 / Rigid Transform %d개 / 조인트 후보 %d개\n\n', numel(solids), numel(transforms), numel(joints));

gp = @(b,p) get_param(b,p);

fprintf('=============== [1] 솔리드 전수 ===============\n');
for i = 1:numel(solids)
    b = solids{i};
    fprintf('--- %s\n', strrep(b, newline, '|'));
    try fprintf('    InertiaType = %s\n', gp(b,'InertiaType')); catch; end
    try
        f = gp(b,'ExtGeomFileName');
        if ~isempty(f); fprintf('    형상 파일 = %s\n', f); end
    catch
    end
    try fprintf('    Mass = %s\n', gp(b,'Mass')); catch; end
    try fprintf('    CoM = %s\n', gp(b,'CenterOfMass')); catch; end
    try fprintf('    MoI = %s\n', gp(b,'MomentsOfInertia')); catch; end
    try fprintf('    PoI = %s\n', gp(b,'ProductsOfInertia')); catch; end
    try fprintf('    Density = %s\n', gp(b,'Density')); catch; end
    try fprintf('    BasedOn = %s\n', gp(b,'InertiaBasedOn')); catch; end
end

fprintf('\n=============== [2] Rigid Transform 전수 (활성값만) ===============\n');
for i = 1:numel(transforms)
    b = transforms{i};
    try
        rm = gp(b,'RotationMethod');
    catch
        continue;
    end
    tm = '';
    try tm = gp(b,'TranslationMethod'); catch; end
    rot = '';
    try
        switch rm
            case 'None'
                rot = '(회전 없음)';
            case 'StandardAxis'
                rot = sprintf('축 %s, 각 %s %s', gp(b,'RotationStandardAxis'), gp(b,'RotationAngle'), gp(b,'RotationAngleUnits'));
            case 'RotationSequence'
                rot = sprintf('%s %s %s (%s)', gp(b,'RotationSequence'), gp(b,'RotationSequenceAngles'), gp(b,'RotationSequenceAnglesUnits'), gp(b,'RotationSequenceAxes'));
            case 'RotationMatrix'
                rot = sprintf('행렬 %s', gp(b,'RotationMatrix'));
            case 'ArbitraryAxis'
                rot = sprintf('임의축 %s, 각 %s %s', gp(b,'RotationArbitraryAxis'), gp(b,'RotationAngle'), gp(b,'RotationAngleUnits'));
            otherwise
                rot = rm;
        end
    catch
        rot = [rm ' (값 읽기 실패)'];
    end
    tr = '';
    try
        switch tm
            case 'None'
                tr = '(이동 없음)';
            case 'Cartesian'
                tr = sprintf('%s %s', gp(b,'TranslationCartesianOffset'), gp(b,'TranslationCartesianOffsetUnits'));
            case 'StandardAxis'
                tr = sprintf('축 %s, %s %s', gp(b,'TranslationStandardAxis'), gp(b,'TranslationStandardOffset'), gp(b,'TranslationStandardOffsetUnits'));
            otherwise
                tr = tm;
        end
    catch
        tr = [tm ' (값 읽기 실패)'];
    end
    fprintf('%-75s | 회전: %-40s | 이동: %s\n', strrep(b, newline, '|'), rot, tr);
end

fprintf('\n=============== [3] 조인트/패키지 체인 ===============\n');
for i = 1:numel(joints)
    fprintf('  조인트: %s\n', strrep(joints{i}, newline, '|'));
end
pkg = {};
for i = 1:numel(allBlk)
    try
        nm1 = get_param(allBlk{i}, 'Name');
    catch
        continue;
    end
    if ~isempty(regexpi(nm1, '(package|payload|basket|battery)', 'once'))
        pkg{end+1} = allBlk{i}; %#ok<AGROW>
    end
end
fprintf('패키지 관련 블록 %d개:\n', numel(pkg));
for i = 1:numel(pkg)
    b = pkg{i};
    fprintf('  %s', strrep(b, newline, '|'));
    try fprintf('  [BlockType=%s]', get_param(b,'BlockType')); catch; end
    fprintf('\n');
end
fprintf('\n감사 완료 (시뮬 없음)\n');
