%% 추력 동작점(호버 피드포워드)이 모델 어디에 박혀 있는지 추적.
%% 물리 계산: drone_mass=1.2726kg -> 호버 총추력 12.48N -> 프로펠러당 3.12N
%% -> Kthrust=0.1072, D=0.254 기준 호버 모터속도 약 475 rad/s.
%% 그런데 원래 게인에서 모터 명령이 10,000+ rad/s였음 -> 추력 경로 어딘가에
%% 옛 기체 기준의 큰 상수/스케일이 있을 것. Motor Mixer와 그 주변을 전부 덤프.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

mixer = [mdl '/Maneuver Controller/Motor Mixer'];

fprintf('=== Motor Mixer 내부 전체 블록 (타입 + 주요 파라미터) ===\n');
blks = find_system(mixer, 'LookUnderMasks', 'all');
for i = 1:numel(blks)
    if strcmp(blks{i}, mixer); continue; end
    bt = get_param(blks{i}, 'BlockType');
    extra = '';
    switch bt
        case 'Constant'
            extra = sprintf(' Value=%s', get_param(blks{i}, 'Value'));
        case 'Gain'
            extra = sprintf(' Gain=%s', get_param(blks{i}, 'Gain'));
        case 'Sum'
            extra = sprintf(' Inputs=%s', get_param(blks{i}, 'Inputs'));
        case 'Math'
            extra = sprintf(' Operator=%s', get_param(blks{i}, 'Operator'));
        case 'Sqrt'
            extra = ' (sqrt)';
        case 'Product'
            extra = sprintf(' Inputs=%s', get_param(blks{i}, 'Inputs'));
        case 'Goto'
            extra = sprintf(' Tag=%s', get_param(blks{i}, 'GotoTag'));
        case 'From'
            extra = sprintf(' Tag=%s', get_param(blks{i}, 'GotoTag'));
        case 'Saturate'
            extra = sprintf(' Upper=%s Lower=%s', get_param(blks{i}, 'UpperLimit'), get_param(blks{i}, 'LowerLimit'));
    end
    fprintf('  %s (%s)%s\n', strrep(blks{i}, [mixer '/'], ''), bt, extra);
end

% Thrust Goto의 소스 추적: Control Thrust 출력 -> ? -> Goto Thrust
fprintf('\n=== "Thrust" Goto 태그의 소스 체인 ===\n');
gotos = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'Goto', 'GotoTag', 'Thrust');
for g = 1:numel(gotos)
    fprintf('  Goto: %s\n', gotos{g});
    blk = gotos{g};
    for step = 1:12
        ph = get_param(blk, 'PortHandles');
        if isempty(ph.Inport); break; end
        lineH = get_param(ph.Inport(1), 'Line');
        if lineH == -1; break; end
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH == -1; break; end
        srcBlk = get_param(srcPortH, 'Parent');
        bt = get_param(srcBlk, 'BlockType');
        extra = '';
        switch bt
            case 'Constant'; extra = sprintf(' Value=%s', get_param(srcBlk, 'Value'));
            case 'Gain';     extra = sprintf(' Gain=%s', get_param(srcBlk, 'Gain'));
            case 'Sum';      extra = sprintf(' Inputs=%s', get_param(srcBlk, 'Inputs'));
            case 'Saturate'; extra = sprintf(' Upper=%s Lower=%s', get_param(srcBlk, 'UpperLimit'), get_param(srcBlk, 'LowerLimit'));
        end
        fprintf('    <- %s (%s)%s\n', srcBlk, bt, extra);
        if strcmp(bt, 'SubSystem') || strcmp(bt, 'Inport'); break; end
        blk = srcBlk;
    end
end

% Altitude and YPR Control 안의 Thrust 관련 블록들도 확인 (Control Thrust 주변)
fprintf('\n=== Altitude and YPR Control 내 Sum/Constant/Gain 블록 ===\n');
ypr = [mdl '/Maneuver Controller/Altitude and  YPR Control'];
for bt = {'Constant', 'Gain', 'Sum', 'Bias'}
    bb = find_system(ypr, 'LookUnderMasks', 'all', 'BlockType', bt{1});
    for i = 1:numel(bb)
        extra = '';
        switch bt{1}
            case 'Constant'; extra = get_param(bb{i}, 'Value');
            case 'Gain';     extra = get_param(bb{i}, 'Gain');
            case 'Sum';      extra = get_param(bb{i}, 'Inputs');
            case 'Bias';     extra = get_param(bb{i}, 'Bias');
        end
        fprintf('  %s (%s) %s\n', strrep(bb{i}, [ypr '/'], ''), bt{1}, extra);
    end
end

% Maneuver Controller 레벨의 Constant/Gain도 확인
fprintf('\n=== Maneuver Controller 직속 Constant/Gain/Sum ===\n');
mc = [mdl '/Maneuver Controller'];
for bt = {'Constant', 'Gain', 'Sum'}
    bb = find_system(mc, 'SearchDepth', 1, 'BlockType', bt{1});
    for i = 1:numel(bb)
        extra = '';
        switch bt{1}
            case 'Constant'; extra = get_param(bb{i}, 'Value');
            case 'Gain';     extra = get_param(bb{i}, 'Gain');
            case 'Sum';      extra = get_param(bb{i}, 'Inputs');
        end
        fprintf('  %s (%s) %s\n', strrep(bb{i}, [mc '/'], ''), bt{1}, extra);
    end
end
