%% 호버 피드포워드("Bias Chassis"=700, "Bias Load"=260*pkgMass)가 들어있는
%% Altitude and YPR Control/Subsystem 내부 배선을 전부 덤프해서,
%% 700이 어떤 단위(추력? 모터속도?)이고 신호가 어떤 순서로 흐르는지 파악.
%% (sqrt 앞이면 추력 단위 -> 새 무게 비례로 수정, 뒤면 속도 단위 -> sqrt(무게비)로 수정)

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
sub = [ypr '/Subsystem'];

fprintf('=== Subsystem 내부 전체 블록 ===\n');
blks = find_system(sub, 'LookUnderMasks', 'all');
for i = 1:numel(blks)
    if strcmp(blks{i}, sub); continue; end
    bt = get_param(blks{i}, 'BlockType');
    extra = '';
    try
        switch bt
            case 'Constant'; extra = sprintf(' Value=%s', get_param(blks{i}, 'Value'));
            case 'Gain';     extra = sprintf(' Gain=%s', get_param(blks{i}, 'Gain'));
            case 'Sum';      extra = sprintf(' Inputs=%s', get_param(blks{i}, 'Inputs'));
            case 'Bias';     extra = sprintf(' Bias=%s', get_param(blks{i}, 'Bias'));
            case 'Math';     extra = sprintf(' Operator=%s', get_param(blks{i}, 'Operator'));
            case 'Sqrt';     extra = sprintf(' Operator=%s', get_param(blks{i}, 'Operator'));
            case 'Product';  extra = sprintf(' Inputs=%s', get_param(blks{i}, 'Inputs'));
            case 'Saturate'; extra = sprintf(' Upper=%s Lower=%s', get_param(blks{i}, 'UpperLimit'), get_param(blks{i}, 'LowerLimit'));
            case 'Lookup_n-D'; extra = sprintf(' Table=%s', get_param(blks{i}, 'Table'));
        end
    catch
    end
    fprintf('  %s (%s)%s\n', strrep(blks{i}, [sub '/'], ''), bt, extra);
end

fprintf('\n=== Subsystem 내부 배선 (블록 간 연결) ===\n');
lines = find_system(sub, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for i = 1:numel(lines)
    srcPortH = get_param(lines(i), 'SrcPortHandle');
    dstPorts = get_param(lines(i), 'DstPortHandle');
    if srcPortH == -1; continue; end
    srcBlk = strrep(get_param(srcPortH, 'Parent'), [sub '/'], '');
    for j = 1:numel(dstPorts)
        if dstPorts(j) == -1; continue; end
        dstBlk = strrep(get_param(dstPorts(j), 'Parent'), [sub '/'], '');
        fprintf('  %s -> %s\n', srcBlk, dstBlk);
    end
end

% Subsystem의 입출력이 바깥(YPR Control)에서 어디에 연결되는지
fprintf('\n=== Subsystem 입출력의 외부 연결 ===\n');
ph = get_param(sub, 'PortHandles');
for i = 1:numel(ph.Inport)
    lineH = get_param(ph.Inport(i), 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        if srcPortH ~= -1
            fprintf('  In%d <- %s\n', i, strrep(get_param(srcPortH, 'Parent'), [ypr '/'], ''));
        end
    end
end
for i = 1:numel(ph.Outport)
    lineH = get_param(ph.Outport(i), 'Line');
    if lineH ~= -1
        dstPorts = get_param(lineH, 'DstPortHandle');
        for j = 1:numel(dstPorts)
            if dstPorts(j) ~= -1
                fprintf('  Out%d -> %s\n', i, strrep(get_param(dstPorts(j), 'Parent'), [ypr '/'], ''));
            end
        end
    end
end

% "Thrust to Torque" 게인과 Control Thrust 주변 흐름도 확인
fprintf('\n=== Control Thrust 출력이 어디로 가는지 ===\n');
ct = [ypr '/Control Thrust'];
ph = get_param(ct, 'PortHandles');
for i = 1:numel(ph.Outport)
    lineH = get_param(ph.Outport(i), 'Line');
    if lineH ~= -1
        dstPorts = get_param(lineH, 'DstPortHandle');
        for j = 1:numel(dstPorts)
            if dstPorts(j) ~= -1
                fprintf('  Out%d -> %s\n', i, strrep(get_param(dstPorts(j), 'Parent'), [ypr '/'], ''));
            end
        end
    end
end

fprintf('\n=== YPR Control 안의 전체 배선 (SearchDepth 1) ===\n');
lines = find_system(ypr, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for i = 1:numel(lines)
    srcPortH = get_param(lines(i), 'SrcPortHandle');
    dstPorts = get_param(lines(i), 'DstPortHandle');
    if srcPortH == -1; continue; end
    srcBlk = strrep(get_param(srcPortH, 'Parent'), [ypr '/'], '');
    for j = 1:numel(dstPorts)
        if dstPorts(j) == -1; continue; end
        dstBlk = strrep(get_param(dstPorts(j), 'Parent'), [ypr '/'], '');
        fprintf('  %s -> %s\n', srcBlk, dstBlk);
    end
end
