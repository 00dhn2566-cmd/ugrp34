%% 게인을 바꿔도 결과가 완전히 똑같이 나온 게 진짜(포화/오버라이드) 때문인지,
%% 아니면 게인 자체가 실제로 안 먹은 건지 확인.
%% (1) sim() 직전/직후 kp_altitude 등이 base workspace에 실제로 반영됐는지 확인
%% (2) "Pause Motor" 서브시스템이 모터 명령을 강제로 덮어쓰는지 확인

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% Pause Motor 서브시스템 내부 확인
pm = [mdl '/Maneuver Controller/Altitude and  YPR Control/Pause Motor'];
fprintf('=== Pause Motor 서브시스템 전체 블록 ===\n');
blks = find_system(pm, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
for i = 1:numel(blks)
    if strcmp(blks{i}, pm); continue; end
    bt = get_param(blks{i}, 'BlockType');
    extra = '';
    try
        switch bt
            case 'Constant'; extra = sprintf(' Value=%s', get_param(blks{i}, 'Value'));
            case 'Gain';     extra = sprintf(' Gain=%s', get_param(blks{i}, 'Gain'));
            case 'Sum';      extra = sprintf(' Inputs=%s', get_param(blks{i}, 'Inputs'));
            case 'Bias';     extra = sprintf(' Bias=%s', get_param(blks{i}, 'Bias'));
            case 'Switch';   extra = sprintf(' Criteria=%s Threshold=%s', get_param(blks{i},'Criteria'), get_param(blks{i},'Threshold'));
            case 'RelationalOperator'; extra = sprintf(' Operator=%s', get_param(blks{i}, 'Operator'));
            case 'Enable';   extra = ' (Enable port - 조건부 서브시스템 가능성)';
        end
    catch
    end
    fprintf('  %s (%s)%s\n', strrep(blks{i}, [pm '/'], ''), bt, extra);
end

% Pause Motor의 En 포트가 어디서 오는지 (무엇이 이걸 켜고 끄는지)
fprintf('\n=== Pause Motor의 En(Enable) 포트 소스 ===\n');
ph = get_param(pm, 'PortHandles');
if isfield(ph, 'Enable') && ~isempty(ph.Enable)
    lineH = get_param(ph.Enable, 'Line');
    if lineH ~= -1
        srcPortH = get_param(lineH, 'SrcPortHandle');
        fprintf('  En <- %s\n', get_param(srcPortH, 'Parent'));
    end
else
    fprintf('  Enable 포트 없음(일반 서브시스템)\n');
end

% Pause Motor 서브시스템 출력이 최종 Motor Thrust에 어떻게 연결되는지
fprintf('\n=== Pause Motor 출력 연결 ===\n');
for i = 1:numel(ph.Outport)
    lineH = get_param(ph.Outport(i), 'Line');
    if lineH ~= -1
        dstPorts = get_param(lineH, 'DstPortHandle');
        for j = 1:numel(dstPorts)
            fprintf('  Out%d -> %s\n', i, get_param(dstPorts(j), 'Parent'));
        end
    end
end

% 실제 게인 적용 여부 직접 검증: kp_altitude를 두 가지 다른 값으로 바꿔가며
% Control Thrust 블록의 실제 P 게인이 그때그때 다르게 읽히는지 확인
kp_altitude = 0.5; ki_altitude=0.1; kd_altitude=0.3;
fprintf('\n=== kp_altitude=%.2f 설정 직후, Control Thrust 블록이 실제로 읽는 P값 ===\n', kp_altitude);
ct = [mdl '/Maneuver Controller/Altitude and  YPR Control/Control Thrust'];
open_system(mdl, 'loadonly');
try
    cs = getSimulinkBlockHandle(ct);
    fprintf('  (블록 P 파라미터 심볼): %s\n', get_param(ct, 'P'));
catch e
    fprintf('  조회 실패: %s\n', e.message);
end

kp_altitude = 5.0; ki_altitude=2.0; kd_altitude=1.0;
fprintf('\n=== kp_altitude=%.2f 로 변경 직후, 같은 조회 ===\n', kp_altitude);
try
    fprintf('  (블록 P 파라미터 심볼): %s\n', get_param(ct, 'P'));
catch e
    fprintf('  조회 실패: %s\n', e.message);
end
fprintf('  (참고: 블록의 P는 "attitude_kp" 같은 심볼 문자열 자체를 담고 있어서 변경 여부와 무관하게 항상 같은 문자열로 보일 수 있음 - 실제 숫자값은 sim 컴파일 시점에 workspace에서 평가됨)\n');
