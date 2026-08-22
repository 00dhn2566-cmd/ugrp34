%% yaw PID 블록 정찰 — 외란 적응 적분(dynamic Ki) 배선 방식 결정용
%% 2026-08-22 외란 강건화 세션. docs/YAW_DISTURBANCE_I.md §7 "열린 항목" 해소.
%% 판정할 것:
%%   Q1. 'Control Yaw' 블록 타입 / 마스크 (Simulink PID Controller 인가)
%%   Q2. ControllerParametersSource(외부 P/I/D 포트) 지원 여부 + 켰을 때 포트 순서·개수
%%   Q3. 현재 파라미터 (Form/적분기 방식/필터/출력 제한)
%%   Q4. Inport 1 소스 (오차 신호) / Outport 1 목적지
%% 규칙: save_system 금지 (읽기 + 임시 in-memory 조작만, 끝나면 close 무저장)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

yawBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Control Yaw');
fprintf('=== Q1: Control Yaw 후보 %d개 ===\n', numel(yawBlk));
for i = 1:numel(yawBlk)
    fprintf('  [%d] %s\n      BlockType=%s  MaskType=%s\n', i, yawBlk{i}, ...
        get_param(yawBlk{i},'BlockType'), get_param(yawBlk{i},'MaskType'));
end
if numel(yawBlk) ~= 1
    error('probe_yaw_pid: Control Yaw 가 1개가 아님 - 스크립트 수정 필요');
end
blk = yawBlk{1};

fprintf('\n=== Q3: 현재 파라미터 ===\n');
keys = {'Controller','TimeDomain','Form','IntegratorMethod','FilterMethod', ...
        'P','I','D','N','InitialConditionSource','LimitOutput', ...
        'UpperSaturationLimit','LowerSaturationLimit','AntiWindupMode', ...
        'ControllerParametersSource','UseFilter','TrackingMode','SampleTime'};
for k = 1:numel(keys)
    try
        fprintf('  %-26s = %s\n', keys{k}, num2str(get_param(blk, keys{k})));
    catch e
        fprintf('  %-26s = <없음: %s>\n', keys{k}, e.identifier);
    end
end

fprintf('\n=== Q4: 배선 ===\n');
ph = get_param(blk, 'PortHandles');
fprintf('  Inport %d개 / Outport %d개\n', numel(ph.Inport), numel(ph.Outport));
ln = get_param(ph.Inport(1), 'Line');
if ln ~= -1
    src = get_param(ln, 'SrcPortHandle');
    sb  = get_param(src, 'Parent');
    fprintf('  In1 <- %s (%s)\n', sb, get_param(sb,'BlockType'));
end
lo = get_param(ph.Outport(1), 'Line');
if lo ~= -1
    dst = get_param(lo, 'DstPortHandle');
    for d = 1:numel(dst)
        fprintf('  Out1 -> %s (%s)\n', get_param(dst(d),'Parent'), ...
                get_param(get_param(dst(d),'Parent'),'BlockType'));
    end
end

fprintf('\n=== Q2: 외부 파라미터 모드 시험 (in-memory, 되돌림) ===\n');
ok_ext = false;
try
    orig = get_param(blk, 'ControllerParametersSource');
    set_param(blk, 'ControllerParametersSource', 'external');
    ph2 = get_param(blk, 'PortHandles');
    fprintf('  external 설정 성공: Inport %d개 (원래 %d개)\n', numel(ph2.Inport), numel(ph.Inport));
    for i = 1:numel(ph2.Inport)
        try
            nm = get_param(ph2.Inport(i), 'Name');
        catch
            nm = '?';
        end
        fprintf('    In%d 이름=%s\n', i, nm);
    end
    ok_ext = true;
    set_param(blk, 'ControllerParametersSource', orig);
    fprintf('  원복 완료 (%s)\n', get_param(blk,'ControllerParametersSource'));
catch e
    fprintf('  external 미지원/실패: %s\n', e.message);
end

fprintf('\n=== 결론 ===\n');
if ok_ext
    fprintf('  경로 A 가능: I 포트를 밖으로 빼서 ki_yaw*g(t) 를 직접 먹인다 (식 정확 일치).\n');
else
    fprintf('  경로 B 필요: 병렬 보조 적분 브랜치 (Integrator + Fcn) 삽입.\n');
end
fprintf('  ※ save_system 하지 않음. 모델은 무저장 상태로 남겨 둠.\n');
