%% 제어기 명세 자동 덤프 (17차, C++ 이식 검증용)
%% 목적: .slx 안 제어 경로의 모든 게인/PID/포화/전달함수/믹서 상수를 기계 추출해
%%       controller_cpp의 config와 1:1 대조 가능한 텍스트로 남긴다.
%%       손으로 옮기다 부호 하나 틀리는 사고 방지 - 모델이 직접 말하게 한다.
%% 출력: diagnose/results/controller_spec.txt (사람/diff용 평문)
%% 규칙: 대상 미발견 시 error() 즉사.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

outDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(outDir, 'dir'); mkdir(outDir); end
fid = fopen(fullfile(outDir, 'controller_spec.txt'), 'w', 'n', 'UTF-8');
if fid < 0; error('출력 파일 열기 실패'); end
pr = @(varargin) fprintf(fid, varargin{:});

pr('=== controller_spec (자동 덤프) ===\n');
pr('모델: %s / 생성 스크립트: dump_controller_spec.m\n\n', mdl);

allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');
norm1 = @(s) regexprep(s, '\s+', ' ');

% --- 1) 모든 PID 블록: 게인/필터/포화/공식 ---
pr('--- [1] PID 블록 전수 ---\n');
nPid = 0;
for i = 1:numel(allBlk)
    try
        bt = get_param(allBlk{i}, 'BlockType');
    catch
        continue;
    end
    if ~strcmp(bt, 'PID Controller') && isempty(regexp(get_param(allBlk{i},'Name'), 'PID', 'once'))
        continue;
    end
    try
        refBlk = get_param(allBlk{i}, 'ReferenceBlock');
    catch
        refBlk = '';
    end
    isPid = strcmp(bt, 'PID Controller') || contains(refBlk, 'pid_lib') || contains(refBlk, 'PID');
    if ~isPid; continue; end
    nPid = nPid + 1;
    pr('\nPID #%d: %s\n', nPid, norm1(strrep(allBlk{i}, newline, '|')));
    for fld = {'Controller','P','I','D','N','InitialConditionForIntegrator', ...
               'UpperSaturationLimit','LowerSaturationLimit','LimitOutput', ...
               'AntiWindupMode','SampleTime','IntegratorMethod','FilterMethod'}
        try
            v = get_param(allBlk{i}, fld{1});
            if ~ischar(v); v = mat2str(v); end
            pr('    %s = %s\n', fld{1}, v);
        catch
        end
    end
end
if nPid == 0; error('PID 블록을 하나도 못 찾음 - find_system 옵션 확인'); end

% --- 2) 모든 Saturation 블록 ---
pr('\n--- [2] Saturation 블록 전수 ---\n');
nSat = 0;
for i = 1:numel(allBlk)
    try
        if ~strcmp(get_param(allBlk{i}, 'BlockType'), 'Saturate'); continue; end
    catch
        continue;
    end
    nSat = nSat + 1;
    pr('SAT #%d: %s | Upper=%s Lower=%s\n', nSat, ...
        norm1(strrep(allBlk{i}, newline, '|')), ...
        get_param(allBlk{i}, 'UpperLimit'), get_param(allBlk{i}, 'LowerLimit'));
end

% --- 3) Maneuver Controller 하위 Gain/TransferFcn/Bias 상수 ---
pr('\n--- [3] 제어기 하위 Gain / TransferFcn / Constant ---\n');
for i = 1:numel(allBlk)
    if isempty(regexp(allBlk{i}, 'Maneuver|Position Control|YPR|Altitude|Motor', 'once'))
        continue;
    end
    try
        bt = get_param(allBlk{i}, 'BlockType');
    catch
        continue;
    end
    nm = norm1(strrep(allBlk{i}, newline, '|'));
    switch bt
        case 'Gain'
            pr('GAIN: %s = %s\n', nm, get_param(allBlk{i}, 'Gain'));
        case 'TransferFcn'
            pr('TF:   %s | num=%s den=%s\n', nm, ...
                get_param(allBlk{i}, 'Numerator'), get_param(allBlk{i}, 'Denominator'));
        case 'Constant'
            pr('CONST:%s = %s\n', nm, get_param(allBlk{i}, 'Value'));
        case 'Sum'
            pr('SUM:  %s | signs=%s\n', nm, get_param(allBlk{i}, 'Inputs'));
    end
end

% --- 4) 워크스페이스 실효값 (스케일 곱 적용 후) ---
pr('\n--- [4] parameters.m 실효 게인 (오늘 로드값) ---\n');
vars = {'kp_position','ki_position','kd_position','filtD_position','pos2attitude','posErrSat', ...
        'kp_attitude','ki_attitude','kd_attitude','filtD_attitude','limit_attitude', ...
        'kp_yaw','ki_yaw','kd_yaw','filtD_yaw','limit_yaw', ...
        'kp_altitude','ki_altitude','kd_altitude','filtD_altitude','limit_altitude', ...
        'kp_motor','ki_motor','limit_motor','sT','sQ','sIa','sIz','sM'};
for v = vars
    if exist(v{1}, 'var')
        pr('%s = %.10g\n', v{1}, eval(v{1}));
    else
        pr('%s = (미정의!)\n', v{1});
    end
end

fclose(fid);
fprintf('덤프 완료: %s\n', fullfile(outDir, 'controller_spec.txt'));
