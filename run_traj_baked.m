%% run_traj_baked.m - trajectory.mat -> 구운 자립 모델 실행 -> 위치/자세 추종 성능 요약
%% (2026-07-13 작성. 다음 세션의 "JSON path -> 컨트롤러 명령" 파이프라인용 글루 코드.
%%  아직 미실행/미검증 - 첫 실행 시 반드시 로그 전체 확인. HANDOFF_PATH_TO_CONTROLLER.md 참고)
%%
%% 사용법 (이 폴더에서):
%%   1) python ../../sample/waypoints_to_maneuver_input.py  (trajectory.mat 생성)
%%   2) "/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "run_traj_baked"
%%
%% 입력: 이 폴더의 trajectory.mat
%%   timespot_spl (N,)  시간 breakpoint [s]
%%   spline_data  (N,3) x,y,z 목표 위치 [m]   <- N×3 그대로 사용 (Lookup Table이 열로 읽음)
%%   spline_yaw   (N,)  yaw [rad]
%%   waypoints    (M,3) 경유점                <- 블록은 3×M을 요구하므로 여기서 전치함
%%
%% 전제: Models/quadcopter_package_delivery.slx 는 구운 자립 모델
%%       (앵커 보정/Bias/클램프 내장, 게인은 Scripts_Data/quadcopter_package_parameters.m).
%%       모델 구조는 건드리지 않음. 로깅 To Workspace만 메모리에서 추가. save_system 절대 금지.

modelDir = fileparts(mfilename('fullpath'));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 궤적 로드 + 형식 검증 (침묵 no-op 금지: 이상하면 즉사) ---
trajFile = fullfile(modelDir, 'trajectory.mat');
if ~exist(trajFile, 'file'); error('trajectory.mat 없음: %s', trajFile); end
S = load(trajFile);
req = {'timespot_spl', 'spline_data', 'spline_yaw', 'waypoints'};
for i = 1:numel(req)
    if ~isfield(S, req{i}); error('trajectory.mat에 %s 없음', req{i}); end
end
timespot_spl = S.timespot_spl(:);
spline_data  = S.spline_data;
spline_yaw   = S.spline_yaw(:);
if size(spline_data, 2) ~= 3
    error('spline_data는 N×3이어야 함 (지금 %d×%d)', size(spline_data, 1), size(spline_data, 2));
end
if numel(timespot_spl) ~= size(spline_data, 1)
    error('timespot_spl(%d)과 spline_data 행수(%d) 불일치', numel(timespot_spl), size(spline_data, 1));
end
waypoints = S.waypoints';   % 파이썬 저장 N×3 -> Waypoints 블록 요구 3×M
if size(waypoints, 1) ~= 3
    error('전치 후 waypoints가 3×M이 아님 (%d×%d) - 저장 형식 확인', size(waypoints, 1), size(waypoints, 2));
end
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
fprintf('궤적: %d점, T=%.2fs, 경유점 %d개\n', numel(timespot_spl), timespot_spl(end), size(waypoints, 2));

% --- 모델 워크스페이스에 주입 (검증된 패턴: verify_hover.m과 동일) ---
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
T = timespot_spl(end);
% 도착 후 잔류 지터(tail) 관측 마진: Lookup Table은 마지막 breakpoint 값을
% 유지하므로 T 이후는 최종 위치 hold = attitude_feedback tail 분석 구간.
T_hold = 8;
set_param(mdl, 'StopTime', num2str(T + T_hold));

% --- Scope 버스 신호 매핑 출력 (신호 추가 태핑 시 근거 자료) ---
scope = [mdl '/Scope'];
ibs = find_system(scope, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^In Bus Element');
fprintf('\nScope 버스 신호 매핑 (참고):\n');
for i = 1:numel(ibs)
    el = '(Element 파라미터 없음)';
    try; el = get_param(ibs{i}, 'Element'); catch; end
    fprintf('  %-22s -> %s\n', strtrim(regexprep(get_param(ibs{i}, 'Name'), '\s+', ' ')), el);
end

% --- 자세 로깅 태핑 (검증된 매핑: Element2=z, Element3=pitch, Element4=roll) ---
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
for i = 1:size(sigMap, 1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% --- 시간축 로깅 (첫 실행에서 확인: act_*/des_*는 SaveFormat Array = 시간 없는
%     double 배열. run_sample_sim.m과 동일하게 Clock -> sim_time 동승) ---
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Sim Time Clock'))
    add_block('simulink/Sources/Clock', [mdl '/Sim Time Clock']);
    add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace sim_time'], ...
        'VariableName', 'sim_time', 'SaveFormat', 'Array');
    clockPh = get_param([mdl '/Sim Time Clock'], 'PortHandles');
    twPh    = get_param([mdl '/To Workspace sim_time'], 'PortHandles');
    add_line(mdl, clockPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% --- 실행 ---
fprintf('\n=== 궤적 추종 시뮬 (T=%gs) ===\n', T);
sim(mdl);

% --- 자세 요약 ---
t = real_roll.time(:);
r = rad2deg(real_roll.signals.values(:));
pch = rad2deg(real_pitch.signals.values(:));
zv = real_z.signals.values(:);
mask = t > 1;
fprintf('자세: RMS %.2f도 / 최대|roll| %.1f도 / 최대|pitch| %.1f도\n', ...
    sqrt(mean(r(mask).^2 + pch(mask).^2)), max(abs(r)), max(abs(pch)));

% --- 위치 추종 요약 (모델 내장 로그 신호 act_*/des_* 사용, run_sample_sim.m과 동일 소스) ---
axesNames = {'x', 'y', 'z'};
trk = struct();
for ai = 1:3
    an = axesNames{ai};
    actName = sprintf('act_%s1', an);
    desName = sprintf('des_%s1', an);
    if ~exist(actName, 'var') || ~exist(desName, 'var')
        fprintf('경고: %s/%s 로그 변수 없음 - 위치 추종(%s축) 평가 생략. (모델 내장 로거 확인 필요)\n', actName, desName, an);
        continue;
    end
    [ta, va] = extract_ts(eval(actName), sim_time);
    [td, vd] = extract_ts(eval(desName), sim_time);
    if isempty(ta) || isempty(td)
        fprintf('경고: %s축 로그 형식 해석 실패 (class: %s) - 생략\n', an, class(eval(actName)));
        continue;
    end
    vdi = interp1(td, vd, ta, 'linear', 'extrap');
    err = va - vdi;
    trk.(an) = struct('t', ta, 'act', va, 'des', vdi, 'err', err);
    fprintf('추종 %s축: RMS %.3fm / 최대 %.3fm / 종점오차 %.3fm\n', an, rms(err), max(abs(err)), abs(err(end)));
end

% --- 결과 저장 (act/des 원시 로그 + sim_time 동승 — Python 지터 분석기 입력) ---
outFile = fullfile(modelDir, 'sim_result_baked.mat');
save(outFile, 'trk', 'real_roll', 'real_pitch', 'real_z', 'timespot_spl', 'spline_data', 'spline_yaw');
extraVars = {'sim_time', 'act_x1','act_y1','act_z1', 'des_x1','des_y1','des_z1'};
for i = 1:numel(extraVars)
    if exist(extraVars{i}, 'var'); save(outFile, extraVars{i}, '-append'); end
end
fprintf('저장: %s\n', outFile);

function [t, v] = extract_ts(x, tArr)
    % timeseries / StructureWithTime / Array(double, sim_time 동승) 3형식 지원
    t = []; v = [];
    try
        if isa(x, 'timeseries')
            t = x.Time(:); v = squeeze(x.Data); v = v(:);
        elseif isstruct(x) && isfield(x, 'time') && isfield(x, 'signals')
            t = x.time(:); v = x.signals.values(:);
        elseif isnumeric(x) && nargin > 1 && ~isempty(tArr)
            v = x(:); t = tArr(:);
            n = min(numel(t), numel(v));   % Array 포맷은 행 순서 대응 (솔버 동일 스텝)
            t = t(1:n); v = v(1:n);
        end
    catch
    end
end
