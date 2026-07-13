%% 강건성 ②: CG 오프셋 ±5mm 호버 유지 테스트
%% 원리: 구운 모델의 Plate Anchor Comp 보정값을 ±10.5mm 흔들면 몸체 스택이 그만큼 이동,
%%       스택/전체 질량비 0.477 (실측: 앵커 30mm 이동 -> CoM 14.3mm)에 의해 실효 CoM이 약 ±5mm 이동.
%%       추력선은 그대로이므로 상수 CoM-추력 오프셋 토크(~0.1 N·m) 하에서 호버 유지력을 본다.
%% 케이스: x+/x-/y+/y- 4방향, 각 10초 호버. 한 프로세스에서 순차 실행 (동시 실행 금지 - RAM).
%% 규칙: 대상 미발견 시 error() 즉사. save_system 금지 (메모리 수정만).

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 앵커 보정 블록 확인 ---
compBlk = [mdl '/Quadcopter/Body/Body/Plate Anchor Comp'];
try
    baseStr = get_param(compBlk, 'TranslationCartesianOffset');
catch
    error('Plate Anchor Comp 못 찾음 (%s) - 구운 모델 맞는지 확인. 실행 무효', compBlk);
end
baseOff = str2num(baseStr); %#ok<ST2NM>
if numel(baseOff) ~= 3; error('보정값 파싱 실패: %s', baseStr); end
fprintf('기준 앵커 보정 [mm]: [%.4f %.4f %.4f]\n', baseOff);

% 링크 완전 해제 (메모리에서만 - 저장 안 함). inactive로는 내부 수정 시 시뮬이 거부함
% ("비활성화된 라이브러리 링크에 수정된 요소가 있을 수 없음" 오류 실측)
p = get_param(compBlk, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        ls = get_param(p, 'LinkStatus');
        if any(strcmp(ls, {'resolved', 'inactive'}))
            set_param(p, 'LinkStatus', 'none');
            fprintf('링크 해제(none): %s (이전: %s)\n', regexprep(p, '\s+', ' '), ls);
        end
    catch
    end
    p = get_param(p, 'Parent');
end

% --- 궤적/워크스페이스: 10초 호버 ---
dt = 0.01; T = 10; N = round(T/dt) + 1;
timespot_spl = (0:N-1)' * dt;
hoverPoint = [0, 0, 1.0];
spline_data = repmat(hoverPoint, N, 1);
spline_yaw = zeros(N, 1);
waypoints = [hoverPoint; hoverPoint + [0 0 2]]';
wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', wayp_path_vis);
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));

% --- 로깅 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% --- 4방향 순차 실행 ---
dAnchor = 10.5;  % mm (실효 CoM ~5mm = 10.5 x 0.477)
cases = {'x+', [ dAnchor 0 0]; 'x-', [-dAnchor 0 0]; 'y+', [0  dAnchor 0]; 'y-', [0 -dAnchor 0]};
fprintf('\n===== CG 오프셋 강건성 (앵커 ±%.1fmm = 실효 CoM ~±5mm, 각 %gs 호버) =====\n', dAnchor, T);
results = {};
for ci = 1:size(cases, 1)
    tag = cases{ci,1};
    off = baseOff + cases{ci,2};
    set_param(compBlk, 'TranslationCartesianOffset', sprintf('[%.6g %.6g %.6g]', off));
    fprintf('\n--- 케이스 %s: 앵커 [%.2f %.2f %.2f]mm ---\n', tag, off);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        results(end+1,:) = {tag, NaN, NaN, NaN, NaN, false}; %#ok<SAGROW>
        continue;
    end
    t = real_roll.time(:);
    r = rad2deg(real_roll.signals.values(:));
    pch = rad2deg(real_pitch.signals.values(:));
    zv = real_z.signals.values(:);
    for ct = 0:2:T
        [~, idx] = min(abs(t - ct));
        fprintf('  t=%4.1f | roll %6.2f pitch %6.2f | z %6.3f\n', t(idx), r(idx), pch(idx), zv(idx));
    end
    mask = t > 2;
    rmsA = sqrt(mean(r(mask).^2 + pch(mask).^2));
    % 정상상태 기울기(마지막 3초 평균) - 상수 오프셋 보상량
    maskSS = t > T-3;
    ssR = mean(r(maskSS)); ssP = mean(pch(maskSS));
    surv = (min(zv) > 0.3) && (max(abs(r)) < 45) && (max(abs(pch)) < 45);
    fprintf('  >> RMS %.2f도 / 최대|R| %.1f |P| %.1f / 정상상태 roll %.2f pitch %.2f / z [%.2f %.2f] / %s\n', ...
        rmsA, max(abs(r)), max(abs(pch)), ssR, ssP, min(zv), max(zv), ternstr(surv, '생존', '추락'));
    results(end+1,:) = {tag, rmsA, max(abs([r; pch])), ssR, ssP, surv}; %#ok<SAGROW>
end

% 원복 (파일은 안 건드렸지만 메모리 상태도 되돌림)
set_param(compBlk, 'TranslationCartesianOffset', baseStr);

fprintf('\n===== 요약 =====\n');
fprintf('%4s | %8s | %8s | %14s | %s\n', '케이스', 'RMS[도]', '최대[도]', '정상상태 R/P', '판정');
allPass = true;
for ci = 1:size(results, 1)
    ok = results{ci,6} && results{ci,2} < 3.0;
    allPass = allPass && ok;
    fprintf('%4s | %8.2f | %8.2f | %6.2f / %5.2f | %s\n', results{ci,1}, results{ci,2}, results{ci,3}, results{ci,4}, results{ci,5}, ternstr(ok, '합격', '불합격'));
end
if allPass
    fprintf('>> 전체 합격: CoM ±5mm 오프셋에서도 호버 유지. 다음: 패키지 드롭\n');
else
    fprintf('>> 일부 불합격: 수치 검토 필요\n');
end

function s = ternstr(c, a, b)
    if c; s = a; else; s = b; end
end
