%% 진자 확정 2탄: 질량 불변 주파수 검사 (짐 1kg -> 2kg) + Load 조인트 구조 조사
%% 진자 지문: f = sqrt(g/L)/2pi 는 질량 무관. 2kg에서도 1.75Hz면 진자 확정.
%% (1g 실험은 1.75Hz 소멸을 보였으나 트림 붕괴로 오염 - 이번엔 트림 유지 방향)
%% 겸사: Quadcopter/Load 하위의 Joint 블록 목록 출력 - 진자 자유도와 길이 지오메트리 확인.
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smKill = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- Load 하위 Joint/Body 구조 조사 (시뮬 전, 정보 수집) ---
fprintf('===== Quadcopter/Load 하위 Joint 블록 =====\n');
loadSys = [mdl '/Quadcopter/Load'];
jnts = find_system(loadSys, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', 'BlockType', 'SubSystem|.*Joint.*');
for i = 1:numel(jnts)
    nm = strtrim(regexprep(jnts{i}, '\s+', ' '));
    try
        mt = get_param(jnts{i}, 'MaskType');
    catch
        mt = '?';
    end
    if contains(lower(nm), 'joint') || contains(lower(mt), 'joint')
        fprintf('  [JOINT] %s (MaskType: %s)\n', nm, mt);
    end
end
allBlk = find_system(loadSys, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
fprintf('  (Load 하위 총 %d블록. Joint 미발견 시 위 목록 방식 한계 - 수동 조사 필요)\n', numel(allBlk));

% --- 투하 로직 무력화 ---
dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(dropBlocks{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(dropBlocks)
    set_param(dropBlocks{i}, 'Value', '-1');
end

waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', smKill);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
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

D0 = pkgDensity;
cases = { '짐 2kg (질량 2배)', D0*2 };

fprintf('\n===== 질량 2배 주파수 불변 검사 (기준 1kg: 1.75Hz, RMS 3.94) =====\n');
for ci = 1:size(cases,1)
    pkgDensity = cases{ci,2};
    fprintf('\n--- %s (pkgDensity=%g) ---\n', cases{ci,1}, pkgDensity);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    tu = (0:0.005:T)';
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    rg = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), tu, 'linear', 'extrap'));
    zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
    xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
    xrg = interp1(tt, smKill(:,1), tu, 'linear', 'extrap');
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    r1 = rmsf(pg(seg(6,9))); r2 = rmsf(pg(seg(9,12)));
    pgP = pg(seg(6,12)); pgP = pgP - mean(pgP);
    freq = sum(abs(diff(sign(pgP)))>0)/2/6;
    fprintf('  pitch RMS 6~9s %.3f / 9~12s %.3f / 비율 %.2f | 주파수 %.2fHz | roll RMS %.3f\n', ...
        r1, r2, r2/r1, freq, rmsf(rg(seg(6,12))));
    fprintf('  x오차 %.2fcm | z 범위 %.2f~%.2fm\n', ...
        sqrt(mean((xrg(seg(6,12))-xg(seg(6,12))).^2))*100, min(zg(seg(3,12))), max(zg(seg(3,12))));
end
fprintf('\n(판정: 2kg에서도 ~1.75Hz면 질량 불변 -> 진자 확정. 주파수가 뚜렷이 이동하면 기각.)\n');
