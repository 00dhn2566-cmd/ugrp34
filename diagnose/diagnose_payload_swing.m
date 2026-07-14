%% 1.75Hz 무감쇠 pitch 왕복의 범인 확정: 매달린 패키지 진자 가설
%% 근거: 주파수가 모든 게인(kp 8/4, kd 3.2/1.6/0, ki_att 0/-10/-30)에서 1.75Hz 불변,
%%       진폭은 kd에만 비례, 감쇠비 ~1.0 (제어 루프 바깥의 물리 공진 시그니처).
%%       진자 환산 L = g/(2*pi*f)^2 = 8.1cm (짐-앵커 거리와 비교할 것).
%% 실험: 짐 밀도 1000분의 1 (1kg -> 1g)로 진자 에너지 제거 후 같은 비행.
%%       1.75Hz 소멸 -> 짐 진자 확정 / 잔존 -> 가설 기각.
%% 주의: 짐 질량이 빠지면 총중량 감소로 z가 살짝 뜰 수 있음(추력 bias는 구운 값).
%%       z 이탈은 진단 결과에 무관 - pitch 스펙트럼만 본다.
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
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'};
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
cases = { '기준(짐 1kg)', D0;
          '짐 1g',        D0*1e-3 };

fprintf('===== 패키지 진자 가설 확정 실험 (게인은 현행 고정) =====\n');
for ci = 1:size(cases,1)
    pkgDensity = cases{ci,2};
    fprintf('\n--- 케이스 %d: %s (pkgDensity=%g) ---\n', ci, cases{ci,1}, pkgDensity);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    tu = (0:0.005:T)';
    xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    rg = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), tu, 'linear', 'extrap'));
    zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
    xrg = interp1(tt, smKill(:,1), tu, 'linear', 'extrap');
    seg = @(t1,t2) (tu>=t1 & tu<t2);
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    r1 = rmsf(pg(seg(6,9)));
    r2 = rmsf(pg(seg(9,12)));
    pgP = pg(seg(6,12)); pgP = pgP - mean(pgP);
    freq = sum(abs(diff(sign(pgP)))>0)/2/6;
    exR = sqrt(mean((xrg(seg(6,12))-xg(seg(6,12))).^2))*100;
    fprintf('  pitch RMS 6~9s %.3f도 / 9~12s %.3f도 / 비율 %.2f | 주파수 %.2fHz | roll RMS %.3f도\n', ...
        r1, r2, r2/tern0(r1), freq, rmsf(rg(seg(6,12))));
    fprintf('  x오차 %.2fcm | z 범위 %.2f~%.2fm (질량 변화로 이탈 가능 - 참고용)\n', ...
        exR, min(zg(seg(3,12))), max(zg(seg(3,12))));
end
fprintf('\n(판정: 짐 1g에서 pitch RMS 크게 감소 + 1.75Hz 소멸 -> 진자 확정.\n');
fprintf(' 확정 시 처방은 게인이 아니라 스무더의 진자 대역 회피(input shaping) 또는 스윙 댐핑 - 연구주제 본론.)\n');

function v = tern0(x)
    if x == 0; v = inf; else; v = x; end
end
