%% 명령 스무더(traj_gate + traj_smoother) 검증 (2026-07-14 스펙)
%% 1부(시뮬 없음): 정상 궤적 → 게이트 통과 + 스무더 무개입 / 발산 궤적(1m/0.67s)
%%   → 게이트 적발 → 스무더 성형 → 게이트 재통과.
%% 2부(시뮬 1회): 성형된 발산 궤적을 구운 모델에 투입 → 안정 비행 확인.
%%   비교 기준(같은 궤적 원본, 어제 실측): 최대|x| 12.05m / 자세 44.5도 / yaw 62.2도 / 발산.
%% 모터 4개 개별 출력(천장 1025rad/s 대비 %) 동시 로깅 - 제어입력 강도 확인.
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

% envelope 실측 2.5/2.5에 여유율 적용
VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;

dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
mj = @(Tm) A * (10*min(max((tt-tStep)/Tm,0),1).^3 - 15*min(max((tt-tStep)/Tm,0),1).^4 + 6*min(max((tt-tStep)/Tm,0),1).^5);

fprintf('===== 1부: 게이트/스무더 단위 검증 (한계 v%.1f a%.1f j%.1f) =====\n', VMAX, AMAX, JMAX);

% --- 정상 궤적: 1m를 3s 최소저크 (피크 v 0.63, a 0.64 - envelope 안) ---
xs = mj(3.0);
posSafe = [xs, zeros(N,1), ones(N,1)];
[ok1, rep1] = traj_gate(tt, posSafe, VMAX, AMAX, false);
[smSafe, infS] = traj_smoother(tt, posSafe, VMAX, AMAX, JMAX);
fprintf('[정상 3.0s] 게이트: %s (vxy %.2f axy %.2f) | 스무더 최대개입 %.4fm (0이어야 정상)\n', ...
    tern(ok1,'통과','차단'), rep1.vxyPk, rep1.axyPk, max(infS.maxDev));

% --- 발산 궤적: 1m를 0.67s (피크 v 2.8, a 12.9 - 어제 발산 확정 조건) ---
xk = mj(0.67);
posKill = [xk, zeros(N,1), ones(N,1)];
[ok2, rep2] = traj_gate(tt, posKill, VMAX, AMAX, false);
fprintf('[발산 0.67s] 게이트: %s (vxy %.2f/%.1f axy %.2f/%.1f) <- 적발돼야 정상\n', ...
    tern(ok2,'통과','차단'), rep2.vxyPk, VMAX, rep2.axyPk, AMAX);
if ok2; error('게이트가 발산 궤적을 통과시킴 - 게이트 무효'); end

[smKill, infK] = traj_smoother(tt, posKill, VMAX, AMAX, JMAX);
[ok3, rep3] = traj_gate(tt, smKill, VMAX, AMAX, false);
iEnd = find(abs(smKill(:,1) - A) < 0.01, 1);
fprintf('[성형 후] 게이트: %s (vxy %.2f axy %.2f) | 성형 피크 v %.2f a %.2f j %.1f | 1cm 도달 %.2fs (원본 0.67s)\n', ...
    tern(ok3,'통과','차단'), rep3.vxyPk, rep3.axyPk, infK.vPk(1), infK.aPk(1), infK.jPk(1), ...
    tern2(isempty(iEnd), NaN, tt(min([iEnd N]))-tStep));
if ~ok3; error('성형 출력이 게이트 불통과 - 스무더 무효'); end

%% ===== 2부: 성형 궤적 실비행 =====
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

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

% --- 궤적 주입 (성형본) ---
spline_data = smKill;
timespot_spl = tt;
spline_yaw = zeros(N,1);
waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', timespot_spl);
mws.assignin('spline_data', spline_data);
mws.assignin('spline_yaw', spline_yaw);
set_param(mdl, 'StopTime', num2str(T));

% --- 로깅: 위치/자세/yaw + 모터 4개 개별 ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; 'In Bus Element5','real_yaw'; ...
          'In Bus Element11','w1'; 'In Bus Element10','w2'; 'In Bus Element12','w3'; 'In Bus Element13','w4'};
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

fprintf('\n===== 2부: 성형 궤적(원본=발산 확정 0.67s 스텝) 실비행 =====\n');
sim(mdl);

t = px.time(:);
x = px.signals.values(:);
zv = interp1(pz.time(:), pz.signals.values(:), t, 'linear', 'extrap');
r = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), t, 'linear', 'extrap'));
pch = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), t, 'linear', 'extrap'));
yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), t, 'linear', 'extrap'));
xr = interp1(tt, smKill(:,1), t, 'linear', 'extrap');
wCeil = 1025;   % rad/s (실측 천장. 호버 635 = 62%)
W = zeros(numel(t), 4);
wsrc = {w1, w2, w3, w4};
for i = 1:4
    W(:,i) = abs(interp1(wsrc{i}.time(:), wsrc{i}.signals.values(:), t, 'linear', 'extrap'));
end
Wpct = W / wCeil * 100;

fprintf('  시각  |  기준x   실제x   오차cm |  P      R      Y    |  z    | 모터%%(1/2/3/4)\n');
for ct = [3 3.5 4 4.5 5 5.5 6 7 8 10 12]
    [~, i2] = min(abs(t - ct));
    fprintf('  t=%4.1f | %6.3f %7.3f %7.1f | %5.1f %6.1f %6.1f | %5.2f | %4.0f %4.0f %4.0f %4.0f\n', ...
        t(i2), xr(i2), x(i2), (x(i2)-xr(i2))*100, pch(i2), r(i2), yw(i2), zv(i2), ...
        Wpct(i2,1), Wpct(i2,2), Wpct(i2,3), Wpct(i2,4));
end

err = x - xr;
i3 = find(t >= tStep, 1);
xMax = max(abs(x));
eRms = sqrt(mean(err(i3:end).^2)) * 100;
ePk = max(abs(err(i3:end))) * 100;
attPk = max(max(abs(r(i3:end))), max(abs(pch(i3:end))));
ywPk = max(abs(yw(i3:end)));
wPk = max(Wpct(:));
wHovPct = 635 / wCeil * 100;
% 정착: 최종 1m에 ±5cm
tSet = NaN;
okm = abs(x - A) < 0.05;
for ii = i3:numel(t)
    if all(okm(ii:end)); tSet = t(ii) - tStep; break; end
end
% 자세 기준 16도: a=2m/s2에 물리적으로 필요한 기울기 atan(2/9.81)=11.5도 + 과도 마진.
% (10도로 잡으면 정상 비행도 불안정 오판 - 12차 실측 13.2도)
stab = xMax < 1.5 && attPk < 16 && min(zv(i3:end)) > 0.9 && ~isnan(tSet);

fprintf('\n===== 판정 =====\n');
fprintf('  최대|x| %.2fm (어제 원본: 12.05m) / 정착(±5cm) %.2fs / 추종오차 RMS %.1fcm 피크 %.1fcm\n', xMax, tSet, eRms, ePk);
fprintf('  자세피크 %.2f도 (어제 원본: 44.5도) / yaw피크 %.2f도 (어제: 62.2도) / z 최저 %.2fm\n', attPk, ywPk, min(zv(i3:end)));
fprintf('  모터 피크 %.0f%% (호버 %.0f%%, 포화=100%%) - 어제 원본은 103~105%% 포화\n', wPk, wHovPct);
fprintf('  >> %s\n', tern(stab, '안정 - 스무더가 발산 시나리오 차단 성공', '불안정 - 스펙 재검토 필요'));

function s = tern(c, a, b)
    if c; s = a; else; s = b; end
end
function v = tern2(c, a, b)
    if c; v = a; else; v = b; end
end
