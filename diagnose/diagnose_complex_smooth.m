%% 복잡 입력 종합 시험: 3D 박스 투어 (x/y/z 동시 + 모서리 방향 전환 + 고도 변화)
%% 파이프라인 전체 검증: 거친 입력(등속 직선, 모서리 속도 불연속) -> traj_gate 차단 확인
%%   -> traj_smoother 성형 -> 게이트 통과 -> 구운 모델 실비행 -> 추종 성능 판정.
%% 게인/모델은 현행 구운 상태 그대로 (처방 미적용 - 부드러운 입력의 실전 경로 검증).
%% 규칙: 메모리 수술만(로깅 탭), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 16; tHold = 3;

% --- 거친 원본: 등속 1.2m/s 직선 연결 박스 투어 ---
wps = [0 0 1.0; 2 0 1.5; 2 2 1.2; 0 2 0.8; 0 0 1.0];
spd = 1.2;
tt = (0:dt:T)';
N = numel(tt);
posRaw = zeros(N, 3);
segT = zeros(size(wps,1)-1, 1);
for s = 1:size(wps,1)-1
    segT(s) = norm(wps(s+1,:) - wps(s,:)) / spd;
end
tKnot = [0; cumsum(segT)] + tHold;
for k = 1:N
    t = tt(k);
    if t <= tKnot(1)
        posRaw(k,:) = wps(1,:);
    elseif t >= tKnot(end)
        posRaw(k,:) = wps(end,:);
    else
        s = find(t >= tKnot(1:end-1) & t < tKnot(2:end), 1);
        tau = (t - tKnot(s)) / segT(s);
        posRaw(k,:) = wps(s,:) + tau * (wps(s+1,:) - wps(s,:));
    end
end

fprintf('===== 1) 게이트: 거친 원본 (등속 직선, 모서리 불연속) =====\n');
[okR, repR] = traj_gate(tt, posRaw, VMAX, AMAX, false);
fprintf('  판정: %s (vxy %.2f axy %.2f vz %.2f az %.2f) <- 차단돼야 정상\n', ...
    tern(okR,'통과','차단'), repR.vxyPk, repR.axyPk, repR.vzPk, repR.azPk);

fprintf('\n===== 2) 스무더 성형 (xy는 한계/√2 축배분 - 성형기 원칙 3) -> 게이트 재검 =====\n');
axF = 0.7;   % xy 축배분 계수 (0.7071 이하 - 동시 기동 시 노름 <= 한계 보장)
[posXY, infXY] = traj_smoother(tt, posRaw(:,1:2), VMAX*axF, AMAX*axF, JMAX);
[posZ,  infZ]  = traj_smoother(tt, posRaw(:,3),   VMAX,     AMAX,     JMAX);
posS = [posXY, posZ];
infS.vPk = [infXY.vPk, infZ.vPk];
infS.aPk = [infXY.aPk, infZ.aPk];
infS.maxDev = [infXY.maxDev, infZ.maxDev];
[okS, repS] = traj_gate(tt, posS, VMAX, AMAX, false);
fprintf('  판정: %s (vxy %.2f axy %.2f) | 성형 피크 v %.2f a %.2f | 원본 대비 최대 이탈 x %.2f y %.2f z %.2f m\n', ...
    tern(okS,'통과','차단'), repS.vxyPk, repS.axyPk, max(infS.vPk), max(infS.aPk), infS.maxDev(1), infS.maxDev(2), infS.maxDev(3));
if ~okS; error('성형본이 게이트 불통과 - 중단'); end

%% ===== 3) 실비행 =====
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

waypoints = unique(wps, 'rows', 'stable')';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', posS);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element1','py'; 'In Bus Element2','pz'; ...
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

fprintf('\n===== 3) 실비행 (박스 투어 %.1fs 구간, 총 %ds) =====\n', tKnot(end)-tHold, T);
sim(mdl);

tu = (0:0.005:T)';
gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
xg = gi(px); yg = gi(py); zg = gi(pz);
pg = rad2deg(gi(real_pitch)); rg = rad2deg(gi(real_roll)); yw = rad2deg(gi(real_yaw));
wCeil = 1025;
W = [abs(gi(w1)), abs(gi(w2)), abs(gi(w3)), abs(gi(w4))] / wCeil * 100;
xr = interp1(tt, posS(:,1), tu); yr = interp1(tt, posS(:,2), tu); zr = interp1(tt, posS(:,3), tu);
ex = xg - xr; ey = yg - yr; ez = zg - zr;
e3 = sqrt(ex.^2 + ey.^2 + ez.^2);

fprintf('  시각 | 기준(x,y,z)          실제(x,y,z)          | 3D오차cm | P     R     Y   | 모터max\n');
for ct = [3 4 5 6 7 8 9 10 11 12 14 16]
    [~,i2] = min(abs(tu-ct));
    fprintf('  t=%4.1f | %5.2f %5.2f %5.2f -> %5.2f %5.2f %5.2f | %7.1f | %+5.1f %+5.1f %+5.1f | %3.0f%%\n', ...
        tu(i2), xr(i2), yr(i2), zr(i2), xg(i2), yg(i2), zg(i2), e3(i2)*100, pg(i2), rg(i2), yw(i2), max(W(i2,:)));
end
iM = tu >= tHold & tu <= tKnot(end) + 1;   % 이동 구간
iT = tu >= tKnot(end) + 1;                 % 도착 후
fprintf('\n===== 판정 =====\n');
fprintf('  이동 중: 3D 추종 RMS %.1fcm / 피크 %.1fcm | 자세 피크 %.1f도 | yaw 피크 %.1f도 | 모터 피크 %.0f%%\n', ...
    sqrt(mean(e3(iM).^2))*100, max(e3(iM))*100, max(max(abs(pg(iM))), max(abs(rg(iM)))), max(abs(yw(iM))), max(W(iM,:),[],'all'));
fprintf('  도착 후: 3D 오차 RMS %.1fcm | 자세 RMS %.2f도 (1.75Hz 짐 흔들림 포함 - 기지 사항)\n', ...
    sqrt(mean(e3(iT).^2))*100, sqrt(mean((pg(iT)-mean(pg(iT))).^2)));
fprintf('  z 범위 %.2f~%.2fm (기준 0.8~1.5 투어)\n', min(zg), max(zg));
ok = max(e3(iM)) < 0.3 && max(W(iM,:),[],'all') < 100 && sqrt(mean(e3(iT).^2)) < 0.05;
fprintf('  >> %s\n', tern(ok, '합격 - 복잡 3D 입력도 스무더 경로로 정상 비행', '불합격 - 항목 확인'));

function s = tern(c, a, b)
    if c; s = a; else; s = b; end
end
