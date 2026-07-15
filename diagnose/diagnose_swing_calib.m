%% 능동 스윙 소거 1단계: 교정 비행 (표준 펄스 -> 유발 진동 감도/위상 + 모드 주파수 정밀)
%% 호버 안정 후 표준 이동(10cm, 성형, ZV 미적용)을 주고, 유발된 pitch 진동을 정밀 분석:
%%  - 모드 주파수 f0 (긴 꼬리 영교차 + 피크 간격, 0.01Hz급) -> ZVD 정밀화에도 사용
%%  - 감도 S = 진폭[deg] / 이동량[m]  (이 이동 프로파일 기준)
%%  - 위상: 이동 시작 시각 대비 진동 위상 오프셋 (상쇄 펄스 타이밍 계산용)
%% 검증: 20cm 펄스도 실행 - S의 선형성 확인 (능동 소거의 진폭 스케일링 전제)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 20; tStep = 3;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;

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

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; 'In Bus Element3','real_pitch'};
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

mws = get_param(mdl, 'ModelWorkspace');
set_param(mdl, 'StopTime', num2str(T));

amps = [0.10, 0.20];
fprintf('===== 스윙 교정 비행 (표준 펄스, ZV 미적용) =====\n');
res = {};
for ci = 1:numel(amps)
    A = amps(ci);
    % 펄스 길이는 저크-가능 조건으로 산정: 피크 저크 60A/Tm^3 <= 0.8*JMAX
    % (0.67s 고정으로 썼다가 10cm에서 저크 20 -> 스무더 뱅뱅 -> 가짜 8.8도 진동 사건, §W)
    Tm = max((60*A/(0.8*JMAX))^(1/3), 0.9);
    tau = min(max((tt-tStep)/Tm,0),1);
    xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
    [okG, repG] = traj_gate(tt, sm, VMAX, AMAX, false, JMAX);
    fprintf('  (Tm=%.2fs, 게이트 %s: a %.2f j %.1f, 스무더 개입 %.1fmm)\n', ...
        Tm, tern(okG,'통과','차단'), repG.axyPk, repG.jxyPk, ...
        max(abs(sm(:,1) - xk))*1000);
    if ~okG; error('교정 펄스가 게이트 불통과 - 펄스 설계 재확인'); end
    % 주의: waypoints는 시각화(Ground/Trajectory/Spline)+투하판정(비활성)용 -
    % 10cm처럼 짧으면 Spline 블록이 "distinct points 부족"으로 컴파일 거부. 가짜 1m 고정.
    waypoints = [0 0 1; 1 0 1]';
    mws.assignin('waypoints', waypoints);
    mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
    mws.assignin('timespot_spl', tt);
    mws.assignin('spline_data', sm);
    mws.assignin('spline_yaw', zeros(N,1));
    fprintf('\n--- 펄스 %gcm ---\n', A*100);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패 전문:\n%s\n', getReport(e, 'extended', 'hyperlinks', 'off'));
        continue;
    end
    tu = (0:0.002:T)';
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    % 이동 완료(대략 tStep+1.5s) 이후 꼬리에서 분석
    iT = tu >= tStep + 2 & tu <= T;
    y = pg(iT); ty = tu(iT);
    y = y - mean(y);
    % 주파수: 영교차 간격 평균 (긴 창 = 정밀)
    zc = find(abs(diff(sign(y)))>0);
    if numel(zc) >= 6
        perZC = 2 * mean(diff(ty(zc)));
        f0 = 1/perZC;
    else
        f0 = NaN;
    end
    % 진폭: 힐베르트 대신 피크 절대값 평균 (초반 3주기)
    iH = ty <= ty(1) + 3/max(f0, 1);
    yH = y(iH);
    pk = max(abs(yH));
    % 위상: 이동 시작(tStep) 기준, 꼬리 시작점에서의 cos 위상 역산
    % theta(t) = pk*cos(2*pi*f0*(t - tRef)) 라 할 때 첫 양피크 시각
    [~, iPk] = max(yH);
    tPk = ty(iPk);
    phase0 = mod(2*pi*f0*(tPk - tStep), 2*pi);   % 이동 시작 대비 첫 양피크 위상
    fprintf('  꼬리 진동: 진폭 %.2f도 | f0 = %.3fHz | 첫 양피크 t=%.3fs (이동시작+%.3fs, 위상 %.2frad)\n', ...
        pk, f0, tPk, tPk - tStep, phase0);
    fprintf('  감도 S = %.1f도/m (이 프로파일 기준)\n', pk / A);
    res(end+1,:) = {A, pk, f0, tPk - tStep, pk/A}; %#ok<SAGROW>
end

fprintf('\n===== 교정 결과 =====\n');
fprintf('%8s | %8s | %8s | %10s | %8s\n', '펄스[m]','진폭[도]','f0[Hz]','피크지연[s]','S[도/m]');
for ci = 1:size(res,1)
    fprintf('%8.2f | %8.2f | %8.3f | %10.3f | %8.1f\n', res{ci,:});
end
fprintf('(선형성: 두 S가 ~같으면 진폭 스케일링 성립. f0는 ZVD 정밀화에 그대로 사용)\n');

function s = tern(c, a, b)
    if c; s = a; else; s = b; end
end
