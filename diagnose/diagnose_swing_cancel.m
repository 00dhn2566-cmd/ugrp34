%% 능동 스윙 소거 2단계: 반위상 공진 펌핑 실증 (판단 로직 없이 상쇄 물리부터)
%% 1m/0.67s 성형 이동(잔류 진동 ~4.3도 유발) 후 t0=7s부터 기준에 미소 공진 흔들기
%% (A=7mm, f0=1.80Hz, 6주기, Tukey 창) 주입. 위상 4종 스윕:
%%   맞는 위상 -> 그네 반대 펌핑 = 진동 에너지 제거 / 반대 위상 -> 증폭 (통제 실험)
%% A=7mm 근거: 공진 정현파의 저크 A(2*pi*f)^3 <= JMAX -> A <= 6.9mm (게이트 준수).
%% 판정: 흔들기 종료 후(11~16s) pitch RMS vs 기준(무주입) 비교.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
F0 = 1.80; AW = 0.007; NCYC = 6; T0W = 7.0;
dt = 0.01; T = 16; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smBase = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

% 흔들기 파형 (Tukey 창 램프 1주기)
wigDur = NCYC / F0;
mkWig = @(ph) AW * cos(2*pi*F0*(tt - T0W) + ph) .* ...
    max(0, min(1, min((tt - T0W)*F0, (T0W + wigDur - tt)*F0))) .* ...
    (tt >= T0W & tt <= T0W + wigDur);

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
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

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

phases = [NaN, 0, pi/2, pi, 3*pi/2];   % NaN = 무주입 기준
names = {'기준(무주입)', '위상 0', '위상 90도', '위상 180도', '위상 270도'};
fprintf('===== 반위상 공진 펌핑 (A=%.0fmm, f0=%.2fHz, %d주기, t0=%.1fs) =====\n', AW*1000, F0, NCYC, T0W);
summ = {};
for ci = 1:numel(phases)
    sm = smBase;
    if ~isnan(phases(ci))
        sm(:,1) = sm(:,1) + mkWig(phases(ci));
    end
    mws.assignin('spline_data', sm);
    fprintf('\n--- %s ---\n', names{ci});
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        summ(end+1,:) = {names{ci}, NaN, NaN}; %#ok<SAGROW>
        continue;
    end
    tu = (0:0.002:T)';
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    rmsf = @(m) sqrt(mean((pg(m)-mean(pg(m))).^2));
    rPre  = rmsf(tu >= 5 & tu < 7);            % 주입 전
    rPost = rmsf(tu >= T0W + wigDur + 0.6 & tu <= T);  % 주입 후
    fprintf('  pitch RMS: 주입 전(5~7s) %.2f도 -> 주입 후(%.1f~%ds) %.2f도 (비 %.2f)\n', ...
        rPre, T0W+wigDur+0.6, T, rPost, rPost/rPre);
    summ(end+1,:) = {names{ci}, rPre, rPost}; %#ok<SAGROW>
end

fprintf('\n===== 요약 =====\n');
fprintf('%-14s | %10s | %10s | %6s\n', '케이스','전RMS[도]','후RMS[도]','후/전');
for ci = 1:size(summ,1)
    fprintf('%-14s | %10.2f | %10.2f | %6.2f\n', summ{ci,1}, summ{ci,2}, summ{ci,3}, summ{ci,3}/summ{ci,2});
end
fprintf('(최적 위상에서 후/전 << 기준, 반대 위상에서 > 기준이면 = 공진 펌핑 제어권 입증)\n');
