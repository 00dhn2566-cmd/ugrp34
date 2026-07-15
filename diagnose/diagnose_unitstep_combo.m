%% 진짜 unit step (순간 점프 1m) 대결: 현행 구운 상태 vs 오늘의 처방 3종 콤보
%% 콤보 = ①미분킥 처방 rate limiter(오차성형, ±0.3m/s) + ③z누설 처방 Sat Z(±0.15)
%%        + ⑤활공오차 처방 ki_attitude=-15 (중간값)
%% 미처방 잔존: ②측정지연(Filter Pitch) ④yaw 오염(포화 시) - 결과가 갈리는 지점.
%% 기대: 현행=발산 / 콤보=? (버티면 클램프 계열 방어선만으로 step 생존 첫 사례)
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
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

mc = [mdl '/Maneuver Controller'];
p = mc;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
pc = [mc '/Position Control'];

% --- 진짜 unit step 궤적: t=3에서 x 0 -> 1 순간 점프 ---
dt = 0.01; T = 15; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
xk = double(tt >= tStep) * A;
waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', [xk, zeros(N,1), ones(N,1)]);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

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

for ci = 1:2
    if ci == 1
        label = '현행 구운 상태 (처방 없음)';
    else
        label = '처방 3종 콤보 (RL0.3 + SatZ0.15 + ki_att-15)';
        % ① rate limiter 삽입 (Sat X/Y 직후 - clamp_rl과 동일 수술)
        muxB = [pc '/PosErr Mux'];
        muxPh = get_param(muxB, 'PortHandles');
        pairs = {[pc '/PosErr Sat X'], [pc '/PosErr RL X'], 1; [pc '/PosErr Sat Y'], [pc '/PosErr RL Y'], 2};
        for k = 1:2
            sph = get_param(pairs{k,1}, 'PortHandles');
            l = get_param(sph.Outport(1), 'Line');
            if l == -1; error('%s 출력 라인 없음', pairs{k,1}); end
            delete_line(l);
            add_block('simulink/Discontinuities/Rate Limiter', pairs{k,2}, ...
                'RisingSlewLimit', '0.3', 'FallingSlewLimit', '-0.3');
            rph = get_param(pairs{k,2}, 'PortHandles');
            add_line(pc, sph.Outport(1), rph.Inport(1), 'autorouting', 'on');
            add_line(pc, rph.Outport(1), muxPh.Inport(pairs{k,3}), 'autorouting', 'on');
        end
        % ③ Sat Z 삽입 (±0.15 - x/y와 동급)
        dmx = [pc '/PosErr Demux'];
        dph = get_param(dmx, 'PortHandles');
        l = get_param(dph.Outport(3), 'Line');
        if l == -1; error('PosErr Demux out3 라인 없음'); end
        delete_line(l);
        satZ = [pc '/PosErr Sat Z'];
        add_block('simulink/Discontinuities/Saturation', satZ, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
        zph = get_param(satZ, 'PortHandles');
        add_line(pc, dph.Outport(3), zph.Inport(1), 'autorouting', 'on');
        add_line(pc, zph.Outport(1), muxPh.Inport(3), 'autorouting', 'on');
        % ⑤ 자세 I (중간값)
        ki_attitude = -15;
        fprintf('\n처방 3종 삽입 완료\n');
    end
    fprintf('\n===== %s | 진짜 unit step 1m =====\n', label);
    try
        sim(mdl);
    catch e
        fprintf('  시뮬 실패: %s\n', e.message);
        continue;
    end
    tu = (0:0.005:T)';
    xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
    zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
    pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
    rg = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), tu, 'linear', 'extrap'));
    yw = rad2deg(interp1(real_yaw.time(:), real_yaw.signals.values(:), tu, 'linear', 'extrap'));
    wCeil = 1025;
    W = [abs(interp1(w1.time(:), w1.signals.values(:), tu, 'linear', 'extrap')), ...
         abs(interp1(w2.time(:), w2.signals.values(:), tu, 'linear', 'extrap')), ...
         abs(interp1(w3.time(:), w3.signals.values(:), tu, 'linear', 'extrap')), ...
         abs(interp1(w4.time(:), w4.signals.values(:), tu, 'linear', 'extrap'))] / wCeil * 100;
    for ct = [3.2 3.5 4 4.5 5 6 7 8 10 12 15]
        [~,i2] = min(abs(tu-ct));
        fprintf('  t=%4.1f | x %6.2f | P %+6.1f R %+6.1f Y %+6.1f | z %5.2f | 모터max %3.0f%%\n', ...
            tu(i2), xg(i2), pg(i2), rg(i2), yw(i2), zg(i2), max(W(i2,:)));
    end
    i3 = tu >= tStep;
    xMax = max(abs(xg));
    tSet = NaN;
    okm = abs(xg - A) < 0.05;
    ii0 = find(tu >= tStep, 1);
    for ii = ii0:numel(tu)
        if all(okm(ii:end)); tSet = tu(ii) - tStep; break; end
    end
    attPk = max(max(abs(pg(i3))), max(abs(rg(i3))));
    fprintf('  >> 최대|x| %.2fm / 정착(±5cm) %.2fs / 자세피크 %.1f도 / yaw피크 %.1f도 / z최저 %.2f / 모터피크 %.0f%% / %s\n', ...
        xMax, tSet, attPk, max(abs(yw(i3))), min(zg(i3)), max(W(i3&true,:),[],'all'), ...
        tern(xMax < 2 && ~isnan(tSet), '생존', '발산/미정착'));
end

function s = tern(c, a, b)
    if c; s = a; else; s = b; end
end
