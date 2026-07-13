%% 모터 2,3이 실제로 음의 방향으로 도는지 확인 (반대회전 실현 여부 검증):
%% 믹서 반전으로 모터 2,3은 음수 ref를 받아야 하는데, combined 테스트에서
%% W1~W4가 전부 +1076으로 나왔음 -> 반대회전이 물리적으로 안 되고 있을 의심.
%% W1~W4(부호 포함)와 믹서 Add5/Add7 출력(ref)을 같이 로깅.
%% 기반 조건은 thrust_path_rescale과 동일:
%% 모터 ref = 2pi x (고도PID + Bias Chassis + (load+1) x BiasLoadGain x 패키지질량)
%% - Bias Chassis: 700 -> 56.5 (기체분: 101 rev/s x 1.2726/2.2726)
%% - Bias Load 상수: 260 -> 44.4 (패키지분: 101 rev/s x 1/2.2726 / 1kg)
%% - 고도 PID 출력(cmd)에 ±30 rev/s 클램프 삽입 (992까지 폭주하던 것 차단)
%% + 기존 확정 수정: Kthrust=9.79, Kdrag=0.597, 프롭2,3+믹서 반전, x,y sat ±0.15m

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

quadcopter_package_parameters;

kp_attitude = 5;    ki_attitude = 0;    kd_attitude = 2;
kp_yaw      = 3;    ki_yaw = 0;         kd_yaw = 1;
kp_altitude = 0.5;  ki_altitude = 0.1;  kd_altitude = 0.3;
kp_position = 8;    ki_position = 0.04; kd_position = 3.2;

mdl = 'quadcopter_package_delivery';
load_system(mdl);

% --- 프로펠러 물리 보정 ---
propeller.Kthrust = 9.79;
propeller.Kdrag   = 0.597;
assignin('base', 'propeller', propeller);

% --- 추력 경로 재스케일 ---
sub = [mdl '/Maneuver Controller/Altitude and  YPR Control/Subsystem'];
set_param([sub '/Bias Chassis'], 'Bias', '56.5');
set_param([sub '/Bias Load'], 'Gain', '44.4*pkgSize(1)^3*pkgDensity');

% 고도 PID 출력(cmd 인포트 신호)에 ±30 클램프 삽입
cmdBlk = [sub '/cmd'];
bcBlk = [sub '/Bias Chassis'];
cph = get_param(cmdBlk, 'PortHandles');
bph = get_param(bcBlk, 'PortHandles');
oldLine = get_param(cph.Outport(1), 'Line');
if oldLine ~= -1; delete_line(oldLine); end
satBlk = [sub '/Alt Cmd Sat'];
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Alt Cmd Sat'))
    add_block('simulink/Discontinuities/Saturation', satBlk, 'UpperLimit', '30', 'LowerLimit', '-30');
end
sph = get_param(satBlk, 'PortHandles');
add_line(sub, cph.Outport(1), sph.Inport(1), 'autorouting', 'on');
add_line(sub, sph.Outport(1), bph.Inport(1), 'autorouting', 'on');

% --- 프로펠러 2,3 + 믹서 반전 ---
% 프로펠러 direction 오버라이드 제거 - 공장 상태(전부 Positive) 그대로 사용.
% 공장 설계: 모터 2,3 하류 내장반전(음의 회전) + Thrust Direction 게인(-1)이
% 이미 반대회전+추력방향을 완결적으로 처리하고 있음.
for p = 1:4
    blk = sprintf('%s/Quadcopter/Propeller %d/Thrust and Drag/Aerodynamic Propeller', mdl, p);
    fprintf('Propeller %d direction(공장값 유지) = %s\n', p, get_param(blk, 'direction'));
end
% --- 확정 실험: 프롭 4개의 External Force에서 토크(TorqueY) 적용 OFF ---
% 반작용 토크가 roll축(수평)으로 잘못 가해지고 있다는 가설 검증.
for pp = 1:4
    efList = find_system(sprintf('%s/Quadcopter/Propeller %d', mdl, pp), ...
        'LookUnderMasks','all','FollowLinks','on','RegExp','on','Name','External Force');
    for k = 1:numel(efList)
        try
            set_param(efList{k}, 'EnableTorqueY', 'off');
            fprintf('P%d %s: EnableTorqueY -> off\n', pp, strrep(efList{k}, sprintf('%s/Quadcopter/Propeller %d/', mdl, pp), ''));
        catch e2
            fprintf('P%d 실패: %s\n', pp, e2.message(1:min(100,end)));
        end
    end
end

mixer = [mdl '/Maneuver Controller/Motor Mixer'];
% 믹서 반전 제거: 원래(디스크) 부호 그대로 사용.
% 근거: 모터가 ref의 절대값을 추종함이 확인됨(부호 무시) -> 원래 부호면
% 차등 성분들이 올바른 기하학적 부호로 크기에 반영됨.
fprintf('Mixer 원래 부호 유지: Add4=%s Add5=%s Add6=%s Add7=%s\n', ...
    get_param([mixer '/Add4'],'Inputs'), get_param([mixer '/Add5'],'Inputs'), ...
    get_param([mixer '/Add6'],'Inputs'), get_param([mixer '/Add7'],'Inputs'));

% --- x,y만 ±0.15m saturation ---
pc = [mdl '/Maneuver Controller/Position Control'];
subBlk = [pc '/Subtract2'];
pidBlk = [pc '/PID Controller'];
subPh = get_param(subBlk, 'PortHandles');
pidPh = get_param(pidBlk, 'PortHandles');
oldLine = get_param(subPh.Outport(1), 'Line');
if oldLine ~= -1; delete_line(oldLine); end
demuxBlk = [pc '/PosErr Demux'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Demux'))
    add_block('simulink/Signal Routing/Demux', demuxBlk, 'Outputs', '3');
end
satXBlk = [pc '/PosErr Sat X'];
satYBlk = [pc '/PosErr Sat Y'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat X'))
    add_block('simulink/Discontinuities/Saturation', satXBlk, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
end
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat Y'))
    add_block('simulink/Discontinuities/Saturation', satYBlk, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
end
muxBlk = [pc '/PosErr Mux'];
if isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Mux'))
    add_block('simulink/Signal Routing/Mux', muxBlk, 'Inputs', '3');
end
demuxPh = get_param(demuxBlk, 'PortHandles');
satXPh = get_param(satXBlk, 'PortHandles');
satYPh = get_param(satYBlk, 'PortHandles');
muxPh = get_param(muxBlk, 'PortHandles');
add_line(pc, subPh.Outport(1), demuxPh.Inport(1), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(1), satXPh.Inport(1), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(2), satYPh.Inport(1), 'autorouting', 'on');
add_line(pc, satXPh.Outport(1), muxPh.Inport(1), 'autorouting', 'on');
add_line(pc, satYPh.Outport(1), muxPh.Inport(2), 'autorouting', 'on');
add_line(pc, demuxPh.Outport(3), muxPh.Inport(3), 'autorouting', 'on');
add_line(pc, muxPh.Outport(1), pidPh.Inport(1), 'autorouting', 'on');
% Scope 분기 재연결
scopeBlk = [pc '/Scope'];
scopePh = get_param(scopeBlk, 'PortHandles');
if get_param(scopePh.Inport(2), 'Line') == -1
    add_line(pc, subPh.Outport(1), scopePh.Inport(2), 'autorouting', 'on');
end

dt = 0.01;
T = 0.6;
N = round(T/dt) + 1;
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

scope = [mdl '/Scope'];
sigMap = {'In Bus Element2','real_z'; 'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element21','cmd_roll'; 'In Bus Element22','cmd_pitch'; ...
          'In Bus Element11','W1'; 'In Bus Element10','W2'; 'In Bus Element12','W3'; 'In Bus Element13','W4'; ...
          'In Bus Element6','T1'; 'In Bus Element7','T2'; 'In Bus Element8','T3'; 'In Bus Element9','T4'};
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

% 믹서 Add4~Add7 출력(모터별 ref) 태핑
for aPair = {{'Add5','ref_w2'},{'Add7','ref_w3'},{'Add4','ref_w1'},{'Add6','ref_w4'}}
    aName = aPair{1}{1}; vName = aPair{1}{2};
    aph = get_param([mixer '/' aName], 'PortHandles');
    twName = ['To Workspace ' vName];
    if isempty(find_system(mixer, 'SearchDepth', 1, 'Name', twName))
        twBlk2 = [mixer '/' twName];
        add_block('simulink/Sinks/To Workspace', twBlk2, 'VariableName', vName, 'SaveFormat', 'StructureWithTime');
        twPh2 = get_param(twBlk2, 'PortHandles');
        add_line(mixer, aph.Outport(1), twPh2.Inport(1), 'autorouting', 'on');
    end
end

fprintf('=== TorqueY OFF 확정실험: 반작용 토크 적용을 끄면 발산이 사라지는가 ===\n');
try
    simOut = sim(mdl);
    t = real_z.time(:);
    z = real_z.signals.values(:);
    r = rad2deg(real_roll.signals.values(:));
    rrad = real_roll.signals.values(:);
    t1 = T1.signals.values(:); t2 = T2.signals.values(:); t3 = T3.signals.values(:); t4 = T4.signals.values(:);

    % 믹서 roll 채널 부호: w1 +, w2 -, w3 +, w4 - (기하: (1,3) vs (2,4) 쌍)
    % roll 모멘트 추정 = ((T1+T3)-(T2+T4)) x 유효 팔길이(대각 0.2m x sin45 = 0.14m)
    Larm = 0.14;
    dT = (t1+t3) - (t2+t4);
    Mthr = dT * Larm;

    fprintf('\n  %5s | %7s %7s %7s %7s | %8s %9s | %8s\n', 't','T1','T2','T3','T4','dT_roll','M_thrust','roll(deg)');
    for ct = 0:0.03:0.6
        [~, idx] = min(abs(t - ct));
        fprintf('  %5.2f | %7.3f %7.3f %7.3f %7.3f | %+8.3f %+9.4f | %8.2f\n', ...
            t(idx), t1(idx), t2(idx), t3(idx), t4(idx), dT(idx), Mthr(idx), r(idx));
    end

    % 실측 roll 각가속도에서 역산한 필요 모멘트 (I_roll ~ 0.03 kg m^2 가정)
    dt_s = mean(diff(t));
    rollRate = gradient(rrad, t);
    rollAcc = gradient(rollRate, t);
    Ireq = 0.03;
    fprintf('\n=== 필요 모멘트(=I*alpha, I=0.03 가정) vs 추력 모멘트 ===\n');
    fprintf('  %5s | %9s | %9s | %s\n', 't', 'M_needed', 'M_thrust', '해석');
    for ct = [0.05 0.1 0.15 0.2 0.25 0.3 0.4]
        [~, idx] = min(abs(t - ct));
        mn_ = Ireq*rollAcc(idx); mt_ = Mthr(idx);
        if abs(mt_) > 0.5*abs(mn_) && sign(mt_)==sign(mn_)
            verdict = '추력차가 주 원인';
        else
            verdict = '추력차로 설명 불가(다른 토크원)';
        end
        fprintf('  %5.2f | %+9.4f | %+9.4f | %s\n', t(idx), mn_, mt_, verdict);
    end
catch e
    fprintf('  *** 실패: %s\n', e.message(1:min(300,end)));
end
