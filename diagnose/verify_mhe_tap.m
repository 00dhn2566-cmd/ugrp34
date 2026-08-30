%% MHE 자세 탭이 진짜 roll/pitch 인가 (2026-08-29, 사용자 지적)
%%
%% qc_mhe_apply 는 'Filter Roll' / 'Filter Pitch' 라는 **블록 이름**을 믿고 자세를
%% 딴다. 그런데 이 저장소는 pitch/roll 혼동 이력이 있고(diagnose_pitch_vs_roll_logic,
%% 판 뒤집힘 사건), qc_delay_apply 가 가리키는 In Bus Element 번호와 log_signals 가
%% 쓰는 번호는 **서로 다른 서브시스템**이라 이름으로 대조가 안 된다.
%%
%% 그래서 실측한다: 탭한 신호를 To Workspace 로 받아 real_roll / real_pitch 와
%% 상관을 낸다. 뒤바뀜과 부호를 한 번에 가른다.
%%   기대: tap_roll  vs real_roll   상관 ~ +1
%%         tap_pitch vs real_pitch  상관 ~ +1
%%   뒤바뀌었으면 교차 상관이 크고, 부호가 뒤집혔으면 -1 근처가 나온다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'), fullfile(modelDir, 'Models'), ...
        fullfile(modelDir, 'Libraries'), fullfile(modelDir, 'diagnose'), ...
        genpath(fullfile(modelDir, 'CAD')));
quadcopter_package_parameters;

mdl = 'quadcopter_package_delivery';
if bdIsLoaded(mdl); close_system(mdl, 0); end
load_system(mdl);
qctest.disable_drop(mdl);
qc_0kg_tuned_apply(mdl);
assignin('base', 'dly_att_s', 0.005);
assignin('base', 'dly_pos_s', 0.060);
qctest.raised_cos_move(mdl, 8.0, 3.0, 1.0, 3.0, 3.927);
qctest.log_signals(mdl, 'all');
qc_delay_apply(mdl);
% MHE_OFF=1 이면 MHE 를 안 끼운다 — **대조군**.
% 60 ms 는 §1.3 에서 '감쇄로 못 고치는 손상' 이 나오는 구간이라(고도 한계사이클,
% yaw 표류 23.8도), 지연만으로도 상시 진동이 생긴다. 스윙이 MHE 탓인지 지연 탓인지는
% 이 대조군 없이는 못 가른다.
if ~strcmp(getenv('MHE_OFF'), '1')
    qc_mhe_apply(mdl);
end

% 탭한 Goto 신호를 To Workspace 로 받는다 (From 을 하나 더 달아서).
% MHE 가 없을 수도 있으므로(대조군) 'Meas Delay Pos' 의 부모로 잡는다 —
% 둘 다 Position Control 안에 있어 같은 부모다.
hmd = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Meas Delay Pos');
pc = get_param(hmd{1}, 'Parent');
tapNames = {'qcMheRoll', 'tap_roll'; 'qcMhePitch', 'tap_pitch'; 'qcMheYaw', 'tap_yaw'};
if strcmp(getenv('MHE_OFF'), '1')
    tapNames = {};        % Goto 가 없다 — 자세 대조는 건너뛰고 스윙만 본다
end
for k = 1:size(tapNames,1)
    fb = [pc '/TAP ' tapNames{k,1}];
    tw = [pc '/TAPW ' tapNames{k,1}];
    if isempty(find_system(pc, 'SearchDepth',1, 'Name', ['TAP ' tapNames{k,1}]))
        add_block('simulink/Signal Routing/From', fb, 'GotoTag', tapNames{k,1}, ...
            'Position', [30 400+40*k 90 420+40*k]);
        add_block('simulink/Sinks/To Workspace', tw, 'VariableName', tapNames{k,2}, ...
            'SaveFormat', 'StructureWithTime', 'Position', [130 400+40*k 190 420+40*k]);
        add_line(pc, get_param(fb,'PortHandles').Outport(1), ...
                     get_param(tw,'PortHandles').Inport(1), 'autorouting','on');
    end
end

% ★ 두 번째 미확인 가정: Subtract2 입력 2 가 [x y z] 순서인가.
%   S-function 은 1번 축에 a_x, 2번 축에 a_y 를 배정한다. 순서가 다르면 x 추정이
%   y 의 가속을 먹어 전진 중 y 가 흔들린다 — 자세 뒤바뀜과 증상이 같다.
hm = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'MHE');
if isempty(hm)
    sp = [];          % 대조군(MHE 없음) — 위치·출력 탭은 건너뛴다
else
    sp = get_param(hm{1}, 'PortHandles');
end
if ~isempty(sp)
lnp = get_param(sp.Inport(1), 'Line');
srcp = get_param(lnp, 'SrcPortHandle');
twp = [pc '/TAPW pos'];
if isempty(find_system(pc, 'SearchDepth',1, 'Name', 'TAPW pos'))
    add_block('simulink/Sinks/To Workspace', twp, 'VariableName', 'tap_pos', ...
        'SaveFormat', 'StructureWithTime', 'Position', [130 560 190 580]);
    add_line(pc, srcp, get_param(twp,'PortHandles').Inport(1), 'autorouting','on');
end
end

% ★ MHE **출력**도 받는다. 계단 가설을 판정하려면 이 신호가 필요하다:
%   무거운 해가 N 스텝마다 p0,v0,b 를 통째로 갈아끼우므로 출력이 불연속으로 튈 수 있고,
%   위치 제어기의 D 항이 그걸 미분하면 큰 스파이크가 된다. 파이썬에서는 RMS 오차로만
%   평가해 이걸 못 봤다 — RMS 는 '정확하지만 튀는' 신호를 잘 나온 것으로 친다.
if ~isempty(tapNames)
    two = [pc '/TAPW out'];
    if isempty(find_system(pc, 'SearchDepth',1, 'Name', 'TAPW out'))
        add_block('simulink/Sinks/To Workspace', two, 'VariableName', 'tap_out', ...
            'SaveFormat', 'StructureWithTime', 'Position', [300 560 360 580]);
        add_line(pc, sp.Outport(1), get_param(two,'PortHandles').Inport(1), 'autorouting','on');
    end
end

set_param(mdl, 'StopTime', '8.0');
sim(mdl);   % 결과는 작업공간 변수로 들어온다 (verify_worstcase 와 같은 규약)
close_system(mdl, 0);

t  = linspace(1.0, 7.5, 3000);
% To Workspace 변수는 base 로 들어오므로 evalin 으로 꺼낸다.
% To Workspace 변수는 base 로 들어온다. MATLAB 은 함수 호출 결과에 바로 인덱싱을
% 못 하므로(find_system(...){1} 이 같은 이유로 죽었다) 먼저 꺼내 놓는다.
getsig = @(n) evalin('base', n);
rs = @(v) interp1(v.time(:), v.signals.values(:), t, 'linear', 'extrap');
gv = @(n) rs(getsig(n));
tr = gv('tap_roll');  tp = gv('tap_pitch');  ty = gv('tap_yaw');
rr = gv('real_roll'); rp = gv('real_pitch'); ry = gv('real_yaw');

cc = @(a,b) sum((a-mean(a)).*(b-mean(b))) / sqrt(sum((a-mean(a)).^2)*sum((b-mean(b)).^2));
% ── 스윙 지표는 MHE 유무와 무관하게 항상 낸다 (범인 특정용)
rr0 = gv('real_roll'); rp0 = gv('real_pitch'); ry0 = gv('real_y');
fprintf('\n===== 스윙 지표 (직진 3 m, 외란 없음, 위치 지연 60 ms) =====\n');
if strcmp(getenv('MHE_OFF'), '1')
    fprintf('  구성: MHE **끔** (대조군)\n');
else
    fprintf('  구성: MHE 켬\n');
end
fprintf('  roll  최대 %.2f deg / RMS %.2f deg\n', rad2deg(max(abs(rr0))), rad2deg(sqrt(mean(rr0.^2))));
fprintf('  pitch 최대 %.2f deg / RMS %.2f deg\n', rad2deg(max(abs(rp0))), rad2deg(sqrt(mean(rp0.^2))));
fprintf('  y 이탈 최대 %.2f cm / RMS %.2f cm\n', 100*max(abs(ry0)), 100*sqrt(mean(ry0.^2)));

if isempty(tapNames)
    return;    % MHE 없으면 탭 대조는 할 게 없다
end

fprintf('\n===== MHE 자세 탭 대조 =====\n');
fprintf('  %-22s %8s %10s\n', '쌍', '상관', '기울기');
pairs = {'tap_roll  vs real_roll', tr, rr; 'tap_pitch vs real_pitch', tp, rp; ...
         'tap_yaw   vs real_yaw',  ty, ry; ...
         'tap_roll  vs real_pitch (교차)', tr, rp; ...
         'tap_pitch vs real_roll  (교차)', tp, rr};
for k = 1:size(pairs,1)
    a = pairs{k,2}; b = pairs{k,3};
    sl = sum((a-mean(a)).*(b-mean(b))) / max(sum((b-mean(b)).^2), eps);
    fprintf('  %-30s %+8.3f %+10.3f\n', pairs{k,1}, cc(a,b), sl);
end
% 위치 순서 대조
tpv = evalin('base','tap_pos');
gp = @(k) interp1(tpv.time(:), tpv.signals.values(:,k), t, 'linear', 'extrap');
px = gp(1); py = gp(2); pz = gp(3);
rx = gv('real_x'); ry = gv('real_y'); rz = gv('real_z');
% 계단 지표: 출력의 1스텝 차분이 입력(부드러운 지연 신호)보다 얼마나 거친가.
tov = evalin('base','tap_out');
ox = interp1(tov.time(:), tov.signals.values(:,1), t, 'linear', 'extrap');
oy = interp1(tov.time(:), tov.signals.values(:,2), t, 'linear', 'extrap');
jump = @(v) max(abs(diff(v)));
rms_ = @(v) sqrt(mean(v.^2));
fprintf('\n  --- 계단 지표 (1스텝 차분, cm) ---\n');
fprintf('  입력 x  최대 %7.4f  RMS %7.4f\n', 100*jump(px), 100*rms_(diff(px)));
fprintf('  출력 x  최대 %7.4f  RMS %7.4f\n', 100*jump(ox), 100*rms_(diff(ox)));
fprintf('  입력 y  최대 %7.4f  RMS %7.4f\n', 100*jump(py), 100*rms_(diff(py)));
fprintf('  출력 y  최대 %7.4f  RMS %7.4f\n', 100*jump(oy), 100*rms_(diff(oy)));
fprintf('  (출력이 입력보다 훨씬 거칠면 계단 가설 확정)\n');

fprintf('\n  --- 위치 벡터 순서 ---\n');
pp = {'입력1 vs real_x', px, rx; '입력2 vs real_y', py, ry; '입력3 vs real_z', pz, rz; ...
      '입력1 vs real_y (교차)', px, ry; '입력2 vs real_x (교차)', py, rx};
for k = 1:size(pp,1)
    fprintf('  %-26s %+8.3f\n', pp{k,1}, cc(pp{k,2}, pp{k,3}));
end
fprintf('  진폭 [deg] tap_roll %.2f / tap_pitch %.2f / real_roll %.2f / real_pitch %.2f\n', ...
        rad2deg(max(abs(tr))), rad2deg(max(abs(tp))), ...
        rad2deg(max(abs(rr))), rad2deg(max(abs(rp))));
