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
qc_mhe_apply(mdl);

% 탭한 Goto 신호를 To Workspace 로 받는다 (From 을 하나 더 달아서).
pc = get_param(find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
      'Name', 'MHE'), 'Parent');
pc = pc{1};
tapNames = {'qcMheRoll', 'tap_roll'; 'qcMhePitch', 'tap_pitch'; 'qcMheYaw', 'tap_yaw'};
for k = 1:3
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
sp = get_param(hm{1}, 'PortHandles');
lnp = get_param(sp.Inport(1), 'Line');
srcp = get_param(lnp, 'SrcPortHandle');
twp = [pc '/TAPW pos'];
if isempty(find_system(pc, 'SearchDepth',1, 'Name', 'TAPW pos'))
    add_block('simulink/Sinks/To Workspace', twp, 'VariableName', 'tap_pos', ...
        'SaveFormat', 'StructureWithTime', 'Position', [130 560 190 580]);
    add_line(pc, srcp, get_param(twp,'PortHandles').Inport(1), 'autorouting','on');
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
fprintf('\n  --- 위치 벡터 순서 ---\n');
pp = {'입력1 vs real_x', px, rx; '입력2 vs real_y', py, ry; '입력3 vs real_z', pz, rz; ...
      '입력1 vs real_y (교차)', px, ry; '입력2 vs real_x (교차)', py, rx};
for k = 1:size(pp,1)
    fprintf('  %-26s %+8.3f\n', pp{k,1}, cc(pp{k,2}, pp{k,3}));
end
fprintf('  진폭 [deg] tap_roll %.2f / tap_pitch %.2f / real_roll %.2f / real_pitch %.2f\n', ...
        rad2deg(max(abs(tr))), rad2deg(max(abs(tp))), ...
        rad2deg(max(abs(rr))), rad2deg(max(abs(rp))));
