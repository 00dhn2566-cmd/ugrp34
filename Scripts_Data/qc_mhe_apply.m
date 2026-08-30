function qc_mhe_apply(mdl)
%QC_MHE_APPLY  위치 측정 경로에 지연 보상 MHE 를 끼운다 (메모리 수술, save 금지).
%
%   배선
%     Meas Delay Pos ─────────────────────┐
%                                          ├─> [MHE S-Function] ─> Subtract2 입력 2
%     Filter Roll/Pitch/Yaw 입력 ─Goto─From┘
%
%   ★ 자세가 다른 서브시스템(Altitude and YPR Control)에 있어 선을 못 끈다.
%     Goto/From 으로 건너온다. 태그 가시성은 global — 두 서브시스템의 공통 조상이
%     모델뿐이기 때문이다.
%
%   ★ 자세는 **필터 입력**에서 딴다. 거기가 지연 블록 뒤라 제어기가 실제로 보는
%     값이고, 진짜 자세(real_roll 등)를 쓰면 시뮬 특권이 되어 실기에서 안 재현된다.
%
%   지연이 0 이면 아무것도 안 한다 — 보상할 게 없는데 센서를 틀렸다고 가정하는
%   위험만 진다 (08-29: tau <' 30 ms 는 배율 1.00 구간이라 값어치를 못 한다).

tau = evalin('base', 'dly_pos_s');
if ~isscalar(tau) || tau <= 0
    fprintf('[qc_mhe_apply] dly_pos_s = 0 -> 보상할 지연이 없어 건너뜀\n');
    return;
end
qc_mhe_defaults();

% ── 1) 자세 3채널에 Goto 를 단다 (필터 입력 선에서 분기)
tags = {'Filter Roll', 'qcMheRoll'; 'Filter Pitch', 'qcMhePitch'; 'Filter Yaw', 'qcMheYaw'};
for k = 1:size(tags,1)
    hits = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', tags{k,1});
    if numel(hits) ~= 1
        error('qc_mhe_apply: %s 블록 %d개 (1개 예상)', tags{k,1}, numel(hits));
    end
    dst = hits{1}; parent = get_param(dst, 'Parent');
    unlink_chain(mdl, parent);
    if ~isempty(find_system(parent, 'SearchDepth',1, 'Name', tags{k,2}))
        continue;
    end
    ph = get_param(dst, 'PortHandles');
    ln = get_param(ph.Inport(1), 'Line');
    if ln == -1
        error('qc_mhe_apply: %s 입력에 선이 없음', tags{k,1});
    end
    src = get_param(ln, 'SrcPortHandle');
    pos = get_param(dst, 'Position');
    gname = [parent '/' tags{k,2}];
    add_block('simulink/Signal Routing/Goto', gname, 'GotoTag', tags{k,2}, ...
        'TagVisibility', 'global', ...
        'Position', [pos(1)-70 pos(2)+40*k pos(1)-20 pos(2)+40*k+20]);
    gph = get_param(gname, 'PortHandles');
    add_line(parent, src, gph.Inport(1), 'autorouting', 'on');
end

% ── 2) 위치 경로에 S-Function 을 끼운다
hits = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Meas Delay Pos');
if isempty(hits)
    error('qc_mhe_apply: Meas Delay Pos 없음 — qc_delay_apply 를 먼저 부를 것');
end
dly = hits{1}; parent = get_param(dly, 'Parent');
if ~isempty(find_system(parent, 'SearchDepth',1, 'Name', 'MHE'))
    fprintf('[qc_mhe_apply] 이미 삽입됨\n');
    return;
end
unlink_chain(mdl, parent);

ph = get_param(dly, 'PortHandles');
ln = get_param(ph.Outport(1), 'Line');
if ln == -1
    error('qc_mhe_apply: Meas Delay Pos 출력에 선이 없음');
end
dstPorts = get_param(ln, 'DstPortHandle');
delete_line(ln);

pos = get_param(dly, 'Position');
froms = {'qcMheRoll', 'qcMhePitch', 'qcMheYaw'};
mux = [parent '/MHE Att Mux'];
add_block('simulink/Signal Routing/Mux', mux, 'Inputs', '3', ...
    'Position', [pos(1)+90 pos(2)+60 pos(1)+95 pos(2)+140]);
mph = get_param(mux, 'PortHandles');
for k = 1:3
    fb = [parent '/MHE From ' froms{k}];
    add_block('simulink/Signal Routing/From', fb, 'GotoTag', froms{k}, ...
        'Position', [pos(1)+10 pos(2)+60+28*(k-1) pos(1)+70 pos(2)+80+28*(k-1)]);
    fph = get_param(fb, 'PortHandles');
    add_line(parent, fph.Outport(1), mph.Inport(k), 'autorouting', 'on');
end

sfb = [parent '/MHE'];
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', sfb, ...
    'FunctionName', 'qc_mhe_sfun', ...
    'Position', [pos(1)+140 pos(2)-10 pos(1)+220 pos(2)+90]);
sph = get_param(sfb, 'PortHandles');
add_line(parent, ph.Outport(1), sph.Inport(1), 'autorouting', 'on');
add_line(parent, mph.Outport(1), sph.Inport(2), 'autorouting', 'on');
for d = 1:numel(dstPorts)
    add_line(parent, sph.Outport(1), dstPorts(d), 'autorouting', 'on');
end

H = evalin('base', 'mhe_horizon_s');
AT = evalin('base', 'mhe_age_trust');
fprintf('[qc_mhe_apply] MHE 삽입: tau %.1f ms, 창 %.0f ms, age_trust %.2f\n', ...
        1000*tau, 1000*H, AT);
end

function unlink_chain(mdl, blk)
% 라이브러리 링크를 위로 훑어 해제한다 (링크된 블록은 수정이 안 된다).
p = blk;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved', 'inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
end
