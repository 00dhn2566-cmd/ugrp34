function info = qc_yawwrap_apply(mdl)
%QC_YAWWRAP_APPLY  yaw 오차를 ±pi 로 랩 (메모리 수술, save_system 금지).
%   2026-08-22 외란 강건화 세션. 사용자 지시: "yaw 도 360도 돌면 다시 업데이트".
%
% 문제: 구운 모델의 `Add3` 는 (ref − meas) 를 그대로 낸다. 랩이 없다.
%   → 기체가 한 바퀴 돌면 오차가 359도로 보이고, 제어기가 **먼 길로** 되돌리려 한다.
%   → anti-windup 이 없으므로(AntiWindupMode=none) 적분기가 그 359도를 계속 쌓는다.
%   실측(verify_yawdist_pulse): 0.3 N·m 3초에 yaw 오차 500도, 25초 내 미정착.
%
% C++ 이식본(`qc_controller.cpp`)은 이미 `wrapPi(in.refYaw - measY)` 로 랩한다.
% 즉 **Simulink 쪽이 C++ 과 다르게 동작하고 있었다** — 이 수술로 둘을 맞춘다.
%
% 배선: Add3 출력 → [Fcn wrapPi] → Control Yaw In1
%   wrapPi(a) = a − 2*pi*floor((a + pi)/(2*pi))     (Fcn 블록은 floor 지원)
%
% 항등성: |e| < 180도 이면 wrapPi(e) == e 라 **정상 임무에서는 완전히 항등**이다.
%   달라지는 것은 한 바퀴를 넘긴 비정상 구간뿐이고, 거기서는 랩한 쪽이 옳다.
%
% 사용: load_system(mdl) 후, sim() 전. 이미 배선돼 있으면 아무것도 안 한다.

info = struct();

blks = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Control Yaw');
if numel(blks) ~= 1
    error('qc_yawwrap_apply: Control Yaw %d개 (1개 예상)', numel(blks));
end
pid = blks{1};
parent = get_param(pid, 'Parent');
info.pid = pid;  info.parent = parent;

if ~isempty(find_system(parent, 'SearchDepth', 1, 'Name', 'QC Yaw Wrap'))
    info.skipped = true;
    fprintf('[qc_yawwrap_apply] 이미 배선됨 - 통과\n');
    return;
end
info.skipped = false;

% 라이브러리 링크 해제 (상위 체인)
p = parent;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end

ph = get_param(pid, 'PortHandles');
ln = get_param(ph.Inport(1), 'Line');
if ln == -1
    error('qc_yawwrap_apply: Control Yaw In1 에 연결된 선이 없음');
end
src = get_param(ln, 'SrcPortHandle');
srcBlk = get_param(src, 'Parent');
delete_line(ln);

pos = get_param(pid, 'Position');
fcn = [parent '/QC Yaw Wrap'];
add_block('simulink/User-Defined Functions/Fcn', fcn, ...
    'Expr', 'u-2*pi*floor((u+pi)/(2*pi))', ...
    'Position', [pos(1)-110 pos(2) pos(1)-50 pos(2)+30]);
fph = get_param(fcn, 'PortHandles');
add_line(parent, src, fph.Inport(1), 'autorouting','on');
add_line(parent, fph.Outport(1), ph.Inport(1), 'autorouting','on');

fprintf('[qc_yawwrap_apply] yaw 오차 ±pi 랩 삽입 (%s -> QC Yaw Wrap -> Control Yaw)\n', ...
        strtrim(regexprep(get_param(srcBlk,'Name'), '\s+', ' ')));
end
