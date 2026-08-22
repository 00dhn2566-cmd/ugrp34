function info = qc_yawdist_apply(mdl)
%QC_YAWDIST_APPLY  yaw 외란 적응 적분 배선 (메모리 수술, save_system 금지).
%   2026-08-22 외란 강건화 세션. 설계 근거: control_seoungjin/docs/YAW_DISTURBANCE_I.md
%
% 하는 일: 'Control Yaw' PID 블록을 외부 파라미터 모드로 바꾸고, I 포트에만
%   시변 게인을 먹인다. P/D/N 은 원래 값(마스크 변수)을 상수로 그대로 물린다.
%
%   I(t) = yaw_ki * (1 + (yd_gmax-1) * k(t))
%   k_raw = [ |psi_ref_dot| <= yd_rate ] * sat01( (|e_lpf| - yd_e0) / yd_e1 )
%   k     = RateLimiter(k_raw)   상승 무제한 / 하강 -yd_relax  (경계 채터링 방지)
%   e_lpf = LPF(e, yd_tau),  psi_ref_dot = s/(yd_taud s + 1) * psi_ref
%
%   yd_gmax = 1 이면 I(t) = yaw_ki 상수 -> 구운 모델과 완전히 같은 식 (항등).
%   PID 블록이 Parallel/Forward Euler 라 I 는 적분 '전'에 곱해진다. 즉 게인을
%   바꿔도 이미 쌓인 적분값은 재스케일되지 않는다 (C++ Pid::kiScale 과 같은 의미).
%
% 필요한 base 워크스페이스 변수: yd_gmax yd_e0 yd_e1 yd_tau yd_rate yd_relax yd_taud
%   (qc_yawdist_defaults 로 기본값 주입 가능)
%
% 사용: load_system(mdl) 후, sim() 전. 이미 배선돼 있으면 아무것도 안 한다.
% 대상 미발견 시 error() 즉사 (조용한 실패 금지 - 세션 규칙).

info = struct();

blks = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Control Yaw');
if numel(blks) ~= 1
    error('qc_yawdist_apply: Control Yaw %d개 (1개 예상) - 구운 모델 맞는지 확인', numel(blks));
end
pid = blks{1};
parent = get_param(pid, 'Parent');
info.pid = pid;
info.parent = parent;

if ~isempty(find_system(parent, 'SearchDepth', 1, 'Name', 'YD Ki'))
    info.skipped = true;
    fprintf('[qc_yawdist_apply] 이미 배선됨 - 통과\n');
    return;
end
info.skipped = false;

% 라이브러리 링크 해제 (상위 체인)
p = parent;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end

% --- 오차 신호(Add3 출력)와 참조 yaw 신호 찾기 ---
ph = get_param(pid, 'PortHandles');
lnE = get_param(ph.Inport(1), 'Line');
if lnE == -1
    error('qc_yawdist_apply: Control Yaw In1 에 선이 없음');
end
errSrc = get_param(lnE, 'SrcPortHandle');          % 오차 e 를 내보내는 포트 (Add3 out)
sumBlk = get_param(errSrc, 'Parent');
if ~strcmp(get_param(sumBlk, 'BlockType'), 'Sum')
    error('qc_yawdist_apply: Control Yaw In1 소스가 Sum 이 아님 (%s) - 모델 구조 변경됨', ...
          get_param(sumBlk, 'BlockType'));
end
info.sum = sumBlk;

% Sum 의 두 입력 중 '측정' 쪽(Filter Yaw 계열)이 아닌 것 = 참조
sph = get_param(sumBlk, 'PortHandles');
refSrc = -1; measName = '';
for i = 1:numel(sph.Inport)
    l = get_param(sph.Inport(i), 'Line');
    if l == -1; continue; end
    s = get_param(l, 'SrcPortHandle');
    nm = strtrim(regexprep(get_param(get_param(s,'Parent'), 'Name'), '\s+', ' '));
    if ~isempty(regexpi(nm, 'filter'))
        measName = nm;
    else
        refSrc = s;
        info.refBlock = nm;
    end
end
if refSrc == -1
    error('qc_yawdist_apply: Add3 입력에서 참조 yaw 를 못 가림 (측정=%s)', measName);
end
fprintf('[qc_yawdist_apply] 오차원=%s / 참조원=%s / 측정원=%s\n', ...
        get_param(sumBlk,'Name'), info.refBlock, measName);

pos = get_param(pid, 'Position');
x0 = pos(1) - 460; y0 = pos(2) - 140;
add = @(lib, nm, dx, dy, varargin) add_block(lib, [parent '/' nm], ...
        'Position', [x0+dx y0+dy x0+dx+60 y0+dy+30], varargin{:});

% --- 블록 생성 ---
add('simulink/Continuous/Transfer Fcn', 'YD Err LPF',   0,   0, ...
    'Numerator','[1]',    'Denominator','[yd_tau 1]');
add('simulink/Continuous/Transfer Fcn', 'YD Ref Rate',  0,  70, ...
    'Numerator','[1 0]',  'Denominator','[yd_taud 1]');
add('simulink/Signal Routing/Mux',      'YD Mux',      90,  20, 'Inputs','2');
% Fcn 블록은 min/max 미지원 -> sat01(x) = (|x| - |x-1| + 1)/2  (nl_att 와 같은 교훈)
kExpr = ['(abs(u(2))<=yd_rate)*((abs((abs(u(1))-yd_e0)/yd_e1)' ...
         '-abs((abs(u(1))-yd_e0)/yd_e1-1)+1)/2)'];
add('simulink/User-Defined Functions/Fcn', 'YD k',     170,  20, 'Expr', kExpr);
add('simulink/Discontinuities/Rate Limiter', 'YD Relax', 250, 20, ...
    'RisingSlewLimit','1e6', 'FallingSlewLimit','-yd_relax');
add('simulink/User-Defined Functions/Fcn', 'YD Ki',    330,  20, ...
    'Expr','yaw_ki*(1+(yd_gmax-1)*u)');
add('simulink/Sources/Constant', 'YD P', 330, -50, 'Value','yaw_kp');
add('simulink/Sources/Constant', 'YD D', 330,  90, 'Value','yaw_kd');
add('simulink/Sources/Constant', 'YD N', 330, 130, 'Value','yaw_f');

g = @(nm) get_param([parent '/' nm], 'PortHandles');

% --- 배선 (스케줄러 내부) ---
add_line(parent, errSrc,               g('YD Err LPF').Inport(1),  'autorouting','on');
add_line(parent, refSrc,               g('YD Ref Rate').Inport(1), 'autorouting','on');
add_line(parent, g('YD Err LPF').Outport(1),  g('YD Mux').Inport(1), 'autorouting','on');
add_line(parent, g('YD Ref Rate').Outport(1), g('YD Mux').Inport(2), 'autorouting','on');
add_line(parent, g('YD Mux').Outport(1),      g('YD k').Inport(1),   'autorouting','on');
add_line(parent, g('YD k').Outport(1),        g('YD Relax').Inport(1), 'autorouting','on');
add_line(parent, g('YD Relax').Outport(1),    g('YD Ki').Inport(1),  'autorouting','on');

% --- PID 외부 파라미터 모드 전환 + P/I/D/N 포트 배선 ---
set_param(pid, 'ControllerParametersSource', 'external');
ph2 = get_param(pid, 'PortHandles');
if numel(ph2.Inport) ~= 5
    error('qc_yawdist_apply: external 전환 후 Inport %d개 (5개 예상: e,P,I,D,N)', numel(ph2.Inport));
end
add_line(parent, g('YD P').Outport(1),  ph2.Inport(2), 'autorouting','on');
add_line(parent, g('YD Ki').Outport(1), ph2.Inport(3), 'autorouting','on');
add_line(parent, g('YD D').Outport(1),  ph2.Inport(4), 'autorouting','on');
add_line(parent, g('YD N').Outport(1),  ph2.Inport(5), 'autorouting','on');

fprintf(['[qc_yawdist_apply] 배선 완료 (P/I/D/N 외부 포트). ' ...
         'yd_gmax=%g e0=%g e1=%g tau=%g rate=%g relax=%g taud=%g\n'], ...
        evalin('base','yd_gmax'), evalin('base','yd_e0'), evalin('base','yd_e1'), ...
        evalin('base','yd_tau'),  evalin('base','yd_rate'), evalin('base','yd_relax'), ...
        evalin('base','yd_taud'));
end
