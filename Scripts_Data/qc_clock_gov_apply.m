function info = qc_clock_gov_apply(mdl)
%QC_CLOCK_GOV_APPLY  외란 연동 속도 조속기 배선 (메모리 수술, save_system 금지).
%   2026-08-22 외란 강건화 세션. 설계: control_seoungjin/docs/SPEED_GOVERNOR.md
%
% 하는 일 — 궤적 참조를 읽는 시계를 실시간 조절 가능하게 만든다.
%   기존:  Reference/Clock1 ──► Interpolate Spline Points  (t 를 그대로)
%   수술:  ∫s dt          ──► Interpolate Spline Points  (가상 시계 tau)
%
%   s = 1 − gov_on · d,      d = 3차임계감쇠필터( d* )
%   d* = (1 − gov_smin) · smootherstep( sat01( (rho_eff − gov_rf)/(gov_rs − gov_rf) ) )
%   rho_eff = max( LPF(|u_yaw|/limit_yaw),  LPF(|e_yaw|)/gov_psi_stop )
%
% 항등 보장: gov_on = 0 이면 s ≡ 1 → tau = ∫1 dt = t (상수 적분은 어떤 솔버에서도 정확)
%            → 구운 모델과 완전히 같은 참조. 필터를 '1로부터의 편차 d' 에 걸었으므로
%            초기조건 0 이 곧 s(0)=1 이다 (s 를 직접 필터하면 s(0)=0 이 되어 시계가 멎는다).
%
% 필요한 base 워크스페이스 변수: limit_yaw (parameters.m) + qc_clock_gov_defaults 의 gov_*
%
% 사용: load_system(mdl) 후, sim() 전. 이미 배선돼 있으면 아무것도 안 한다.
% 대상 미발견 시 error() 즉사 (조용한 실패 금지 — 세션 규칙).

info = struct();

% ---------- 1. 대상 찾기 ----------
clk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                  'BlockType','Clock', 'Name','Clock1');
if numel(clk) ~= 1
    error('qc_clock_gov_apply: Reference/Clock1 %d개 (1개 예상)', numel(clk));
end
clk = clk{1};
ref = get_param(clk, 'Parent');           % .../Reference
info.clock = clk;  info.ref = ref;

yawPid = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name','Control Yaw');
if numel(yawPid) ~= 1
    error('qc_clock_gov_apply: Control Yaw %d개 (1개 예상)', numel(yawPid));
end
yawPid = yawPid{1};
ypr = get_param(yawPid, 'Parent');        % .../Altitude and  YPR Control
info.yawPid = yawPid;  info.ypr = ypr;

if ~isempty(find_system(ref, 'SearchDepth', 1, 'Name', 'QC Clock Int'))
    info.skipped = true;
    fprintf('[qc_clock_gov_apply] 이미 배선됨 - 통과\n');
    return;
end
info.skipped = false;

qc_unlink_chain(ref, mdl);
qc_unlink_chain(ypr, mdl);

% ---------- 2. YPR 쪽 신호 탭 (Goto, 전역 태그) ----------
% u_yaw : Control Yaw 출력 (= 외란 상쇄에 쓰는 권한)
% e_yaw : Control Yaw 입력 (= Add3 출력, yaw 오차)
yph = get_param(yawPid, 'PortHandles');
lIn = get_param(yph.Inport(1), 'Line');
if lIn == -1; error('qc_clock_gov_apply: Control Yaw In1 미연결'); end
eSrc = get_param(lIn, 'SrcPortHandle');
uSrc = yph.Outport(1);

ypos = get_param(yawPid, 'Position');
qc_add(ypr, 'simulink/Signal Routing/Goto', 'QC Goto uYaw', ...
       [ypos(1)+120 ypos(2)-60 ypos(1)+200 ypos(2)-30], ...
       'GotoTag','qc_uyaw', 'TagVisibility','global');
qc_add(ypr, 'simulink/Signal Routing/Goto', 'QC Goto eYaw', ...
       [ypos(1)+120 ypos(2)+40 ypos(1)+200 ypos(2)+70], ...
       'GotoTag','qc_eyaw', 'TagVisibility','global');
add_line(ypr, uSrc, get_param([ypr '/QC Goto uYaw'],'PortHandles').Inport(1), 'autorouting','on');
add_line(ypr, eSrc, get_param([ypr '/QC Goto eYaw'],'PortHandles').Inport(1), 'autorouting','on');

% ---------- 3. Reference 쪽 조속기 ----------
cpos = get_param(clk, 'Position');
x0 = cpos(1) - 560;  y0 = cpos(2) + 120;
A = @(lib, nm, dx, dy, varargin) qc_add(ref, lib, nm, ...
        [x0+dx y0+dy x0+dx+70 y0+dy+30], varargin{:});

A('simulink/Signal Routing/From', 'QC From uYaw',   0,   0, 'GotoTag','qc_uyaw');
A('simulink/Signal Routing/From', 'QC From eYaw',   0,  80, 'GotoTag','qc_eyaw');

% rho = |u_yaw| / limit_yaw ,  epsi_n = |e_yaw| / gov_psi_stop   (둘 다 1차 저역통과)
A('simulink/User-Defined Functions/Fcn', 'QC Rho',   100,   0, 'Expr','abs(u)/limit_yaw');
% ±pi 랩 필수: Simulink Add3 는 ref-meas 를 그대로 낸다 (C++ 는 wrapPi 적용).
% 랩 안 하면 한 바퀴 돈 것을 359도 오차로 보고 조속기가 영원히 안 풀린다.
A('simulink/User-Defined Functions/Fcn', 'QC EpsiN', 100,  80, ...
  'Expr','abs(u-2*pi*floor((u+pi)/(2*pi)))/gov_psi_stop');
A('simulink/Continuous/Transfer Fcn',    'QC Rho LPF',   200,  0, ...
  'Numerator','[1]', 'Denominator','[gov_tau_rho 1]');
A('simulink/Continuous/Transfer Fcn',    'QC EpsiN LPF', 200, 80, ...
  'Numerator','[1]', 'Denominator','[gov_tau_psi 1]');

% rho_eff = max(둘)
A('simulink/Math Operations/MinMax', 'QC RhoEff', 300, 40, 'Function','max', 'Inputs','2');

% d* = (1-smin)·smootherstep( sat01((rho_eff-rf)/(rs-rf)) )
% Fcn 블록은 min/max 미지원 -> sat01(x) = (|x| - |x-1| + 1)/2  (qc_nl_att_apply 교훈)
A('simulink/User-Defined Functions/Fcn', 'QC W', 400, 40, ...
  'Expr', ['(abs((u-gov_rf)/(gov_rs-gov_rf))-abs((u-gov_rf)/(gov_rs-gov_rf)-1)+1)/2']);
% 벗어난 양에 '선형 비례' (사용자 지시). 뒤의 3차 필터가 C0 입력을 C3 로 만들어 주므로
% 스냅 연속성(§6)은 유지된다 — smootherstep 이 없어도 된다.
A('simulink/User-Defined Functions/Fcn', 'QC Dstar', 500, 40, ...
  'Expr', '(1-gov_smin)*u');

% 3차 임계감쇠 필터: (s/w+1)^3 -> [1/w^3, 3/w^2, 3/w, 1].  '1로부터의 편차' d 를 필터 -> IC 0 이 곧 s(0)=1
A('simulink/Continuous/Transfer Fcn', 'QC D Filt', 600, 40, ...
  'Numerator','[1]', 'Denominator','[1/gov_ws^3 3/gov_ws^2 3/gov_ws 1]');

% s = 1 - gov_on·d   (gov_on=0 이면 s≡1, 항등)
A('simulink/User-Defined Functions/Fcn', 'QC S', 700, 40, 'Expr','1-gov_on*u');

% tau = ∫ s dt
A('simulink/Continuous/Integrator', 'QC Clock Int', 800, 40, 'InitialCondition','0');

G = @(nm) get_param([ref '/' nm], 'PortHandles');
add_line(ref, G('QC From uYaw').Outport(1),   G('QC Rho').Inport(1),      'autorouting','on');
add_line(ref, G('QC From eYaw').Outport(1),   G('QC EpsiN').Inport(1),    'autorouting','on');
add_line(ref, G('QC Rho').Outport(1),         G('QC Rho LPF').Inport(1),  'autorouting','on');
add_line(ref, G('QC EpsiN').Outport(1),       G('QC EpsiN LPF').Inport(1),'autorouting','on');
add_line(ref, G('QC Rho LPF').Outport(1),     G('QC RhoEff').Inport(1),   'autorouting','on');
add_line(ref, G('QC EpsiN LPF').Outport(1),   G('QC RhoEff').Inport(2),   'autorouting','on');
add_line(ref, G('QC RhoEff').Outport(1),      G('QC W').Inport(1),        'autorouting','on');
add_line(ref, G('QC W').Outport(1),           G('QC Dstar').Inport(1),    'autorouting','on');
add_line(ref, G('QC Dstar').Outport(1),       G('QC D Filt').Inport(1),   'autorouting','on');
add_line(ref, G('QC D Filt').Outport(1),      G('QC S').Inport(1),        'autorouting','on');
add_line(ref, G('QC S').Outport(1),           G('QC Clock Int').Inport(1),'autorouting','on');

% ---------- 4. 시계 교체 ----------
cph = get_param(clk, 'PortHandles');
lc = get_param(cph.Outport(1), 'Line');
if lc == -1
    error('qc_clock_gov_apply: Clock1 출력이 미연결 - 모델 구조 변경됨');
end
dsts = get_param(lc, 'DstPortHandle');
delete_line(lc);
for k = 1:numel(dsts)
    add_line(ref, G('QC Clock Int').Outport(1), dsts(k), 'autorouting','on');
end
% 남은 Clock1 출력은 Terminator 로 마감 (미연결 경고 방지)
A('simulink/Sinks/Terminator', 'QC Clock Term', 800, -60);
add_line(ref, cph.Outport(1), G('QC Clock Term').Inport(1), 'autorouting','on');

fprintf(['[qc_clock_gov_apply] 가상 시계 배선 완료 (참조 %d곳 재연결). ' ...
         'gov_on=%g rf=%g rs=%g smin=%g ws=%g psi_stop=%.1fdeg\n'], numel(dsts), ...
        evalin('base','gov_on'), evalin('base','gov_rf'), evalin('base','gov_rs'), ...
        evalin('base','gov_smin'), evalin('base','gov_ws'), ...
        evalin('base','gov_psi_stop')*180/pi);
end

% ================= 로컬 =================
function qc_unlink_chain(blk, mdl)
p = blk;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
end

function qc_add(parent, lib, nm, pos, varargin)
full = [parent '/' nm];
if isempty(find_system(parent, 'SearchDepth', 1, 'Name', nm))
    add_block(lib, full, 'Position', pos, varargin{:});
end
end
