function qc_predictor_apply(mdl)
% qc_predictor_apply  위치 측정 경로에 **지연 보상 예측기**를 끼운다 (메모리 수술, save 금지).
%
% 목적: 파이썬 `control_seoungjin/delay_compensator.py` 가 추정기 단독으로 보인 이득이
%       **폐루프에서도 나오는가**를 재는 것. 특히 두 가지를 갈라 보려는 것이다:
%         (a) 추종 뒤처짐 — 줄어야 한다 (예측기가 정확히 이걸 노린다)
%         (b) roll 11 Hz 한계사이클 — **줄 수도 있다** (예상 정정)
%             처음엔 "관측기 대역 1.9 Hz 라 11 Hz 엔 손이 안 닿는다" 고 봤는데,
%             그건 파이썬 모듈의 obs_w 기준이었다. 여기 리드는 대역이 훨씬 넓어
%             11.08 Hz 에서 **위상 +26~27도**를 준다 (tau 20/60 ms 둘 다).
%             한계사이클이 위상 여유 부족이면 26도는 충분히 큰 값이다.
%             대가는 같은 주파수의 이득이 1.3~2.2배가 되는 것 — 시뮬은 잡음이
%             없어 이 대가가 안 보이지만 실기에서는 VIO 잡음이 그만큼 증폭된다.
%
% ── 왜 리드 보상기인가 ────────────────────────────────────────────────
% 순수 지연 tau 에 대한 예측은 p(t+tau) ~ p(t) + tau*dp/dt 다. 즉 **가장 단순한
% 스미스 예측기는 리드 보상기**다:
%
%       G(s) = (1 + tau_eff*s) / (tau_f*s + 1)
%
%   - 저주파 이득 1 (정상상태 안 건드림 -> 종단 오차 불변)
%   - 위상 앞섬 = 지연이 먹은 위상 여유를 되돌려 받는 부분
%   - tau_f 는 미분 필터. 없으면 고주파 이득이 무한대라 잡음을 증폭한다.
%     고주파 이득은 tau_eff/tau_f 로 묶인다.
%
% 파이썬 쪽의 알파-베타 관측기가 하는 일이 바로 이 '필터된 미분' 이라, 두 구현은
% 같은 것의 다른 표현이다. 다만 이건 **모델 없는(model-free) 변형**이다 —
% 자세 채널을 안 쓰고 측정 자신의 미분만 쓴다. 자세를 쓰는 완전판은 외란 과도에서
% 더 낫겠지만, "뒤처짐을 지우면 폐루프가 좋아지나" 라는 이 질문에는 이걸로 답이 된다.
%
% ── 보수적으로 (파이썬과 같은 규약) ──────────────────────────────────
% tau_eff = pred_trust * dly_pos_s, 기본 pred_trust = 0.7.
% 과대보상은 추정을 실제보다 앞에 놓아 양의 되먹임이 되므로, 보고된 지연보다
% **덜** 민다. 과소보상의 손해는 성능이지만 과대보상의 손해는 안정성이다.
%
% base 워크스페이스 변수:
%   dly_pos_s   [s] 위치 경로 지연 (qc_delay_apply 와 같은 값)
%   pred_trust  [-] 기본 0.7
%   pred_hfg    [-] 고주파 이득 상한, 기본 3  (tau_f = tau_eff / pred_hfg)
%
% 삽입 지점: qc_delay_apply 가 넣은 두 지연 블록의 **바로 뒤**
%   'Meas Delay Pos' -> Position Control/Subtract2 입력 2  (측정 xyz)
%   'Meas Delay Z'   -> Filter pz 입력 1                    (측정 z)
% 지연 블록이 없으면(무지연 기준선) 아무것도 하지 않는다 — 그때는 보상할 게 없다.

tau = evalin('base', 'dly_pos_s');
if ~isscalar(tau) || tau <= 0
    fprintf('[qc_predictor_apply] dly_pos_s = 0 -> 보상할 지연이 없어 건너뜀\n');
    return;
end
if evalin('base', "exist('pred_trust','var')")
    trust = evalin('base', 'pred_trust');
else
    trust = 0.7;
end
if evalin('base', "exist('pred_hfg','var')")
    hfg = evalin('base', 'pred_hfg');
else
    hfg = 3.0;
end
tau_eff = max(0, min(trust, 1.0)) * tau;
tau_f   = tau_eff / max(hfg, 1.0);
if tau_eff <= 0
    fprintf('[qc_predictor_apply] pred_trust = 0 -> 항등, 건너뜀\n');
    return;
end

% 신호 폭: 'Meas Delay Pos' 는 측정 xyz **3원소 벡터**, 'Meas Delay Z' 는 스칼라다
% (qc_delay_apply 의 삽입 지점 참조 — 앞은 Position Control/Subtract2 입력 2,
%  뒤는 Filter pz 입력 1). Transfer Fcn 은 SISO 라 벡터에 그냥 못 붙는다 —
% 처음에 붙였다가 "포트 폭 불일치" 로 죽었다. 벡터는 Demux -> 축별 보상기 -> Mux.
spec = {'Meas Delay Pos', 'Pred Lead Pos', 3; ...
        'Meas Delay Z',   'Pred Lead Z',   1};
nAdd = 0;
for k = 1:size(spec, 1)
    srcName = spec{k,1}; newName = spec{k,2};
    hits = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', srcName);
    if isempty(hits)
        continue;   % 그 경로에 지연이 안 걸려 있다
    end
    if numel(hits) ~= 1
        error('qc_predictor_apply: %s 블록 %d개 (1개 예상)', srcName, numel(hits));
    end
    dly = hits{1};
    parent = get_param(dly, 'Parent');
    if ~isempty(find_system(parent, 'SearchDepth', 1, 'Name', newName))
        continue;   % 이미 삽입됨 (재호출 안전)
    end

    ph = get_param(dly, 'PortHandles');
    ln = get_param(ph.Outport(1), 'Line');
    if ln == -1
        error('qc_predictor_apply: %s 출력에 연결된 선이 없음', srcName);
    end
    dstPorts = get_param(ln, 'DstPortHandle');
    delete_line(ln);

    pos = get_param(dly, 'Position');
    num = sprintf('[%.10g %.10g]', tau_eff, 1);
    den = sprintf('[%.10g %.10g]', tau_f, 1);
    w = spec{k,3};
    if w == 1
        blk = [parent '/' newName];
        add_block('simulink/Continuous/Transfer Fcn', blk, ...
            'Numerator', num, 'Denominator', den, ...
            'Position', [pos(1)+60 pos(2) pos(3)+60 pos(4)]);
        inPort = get_param(blk, 'PortHandles').Inport(1);
        outPort = get_param(blk, 'PortHandles').Outport(1);
    else
        % 벡터 경로: Demux -> 축별 Transfer Fcn -> Mux
        dmx = [parent '/' newName ' Demux'];
        mux = [parent '/' newName ' Mux'];
        add_block('simulink/Signal Routing/Demux', dmx, ...
            'Outputs', num2str(w), 'Position', [pos(1)+50 pos(2)-20 pos(1)+55 pos(4)+20]);
        add_block('simulink/Signal Routing/Mux', mux, ...
            'Inputs', num2str(w), 'Position', [pos(1)+200 pos(2)-20 pos(1)+205 pos(4)+20]);
        dph = get_param(dmx, 'PortHandles');
        mph = get_param(mux, 'PortHandles');
        for a = 1:w
            ab = [parent '/' newName ' ' char('W' + a)];   % X, Y, Z
            add_block('simulink/Continuous/Transfer Fcn', ab, ...
                'Numerator', num, 'Denominator', den, ...
                'Position', [pos(1)+100 pos(2)+40*(a-1)-30 pos(1)+160 pos(2)+40*(a-1)]);
            aph = get_param(ab, 'PortHandles');
            add_line(parent, dph.Outport(a), aph.Inport(1), 'autorouting', 'on');
            add_line(parent, aph.Outport(1), mph.Inport(a), 'autorouting', 'on');
        end
        inPort = dph.Inport(1);
        outPort = mph.Outport(1);
    end
    add_line(parent, ph.Outport(1), inPort, 'autorouting', 'on');
    for d = 1:numel(dstPorts)
        add_line(parent, outPort, dstPorts(d), 'autorouting', 'on');
    end
    nAdd = nAdd + 1;
end

fprintf(['[qc_predictor_apply] 리드 보상기 %d개 삽입: tau %.1f ms, trust %.2f -> ' ...
         'tau_eff %.1f ms, 미분필터 %.2f ms (고주파 이득 %.1f)\n'], ...
        nAdd, 1000*tau, trust, 1000*tau_eff, 1000*tau_f, hfg);
end
