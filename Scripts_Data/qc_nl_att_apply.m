function qc_nl_att_apply(mdl)
% qc_nl_att_apply  자세 PID 입력에 오차 의존 비선형 게인 삽입 (메모리 수술, save 금지). 2026-08-18 성능 지표 세션.
%
% 목적(사용자): 0 kg 에서 호버 정밀(sA 0.35 = 지터 0.005°)은 유지하되 외란 이탈의 '크기'를 범위 안에 가둔다.
%   e_pid = e · g(|e|),  g = 1 + (nl_gmax-1)·sat((|e|-nl_e0)/nl_e1)   [e: rad]
%   |e| < nl_e0        : g = 1       (평시 게인 그대로 → 지터 무영향)
%   |e| > nl_e0+nl_e1  : g = nl_gmax (P/I/D 동시 배율 = sA 배율과 동일 효과 → 권한 상승)
%   nl_gmax = 1 이면 항등(기존 동작 불변). 1 kg 회귀: 질량 스케줄이 1 kg 에서 nl_gmax=1 을 주므로 골든 불변.
% 배선: Altitude and YPR Control / Control Pitch(Roll) PID 의 Inport 1 앞에 Fcn 블록 'NL Gain Pitch/Roll' 삽입.
%   base 워크스페이스 변수 nl_gmax / nl_e0 / nl_e1 참조. 이미 삽입돼 있으면 재삽입하지 않음.
% C++ 동기: qc_controller.hpp SwingDamper 아래 'NlAttGain' (동일 식) — [TODO-verify].
for nm = {'Pitch', 'Roll'}
    pid = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', ['Control ' nm{1}]);
    if numel(pid) ~= 1
        error('qc_nl_att_apply: Control %s 블록 %d개 (1개 예상) - 구운 모델 맞는지 확인', nm{1}, numel(pid));
    end
    pid = pid{1};
    parent = get_param(pid, 'Parent');
    fcnName = ['NL Gain ' nm{1}];
    if ~isempty(find_system(parent, 'SearchDepth', 1, 'Name', fcnName))
        continue;   % 이미 삽입
    end
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
    ph = get_param(pid, 'PortHandles');
    ln = get_param(ph.Inport(1), 'Line');
    if ln == -1
        error('qc_nl_att_apply: Control %s Inport 1 에 연결된 선이 없음', nm{1});
    end
    src = get_param(ln, 'SrcPortHandle');
    delete_line(ln);
    fcn = [parent '/' fcnName];
    pos = get_param(pid, 'Position');
    add_block('simulink/User-Defined Functions/Fcn', fcn, ...
        'Expression', 'u*(1+(nl_gmax-1)*(abs((abs(u)-nl_e0)/nl_e1)-abs((abs(u)-nl_e0)/nl_e1-1)+1)/2)', ...   % Fcn 블록은 min/max 미지원 -> sat01(x)=(|x|-|x-1|+1)/2 (r5 1차 실행 전부 NaN 원인)
        'Position', [pos(1)-90 pos(2) pos(1)-40 pos(2)+30]);
    fph = get_param(fcn, 'PortHandles');
    add_line(parent, src, fph.Inport(1), 'autorouting', 'on');
    add_line(parent, fph.Outport(1), ph.Inport(1), 'autorouting', 'on');
end
fprintf('[qc_nl_att_apply] 자세 PID 비선형 게인 삽입 (nl_gmax/nl_e0/nl_e1 = base 워크스페이스)\n');
end
