function info = qc_antiwindup_apply(mdl, mode, targets)
%QC_ANTIWINDUP_APPLY  PID 적분기 와인드업 방지 켜기 (메모리 수술, save_system 금지).
%   2026-08-22 외란 강건화 세션. 사용자 지시: "적분 값도 saturation 방지 로직 같이 돌려봐".
%
% 배경: 구운 모델의 PID 들은 `LimitOutput=on` 인데 `AntiWindupMode='none'` 이다.
%   출력만 클램프하고 적분기는 계속 적분한다 (TUNING_STATUS 에 의도된 것으로 명시돼 있음).
%   → 외란으로 출력이 포화하면 적분기가 무한정 쌓이고, 외란이 사라져도 그만큼
%     되돌아오는 데 시간이 걸린다 (verify_yawdist_pulse: 0.3 N·m 3초 → 25초 내 미정착).
%
% mode    : 'clamping' (기본) | 'back-calculation' | 'none'(원복)
%           clamping  = 포화 중이고 오차가 포화를 더 밀면 적분 정지 (조건부 적분)
%           back-calc = 포화 초과분을 Kb 로 적분기에 되먹임
% targets : 셀 배열. 기본 {'Control Yaw'}. 예: {'Control Yaw','Control Pitch','Control Roll'}
%
% 항등성: 출력이 한 번도 포화하지 않는 구간에서는 완전히 항등이다.
%   달라지는 것은 포화 구간뿐이고, 거기서는 와인드업을 막는 쪽이 옳다.
%   ※ 다만 이 모델은 게인이 포화 동작까지 포함해 튜닝됐을 수 있다 (TUNING_STATUS).
%     반드시 회귀(무외란 + 기존 미션)를 같이 돌려서 확인할 것.

if nargin < 2 || isempty(mode);    mode = 'clamping'; end
if nargin < 3 || isempty(targets); targets = {'Control Yaw'}; end

info = struct('mode', mode, 'applied', {{}});

for k = 1:numel(targets)
    nm = targets{k};
    blks = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', nm);
    if numel(blks) ~= 1
        error('qc_antiwindup_apply: %s %d개 (1개 예상)', nm, numel(blks));
    end
    pid = blks{1};

    p = get_param(pid, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
                set_param(p, 'LinkStatus', 'none');
            end
        catch
        end
        p = get_param(p, 'Parent');
    end

    if ~strcmp(get_param(pid, 'LimitOutput'), 'on')
        error('qc_antiwindup_apply: %s 의 LimitOutput 이 off — 클램프가 없으면 와인드업 방지도 의미 없음', nm);
    end
    before = get_param(pid, 'AntiWindupMode');
    set_param(pid, 'AntiWindupMode', mode);
    info.applied{end+1} = sprintf('%s: %s -> %s', nm, before, mode); %#ok<AGROW>
    fprintf('[qc_antiwindup_apply] %s  AntiWindupMode %s -> %s  (Limit ±%s)\n', ...
            nm, before, mode, get_param(pid,'UpperSaturationLimit'));
end
end
