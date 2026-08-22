function qc_ff_trim_apply(mdl)
%QC_FF_TRIM_APPLY 호버 FF 트림을 √질량 법칙으로 배선 (메모리 수술, save 금지 규칙 준수)
%   구운 .slx: Bias Chassis(=56.5) + Bias Load(=44.4·pkgSize³·pkgDensity) 가 모터 ref에 더해지는 호버 트림.
%   parameters.m 의 bias_hover_rps = 100.9·√(m_tot/2.2726) 로 교체하고 Bias Load 는 0 으로 둔다.
%   1 kg 에서 수치 불변(100.9). 대상 미발견 시 error() 즉사. 사용: load_system(mdl) 후, sim() 전.
%   성능 지표 세션 2026-08-18 (근거: tune_0kg_r2 — bias 56.5→75 에서 0 kg 이륙 새그 14.7→4.4 cm).

bc = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Bias Chassis');
bl = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Bias Load');
if numel(bc) ~= 1 || numel(bl) ~= 1
    error('qc_ff_trim_apply: Bias Chassis %d개 / Bias Load %d개 (각 1개 예상) - 구운 모델 맞는지 확인', numel(bc), numel(bl));
end
for blk = {bc{1}, bl{1}}
    p = get_param(blk{1}, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
                set_param(p, 'LinkStatus', 'none');
            end
        catch
        end
        p = get_param(p, 'Parent');
    end
end
set_param(bc{1}, 'Bias', 'bias_hover_rps');
set_param(bl{1}, 'Gain', '0');
end
