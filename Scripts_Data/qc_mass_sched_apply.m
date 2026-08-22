function qc_mass_sched_apply(mdl)
%QC_MASS_SCHED_APPLY 질량 스케줄 중 .slx 하드코딩 상수 배선 (메모리 수술, save 금지 규칙 준수)
%   현재 대상: 'Filter pz' (z 측정 필터, 구운 값 [0.01 1]) -> [filtPz_mass 1]
%   parameters.m 의 filtPz_mass = 0.005 + 0.005·min(m_pkg,1) (0 kg 재튜닝 08-18: 0.005 에서 이륙 새그 14.6 -> 5.3 cm).
%   1 kg 에서 0.01 로 구운 값과 동일 -> 회귀 없음. 대상 미발견 시 error() 즉사.
%   사용: load_system(mdl) 후, sim() 전 (qc_zsplit_apply / qc_ff_trim_apply 와 같은 자리).

fz = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', 'Filter pz');
if numel(fz) ~= 1
    error('qc_mass_sched_apply: Filter pz %d개 (1개 예상) - 구운 모델 맞는지 확인', numel(fz));
end
p = get_param(fz{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
set_param(fz{1}, 'Denominator', '[filtPz_mass 1]');
end
