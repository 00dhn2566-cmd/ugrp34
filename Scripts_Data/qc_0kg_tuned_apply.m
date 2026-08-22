function info = qc_0kg_tuned_apply(mdl)
%QC_0KG_TUNED_APPLY  08-18 성능 지표 세션이 채택한 0 kg 튜닝 구성 복원 (메모리 수술).
%
%   출처: diagnose/tune_0kg_r5.m 의 BASE 구조체 + eval_cfg 배선 (그 세션이 실제로 돌린 코드).
%         문서 요약은 PERFORMANCE_SPEC_0KG.md v0.3 §0.
%
%   ※ 왜 이 스크립트가 필요한가 —
%     이 채택안은 원래 `parameters.m` 에 들어갔어야 하는데(보드 08-18 19:10),
%     그 파일이 이 머신으로 동기되지 않았다. `parameters.m` 은 아직 07-19 18차 앵커
%     (sA_mass = 0.75 + …) 이고, `nl_gmax`/`filtPz_mass`/`bias_hover_rps` 도 미정의다.
%     → 원본 `parameters.m` 을 회수하면 이 스크립트는 폐기할 것. [TODO-회수]
%
%   채택 근거 요약:
%     sA 0.35   : 0.75 는 5 Hz 세차 한계사이클 ±8°, 0.40 은 자려 지터 0.156° → 0.35 에서 0.005°
%     r_att 0.6 : 0.4 이동 불안정 / 0.8 호버 초과
%     limit_att 100 : 20~40 은 외란 권한 부족(이탈 26~78°), ≥50 동일 → 이동 중 명령 <50
%     kp_pos 5  : 0 kg 에서 8 은 이동 오버슈트 확대
%     filtPz 0.005 : 이륙 새그 14.6 → 5.3 cm
%     biasChassis 75.5 : 0 kg 호버 FF 트림 (56.5 는 1 kg 값)
%     nl_gmax 2.1 / e0=e1=3° : 외란 이탈 34 → 10.1°, 밀림 10.3 → 0.26 m, 재진입 5.3 s
%
%   사용: quadcopter_package_parameters; load_system(mdl); qc_0kg_tuned_apply(mdl);  (sim 전)
%   규칙: save_system 금지. 대상 미발견 시 error() 즉사.

if ~evalin('base', "exist('sT','var')")
    error('qc_0kg_tuned_apply: parameters.m 을 먼저 실행할 것 (sT 없음)');
end
sT = evalin('base', 'sT');
sQ = evalin('base', 'sQ');

% ---------- 채택 구성 (tune_0kg_r5.m BASE, 08-18) ----------
c = struct( ...
    'sA', 0.35, 'r_att', 0.6, 'ki_att', -10, 'filtD_att', 2500, 'filtM_att_meas', 0.05, 'limit_att', 100, ...
    'kp_pos', 5, 'kd_pos', 2.0, 'ki_pos', 0.04, 'pos2att', 2.4, 'posErrSat', 1.2/5, 'posErrSatZ', 1.2/5, ...
    'filtM_pos', 0.005, 'filtD_pos', 100, 'tiltLimit', pi/3, ...
    'sZ', 0.56, 'kp_alt', 0.5, 'ki_alt', 0.1, 'kd_alt', 0.15, 'filtM_alt', 0.05, 'filtD_alt', 1000, ...
    'filtPz', 0.005, 'limit_alt', 10, 'altCmdSat', 30, 'biasChassis', 75.5, ...
    'kp_yaw', 15, 'ki_yaw', 1.5, 'kd_yaw', 4, 'filtM_yaw', 0.01, 'filtD_yaw', 100, 'limit_yaw', 20, ...
    'nl_gmax', 2.1, 'nl_e0_deg', 3, 'nl_e1_deg', 3);
info.cfg = c;

% ---------- 플랜트 질량 0 kg (극소 밀도 웰드 — perf_battery/tune_0kg 관례) ----------
pkgSize = [1 1 1] * 0.14;
assignin('base', 'pkgSize', pkgSize);
assignin('base', 'pkgDensity', 1e-6 / prod(pkgSize));
assignin('base', 'm_pkg_now', 0);

% ---------- 게인 배선 (tune_0kg_r5.m eval_cfg 와 1:1) ----------
A = @(n, v) assignin('base', n, v);
A('sA_mass', c.sA);
A('kp_attitude', -85 * sT * c.sA);
A('ki_attitude', c.ki_att * sT * c.sA);
A('kd_attitude', -85 * c.r_att * sT * c.sA);
A('filtD_attitude', c.filtD_att);
A('filtM_att_meas', c.filtM_att_meas);
A('limit_attitude', c.limit_att);
A('kp_position', c.kp_pos); A('kd_position', c.kd_pos); A('ki_position', c.ki_pos);
A('pos2attitude', c.pos2att);
A('posErrSat', c.posErrSat); A('posErrSatZ', c.posErrSatZ);
A('filtM_position', c.filtM_pos); A('filtD_position', c.filtD_pos);
A('tiltLimit', c.tiltLimit);
A('sZ_mass', c.sZ);
A('kp_altitude', c.kp_alt * sT * c.sZ);
A('ki_altitude', c.ki_alt * sT * c.sZ);
A('kd_altitude', c.kd_alt * sT * c.sZ);
A('filtM_altitude', c.filtM_alt); A('filtD_altitude', c.filtD_alt); A('filtPz', c.filtPz);
A('limit_altitude', c.limit_alt); A('altCmdSat', c.altCmdSat); A('biasChassis', c.biasChassis);
A('kp_yaw', c.kp_yaw * sQ); A('ki_yaw', c.ki_yaw * sQ); A('kd_yaw', c.kd_yaw * sQ);
A('filtM_yaw', c.filtM_yaw); A('filtD_yaw', c.filtD_yaw); A('limit_yaw', c.limit_yaw);
A('nl_gmax', c.nl_gmax);
A('nl_e0', deg2rad(c.nl_e0_deg)); A('nl_e1', deg2rad(c.nl_e1_deg));

% ---------- 모델 하드코딩 상수 변수화 (r5 와 동일 5종) ----------
vz = { 'Alt Cmd Sat',  {'UpperLimit','altCmdSat',  'LowerLimit','-altCmdSat'}; ...
       'Pitch Limit',  {'UpperLimit','tiltLimit',  'LowerLimit','-tiltLimit'}; ...
       'Roll Limit',   {'UpperLimit','tiltLimit',  'LowerLimit','-tiltLimit'}; ...
       'Filter pz',    {'Denominator','[filtPz 1]'}; ...
       'Filter Pitch', {'Denominator','[filtM_att_meas 1]'}; ...
       'Filter Roll',  {'Denominator','[filtM_att_meas 1]'}; ...
       'Bias Chassis', {'Bias','biasChassis'} };
for k = 1:size(vz,1)
    b = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', vz{k,1});
    if numel(b) ~= 1
        error('qc_0kg_tuned_apply: %s %d개 (1개 예상) - 구운 모델 맞는지 확인', vz{k,1}, numel(b));
    end
    p = get_param(b{1}, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            if any(strcmp(get_param(p,'LinkStatus'), {'resolved','inactive'}))
                set_param(p, 'LinkStatus', 'none');
            end
        catch
        end
        p = get_param(p, 'Parent');
    end
    set_param(b{1}, vz{k,2}{:});
end
qc_zsplit_apply(mdl);        % 18차 z분리 (r5 도 같이 걸었다)
qc_nl_att_apply(mdl);        % 오차 의존 비선형 자세 게인 (nl_gmax=2.1)

fprintf(['[qc_0kg_tuned_apply] 08-18 채택 0 kg 구성 복원: sA %.2f / kd:kp %.1f / limit_att %g / ' ...
         'kp_pos %g / filtPz %.3f / biasChassis %.1f / nl_gmax %.1f(e0 %g도)\n'], ...
        c.sA, c.r_att, c.limit_att, c.kp_pos, c.filtPz, c.biasChassis, c.nl_gmax, c.nl_e0_deg);
end
