function info = qc_mass_lerp_apply(mdl, m_pkg)
%QC_MASS_LERP_APPLY  주요 튜닝값을 짐 질량으로 **선형 보간**해 적용 (메모리 수술).
%
%   사용자 지시 2026-08-23: "무게에 따라서 중요 튜닝 값들 선형 보간해서 넣어보고 검증해봐"
%
%   ## 왜 필요한가 — 현행 스케줄이 08-18 재튜닝과 어긋나 있다
%
%   `parameters.m` 은 `sA_mass = 0.75 + 0.25*m` 을 쓴다 (07-19 18차 앵커).
%   그런데 08-18 성능 세션이 0 kg 을 다시 튜닝해 **sA 0.35** 를 채택했다
%   (0.75 는 5 Hz 세차 한계사이클 ±8°, 0.40 은 자려 지터 0.156° -> 0.35 에서 0.005°).
%   그 결과가 이 머신의 `parameters.m` 에 동기되지 않아, 지금은
%     · 1 kg 근처는 parameters.m 스케줄
%     · 0 kg 은 `qc_0kg_tuned_apply` 라는 **별도 이산 구성**
%   으로 갈라져 있고 그 사이 질량은 어느 쪽도 아니다.
%
%   이 함수는 두 실측 앵커(0 kg 08-18 채택 / 1 kg 현행)를 잇는 **하나의 1차식**으로
%   통일한다. 0/1 kg 에서는 각 앵커를 정확히 재현하고, 사이는 선형으로 채운다.
%
%   ## 무엇을 보간하고 무엇을 안 하나
%
%   보간(두 앵커가 다른 것):
%     sA_mass 0.35->1.00 / kd:kp 비 0.6->1.5 / limit_attitude 100->800 /
%     kp_position 5->8 / sZ_mass 0.56->1.00 / filtPz 0.005->0.01 /
%     biasChassis 75.5->56.5 / nl_gmax 2.1->1.0
%
%   보간 안 함(두 앵커가 같음 — 실측으로 확인, probe_1kg_anchor.m):
%     filtD_attitude 2500 / filtM_att_meas 0.05 / tiltLimit pi/3 / altCmdSat 30 /
%     ki_attitude 계수 -10 / 위치 ki 0.04 / pos2attitude 2.4 / yaw 전체
%
%   ★ 파생 관계는 보간하지 않고 **다시 계산**한다:
%     posErrSat = 1.2 / kp_position   (곱 불변식, 15차 사용자 설계)
%     kd_position = 0.4 * kp_position (두 앵커 모두 비가 0.4 — 2.0/5 = 3.2/8)
%   보간한 값끼리 어긋나면 불변식이 깨지므로, 한 축만 보간하고 나머지는 유도한다.
%
%   사용: quadcopter_package_parameters; load_system(mdl); qc_mass_lerp_apply(mdl, 0.5);
%   규칙: save_system 금지. 대상 미발견 시 error() 즉사.

if nargin < 2 || isempty(m_pkg)
    m_pkg = evalin('base', 'm_pkg_now');
end
if ~evalin('base', "exist('sT','var')")
    error('qc_mass_lerp_apply: parameters.m 을 먼저 실행할 것 (sT 없음)');
end
sT = evalin('base', 'sT');
sQ = evalin('base', 'sQ');

% 앵커 밖은 클램프한다 — 2 kg 은 1 kg 복사본이라 외삽할 근거가 없다.
u = min(max(double(m_pkg), 0), 1);
L = @(a0, a1) a0 + (a1 - a0) * u;      % 0 kg 값 -> 1 kg 값

c.sA         = L(0.35,  1.00);
c.r_att      = L(0.60,  1.50);         % kd/kp 비
c.limit_att  = L(100,   800);
c.kp_pos     = L(5,     8);
c.sZ         = L(0.56,  1.00);
c.filtPz     = L(0.005, 0.01);
c.biasChassis= L(75.5,  56.5);
c.nl_gmax    = L(2.1,   1.0);
% 파생 (보간 아님)
c.kd_pos     = 0.4 * c.kp_pos;
c.posErrSat  = 1.2 / c.kp_pos;
% 두 앵커 공통
c.ki_att = -10; c.filtD_att = 2500; c.filtM_att_meas = 0.05;
c.ki_pos = 0.04; c.pos2att = 2.4; c.filtM_pos = 0.005; c.filtD_pos = 100;
c.tiltLimit = pi/3;
c.kp_alt = 0.5; c.ki_alt = 0.1; c.kd_alt = 0.15;
c.filtM_alt = 0.05; c.filtD_alt = 1000; c.limit_alt = 10; c.altCmdSat = 30;
c.kp_yaw = 15; c.ki_yaw = 1.5; c.kd_yaw = 4;
c.filtM_yaw = 0.01; c.filtD_yaw = 100; c.limit_yaw = 20;
c.nl_e0_deg = 3; c.nl_e1_deg = 3;
info.m_pkg = m_pkg; info.u = u; info.cfg = c;

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
A('posErrSat', c.posErrSat); A('posErrSatZ', c.posErrSat);
A('filtM_position', c.filtM_pos); A('filtD_position', c.filtD_pos);
A('tiltLimit', c.tiltLimit);
A('sZ_mass', c.sZ);
A('kp_altitude', c.kp_alt * sT * c.sZ);
A('ki_altitude', c.ki_alt * sT * c.sZ);
A('kd_altitude', c.kd_alt * sT * c.sZ);
A('filtM_altitude', c.filtM_alt); A('filtD_altitude', c.filtD_alt);
A('filtPz', c.filtPz);
A('limit_altitude', c.limit_alt); A('altCmdSat', c.altCmdSat);
A('biasChassis', c.biasChassis);
A('kp_yaw', c.kp_yaw * sQ); A('ki_yaw', c.ki_yaw * sQ); A('kd_yaw', c.kd_yaw * sQ);
A('filtM_yaw', c.filtM_yaw); A('filtD_yaw', c.filtD_yaw); A('limit_yaw', c.limit_yaw);
A('nl_gmax', c.nl_gmax);
A('nl_e0', deg2rad(c.nl_e0_deg)); A('nl_e1', deg2rad(c.nl_e1_deg));

% 모델에 하드코딩된 상수를 변수 참조로 바꾼다 (qc_0kg_tuned_apply 와 같은 5종+2).
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
        error('qc_mass_lerp_apply: %s %d개 (1개 예상) - 구운 모델 맞는지 확인', ...
              vz{k,1}, numel(b));
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
qc_zsplit_apply(mdl);
qc_nl_att_apply(mdl);

fprintf(['[qc_mass_lerp_apply] m_pkg %.2f kg: sA %.3f / kd:kp %.2f / limit_att %.0f / ' ...
         'kp_pos %.2f / sZ %.3f / filtPz %.4f / bias %.1f / nl_gmax %.2f\n'], ...
        m_pkg, c.sA, c.r_att, c.limit_att, c.kp_pos, c.sZ, c.filtPz, ...
        c.biasChassis, c.nl_gmax);
end
