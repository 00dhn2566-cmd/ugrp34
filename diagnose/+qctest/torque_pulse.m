function torque_pulse(mdl, amp, tOn, dur)
% 기체 Body 프레임에 roll 축(x) 외란 토크 펄스를 물린다 — 능력 카드 R1 규약.
%   amp [N·m], tOn [s], dur [s]
%
% 배선: Pulse Generator -> Simulink-PS Converter -> External Force and Torque
%       -> Body 프레임 (Transform Arm1 의 부모 안 'Body' 서브시스템 conserving 포트)
% conserving 포트는 방향이 없어 어느 쪽에 무엇을 물릴지 이름으로 알 수 없다.
% 그래서 두 순서를 다 시도하고 compile 이 통과하는 쪽을 채택한다.
PER = 1000;   % 펄스 주기 — 시뮬 길이보다 훨씬 크게 두어 '한 번만' 인가되게 한다

armTf1 = '';
allBlk = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
for i = 1:numel(allBlk)
    try
        if strcmp(strtrim(regexprep(get_param(allBlk{i}, 'Name'), '\s+', ' ')), 'Transform Arm1')
            armTf1 = allBlk{i};
        end
    catch
    end
end
if isempty(armTf1); error('qctest.torque_pulse: Transform Arm1 못 찾음'); end
sys = get_param(armTf1, 'Parent');
qctest.unlink_up(sys, mdl);

bodyBlk = find_system(sys, 'SearchDepth', 1, 'BlockType', 'SubSystem', 'Name', 'Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, sys));
if isempty(bodyBlk); error('qctest.torque_pulse: 내부 Body 서브시스템 못 찾음'); end
bph = get_param(bodyBlk{1}, 'PortHandles');
bconn = [bph.LConn bph.RConn];
attPort = -1;
for ci = 1:numel(bconn)
    if get_param(bconn(ci), 'Line') ~= -1 && attPort == -1; attPort = bconn(ci); end
end
if attPort == -1; error('qctest.torque_pulse: Body 프레임 주입점 없음'); end

extB = [sys '/Disturb Torque X'];
if isempty(find_system(sys, 'SearchDepth', 1, 'Name', 'Disturb Torque X'))
    add_block('sm_lib/Forces and Torques/External Force and Torque', extB);
end
set_param(extB, 'EnableTorqueX', 'on');

plsB = [sys '/Disturb Pulse X'];
if isempty(find_system(sys, 'SearchDepth', 1, 'Name', 'Disturb Pulse X'))
    add_block('simulink/Sources/Pulse Generator', plsB);
end
set_param(plsB, 'Amplitude', num2str(amp), 'Period', num2str(PER), ...
                'PulseWidth', num2str(100 * dur / PER), 'PhaseDelay', num2str(tOn));

spsB = [sys '/Disturb SPS X'];
if isempty(find_system(sys, 'SearchDepth', 1, 'Name', 'Disturb SPS X'))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try; set_param(spsB, 'Unit', 'N*m'); catch; end

pph = get_param(plsB, 'PortHandles'); sph = get_param(spsB, 'PortHandles');
if get_param(sph.Inport(1), 'Line') == -1
    add_line(sys, pph.Outport(1), sph.Inport(1), 'autorouting', 'on');
end
eph = get_param(extB, 'PortHandles');
allC = [eph.LConn eph.RConn];
if numel(allC) ~= 2
    error('qctest.torque_pulse: conserving 포트 %d개 (2개 예상)', numel(allC));
end
if get_param(allC(1), 'Line') ~= -1 || get_param(allC(2), 'Line') ~= -1
    return;   % 이미 배선됨 (재호출 안전)
end
orders = [2 1; 1 2];
lastErr = '';
for oi = 1:2
    added = [];
    try
        added(end+1) = add_line(sys, attPort,        allC(orders(oi,1)), 'autorouting','on'); %#ok<AGROW>
        added(end+1) = add_line(sys, sph.RConn(1),   allC(orders(oi,2)), 'autorouting','on'); %#ok<AGROW>
        feval(mdl, [], [], [], 'compile'); feval(mdl, [], [], [], 'term');
        return;
    catch ME
        lastErr = ME.message;
        try; feval(mdl, [], [], [], 'term'); catch; end
        for l = added; try; delete_line(l); catch; end; end
    end
end
% ★ 밑에 깔린 컴파일 오류를 그대로 올린다. 이걸 삼키면 '배선 실패' 로 보이지만
%   실제 원인은 대개 **다른 곳**이다 (2026-08-23: qc_mass_sched_apply 가 이 머신에
%   없는 변수를 참조해 컴파일이 깨진 것을 배선 탓으로 30분 오인했다).
error('qctest.torque_pulse: 외란 배선 실패 (두 순서 모두 compile 실패). 실제 오류: %s', ...
      lastErr);
end
