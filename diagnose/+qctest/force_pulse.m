function force_pulse(mdl, amp, tOn, dur, axis)
% 기체 Body 프레임에 **힘** 외란 펄스를 물린다 (torque_pulse 의 힘 버전).
%   amp [N], tOn [s], dur [s], axis 'x'|'y'|'z' (기본 'y')
%
% 왜 따로 필요한가: 토크 펄스는 기체를 **돌려서** 간접적으로 움직이지만, 옆에서
% 부는 바람은 기체를 **직접 민다**. 자세 루프가 개입하기 전에 이미 위치가 밀리므로
% 응답 모양이 다르다. 둘 다 봐야 외란 강건성을 말할 수 있다.
%
% 크기 감각: 정면 0.2 m^2, Cd~1 기준 3 m/s 돌풍 = 1.1 N, 5 m/s = 3.1 N.
%
% 배선은 torque_pulse 와 같다 (같은 블록의 Force 채널을 쓸 뿐):
%   Pulse Generator -> Simulink-PS Converter -> External Force and Torque -> Body 프레임
% conserving 포트는 방향이 없어 이름으로 판별할 수 없으므로, 두 순서를 다 시도하고
% compile 이 통과하는 쪽을 채택한다.
if nargin < 5 || isempty(axis); axis = 'y'; end
axis = upper(axis(1));
if ~any(axis == 'XYZ'); error('qctest.force_pulse: axis 는 x|y|z'); end
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
if isempty(armTf1); error('qctest.force_pulse: Transform Arm1 못 찾음'); end
sys = get_param(armTf1, 'Parent');
qctest.unlink_up(sys, mdl);

bodyBlk = find_system(sys, 'SearchDepth', 1, 'BlockType', 'SubSystem', 'Name', 'Body');
bodyBlk = bodyBlk(~strcmp(bodyBlk, sys));
if isempty(bodyBlk); error('qctest.force_pulse: 내부 Body 서브시스템 못 찾음'); end
bph = get_param(bodyBlk{1}, 'PortHandles');
bconn = [bph.LConn bph.RConn];
attPort = -1;
for ci = 1:numel(bconn)
    if get_param(bconn(ci), 'Line') ~= -1 && attPort == -1; attPort = bconn(ci); end
end
if attPort == -1; error('qctest.force_pulse: Body 프레임 주입점 없음'); end

nameF = ['Disturb Force ' axis];
extB = [sys '/' nameF];
if isempty(find_system(sys, 'SearchDepth', 1, 'Name', nameF))
    add_block('sm_lib/Forces and Torques/External Force and Torque', extB);
end
set_param(extB, ['EnableForce' axis], 'on');

nameP = ['Disturb FPulse ' axis];
plsB = [sys '/' nameP];
if isempty(find_system(sys, 'SearchDepth', 1, 'Name', nameP))
    add_block('simulink/Sources/Pulse Generator', plsB);
end
set_param(plsB, 'Amplitude', num2str(amp), 'Period', num2str(PER), ...
                'PulseWidth', num2str(100 * dur / PER), 'PhaseDelay', num2str(tOn));

nameS = ['Disturb FSPS ' axis];
spsB = [sys '/' nameS];
if isempty(find_system(sys, 'SearchDepth', 1, 'Name', nameS))
    add_block('nesl_utility/Simulink-PS Converter', spsB);
end
try; set_param(spsB, 'Unit', 'N'); catch; end

pph = get_param(plsB, 'PortHandles'); sph = get_param(spsB, 'PortHandles');
if get_param(sph.Inport(1), 'Line') == -1
    add_line(sys, pph.Outport(1), sph.Inport(1), 'autorouting', 'on');
end
eph = get_param(extB, 'PortHandles');
allC = [eph.LConn eph.RConn];
if numel(allC) ~= 2
    error('qctest.force_pulse: conserving 포트 %d개 (2개 예상)', numel(allC));
end
if get_param(allC(1), 'Line') ~= -1 || get_param(allC(2), 'Line') ~= -1
    return;   % 이미 배선됨 (재호출 안전)
end
orders = [2 1; 1 2];
lastErr = '';
for oi = 1:2
    added = [];
    try
        added(end+1) = add_line(sys, attPort,      allC(orders(oi,1)), 'autorouting','on'); %#ok<AGROW>
        added(end+1) = add_line(sys, sph.RConn(1), allC(orders(oi,2)), 'autorouting','on'); %#ok<AGROW>
        feval(mdl, [], [], [], 'compile'); feval(mdl, [], [], [], 'term');
        return;
    catch ME
        lastErr = ME.message;
        try; feval(mdl, [], [], [], 'term'); catch; end
        for l = added; try; delete_line(l); catch; end; end
    end
end
% 밑에 깔린 컴파일 오류를 그대로 올린다 (torque_pulse 주석 참조 — 삼키면
% '배선 실패' 로 보이지만 실제 원인은 대개 다른 곳이다).
error('qctest.force_pulse: 외란 배선 실패 (두 순서 모두 compile 실패). 실제 오류: %s', ...
      lastErr);
end
