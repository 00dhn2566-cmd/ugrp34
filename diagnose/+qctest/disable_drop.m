function disable_drop(mdl)
% 투하 로직 무력화.
%
% 왜 필수인가 — 투하가 걸리면 질량이 2.27 -> 1.27 kg 로 떨어지는데 게인은 1 kg
% 스케줄 그대로라, 그 뒤 나오는 발산(08-22 세션에서 7~29 m)은 **시험 조건의 산물**
% 이지 지연/외란 탓이 아니다. 지연·외란만 보고 싶으면 반드시 꺼야 한다.
b = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
      [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
qctest.unlink_up(get_param(b{1}, 'Parent'), mdl);
for i = 1:numel(b); set_param(b{i}, 'Value', '-1'); end
end
