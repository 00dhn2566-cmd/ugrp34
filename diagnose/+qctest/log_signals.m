function log_signals(mdl, which)
% Scope 의 In Bus Element 에 To Workspace 를 물려 신호를 기록한다.
% which : 'pos' (x,y,z) | 'att' (roll,pitch,yaw,z) | 'all'
if nargin < 2; which = 'all'; end
map = { 'In Bus Element',  'real_x',     'pos'; ...
        'In Bus Element1', 'real_y',     'pos'; ...
        'In Bus Element2', 'real_z',     'pos'; ...
        'In Bus Element4', 'real_roll',  'att'; ...
        'In Bus Element3', 'real_pitch', 'att'; ...
        'In Bus Element5', 'real_yaw',   'att'};
% ⚠ 포트 이름 대 신호의 대응은 구운 모델 실측 결과다 (2/4 는 z/roll 로 엇갈려 있다).
%   구조를 바꾸면 여기부터 다시 확인할 것.
scope = [mdl '/Scope'];
for i = 1:size(map, 1)
    if ~strcmp(which, 'all') && ~strcmp(map{i,3}, which); continue; end
    nm  = ['To Workspace ' map{i,2}];
    old = find_system(scope, 'SearchDepth', 1, 'Name', nm);
    if ~isempty(old); delete_block(old{1}); end
    tw = [scope '/' nm];
    add_block('simulink/Sinks/To Workspace', tw, ...
              'VariableName', map{i,2}, 'SaveFormat', 'StructureWithTime');
    add_line(scope, get_param([scope '/' map{i,1}], 'PortHandles').Outport(1), ...
             get_param(tw, 'PortHandles').Inport(1), 'autorouting', 'on');
end
end
