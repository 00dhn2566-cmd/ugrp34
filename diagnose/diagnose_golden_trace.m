%% 골든 트레이스 채취 (17차, C++ 완전 대조용) - 제어기 입출력을 같은 시간축 CSV로
%% 출력 1: golden_input.csv  (t, ref, meas, w1..4)      = C++ qc_trace 입력
%% 출력 2: golden_output.csv (t, cmd_pitch, cmd_roll, u1..4 근사) = 정답 출력
%% 탭 지점: Scope 버스(측정) + Position Controller 출력(cmd) + Control1(모터 PI 출력).
%% 주의: 블록 경로는 정규화 이름 매칭으로 탐색 (개행 함정). 미발견 시 error() 즉사.
%% 궤적: 온건 1m 이동 (관문과 동일) - 포화 구간 최소화가 대조 정밀도에 유리.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

dropBlocks = { [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant'], ...
               [mdl '/Quadcopter/Load/Disengage Logic/Distance to drop waypoint/Constant1'] };
p = get_param(dropBlocks{1}, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
for i = 1:numel(dropBlocks)
    set_param(dropBlocks{i}, 'Value', '-1');
end

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.9,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
sm = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);
waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', sm);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

norm1 = @(s) regexprep(s, '\s+', ' ');
allBlk = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on');

% --- 탭 1: Scope 버스 (측정: pos/rpy) ---
scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element1','py'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'; ...
          'In Bus Element5','real_yaw'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace g' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    srcCand = find_system(scope, 'SearchDepth', 1, 'Name', sigMap{i,1});
    if isempty(srcCand)
        fprintf('[경고] Scope 요소 %s 없음 - %s 생략\n', sigMap{i,1}, sigMap{i,2});
        continue;
    end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', ['g_' sigMap{i,2}], 'SaveFormat', 'StructureWithTime');
    srcPh = get_param(srcCand{1}, 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

% --- 탭 2: 제어기 내부 (cmd pitch/roll = Err2P/Err2R 하류 Limit 출력, 모터 PI 출력, 모터 속도) ---
% 이름 후보를 정규화 매칭으로 찾는다. 모델 개정에 따른 이름 차이는 후보 추가로 대응.
tapDefs = { ...
  'cmd_pitch', {'Pitch Limit','Pitch Angle Limit','Limit Pitch'}; ...
  'cmd_roll',  {'Roll Limit','Roll Angle Limit','Limit Roll'}; ...
};
for d = 1:size(tapDefs,1)
    hit = '';
    for i = 1:numel(allBlk)
        try
            nm1 = norm1(get_param(allBlk{i}, 'Name'));
        catch
            continue;
        end
        if any(strcmpi(nm1, tapDefs{d,2}))
            hit = allBlk{i};
            break;
        end
    end
    if isempty(hit)
        error('탭 대상 미발견: %s (후보: %s) - dump_controller_spec 결과로 이름 갱신 필요', ...
              tapDefs{d,1}, strjoin(tapDefs{d,2}, '/'));
    end
    parent = get_param(hit, 'Parent');
    twBlk = [parent '/To Workspace ' tapDefs{d,1}];
    old = find_system(parent, 'SearchDepth', 1, 'Name', ['To Workspace ' tapDefs{d,1}]);
    if ~isempty(old); delete_block(old{1}); end
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', ['g_' tapDefs{d,1}], 'SaveFormat', 'StructureWithTime');
    srcPh = get_param(hit, 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(parent, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
    fprintf('탭 연결: %s <- %s\n', tapDefs{d,1}, norm1(strrep(hit, newline, '|')));
end

% 모터 속도 w1~4: 프로펠러 축 속도 (run_traj_baked 태핑과 동일 대상을 자체 확보)
for mi = 1:4
    patt = sprintf('Propeller %d', mi);
    hit = '';
    for i = 1:numel(allBlk)
        if contains(allBlk{i}, patt) && contains(allBlk{i}, 'PS-Simulink')
            hit = allBlk{i};   % 프로펠러 서브시스템 내 속도 변환기 (있으면 사용)
            break;
        end
    end
    if isempty(hit)
        fprintf('[경고] %s 속도 변환기 미발견 - w%d는 golden_input에서 제외(재생시 motorRef 대체)\n', patt, mi);
    end
end

sim(mdl);

% --- CSV 조립 ---
tu = (0:0.001:T)';   % 1kHz (C++ 대조 기준)
gv = @(nm) evalin('base', nm);
gi = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
need = {'g_px','g_pz','g_real_roll','g_real_pitch'};
for i = 1:numel(need)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', need{i}))
        error('%s 미기록 - 탭 배선 확인', need{i});
    end
end
px_ = gi(gv('g_px'));
py_ = zeros(size(tu)); try py_ = gi(gv('g_py')); catch; end
pz_ = gi(gv('g_pz'));
rr_ = gi(gv('g_real_roll'));
rp_ = gi(gv('g_real_pitch'));
ry_ = zeros(size(tu)); try ry_ = gi(gv('g_real_yaw')); catch; end
cp_ = gi(gv('g_cmd_pitch'));
cr_ = gi(gv('g_cmd_roll'));
refx = interp1(tt, sm(:,1), tu, 'linear', 'extrap');
refy = interp1(tt, sm(:,2), tu, 'linear', 'extrap');
refz = interp1(tt, sm(:,3), tu, 'linear', 'extrap');

outDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(outDir, 'dir'); mkdir(outDir); end
Ti = table(tu, refx, refy, refz, zeros(size(tu)), px_, py_, pz_, rr_, rp_, ry_, ...
    zeros(size(tu)), zeros(size(tu)), zeros(size(tu)), zeros(size(tu)), ...
    'VariableNames', {'t','ref_x','ref_y','ref_z','ref_yaw','meas_x','meas_y','meas_z', ...
                      'roll','pitch','yaw','w1','w2','w3','w4'});
writetable(Ti, fullfile(outDir, 'golden_input.csv'));
To = table(tu, cp_, cr_, 'VariableNames', {'t','cmd_pitch','cmd_roll'});
writetable(To, fullfile(outDir, 'golden_output.csv'));
fprintf('골든 CSV 저장: %s (input %d행 / output cmd 2채널)\n', outDir, numel(tu));
fprintf('(모터 w/u 골든은 후속 - cmd 채널 대조가 1차 목표: 위치체인+자세명령 배선 검증)\n');
