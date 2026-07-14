%% 진자 확정 3탄 (사용자 지시): 짐 사실상 0kg(1mg) - 1.75Hz 소멸 재확인 + z 거동 추적
%% (딱 0은 Simscape 강체 질량 제약으로 불가 -> 밀도 x1e-6 = 1mg)
%% 기준(1kg): pitch RMS 3.94/4.00, 1.75Hz, 무감쇠. 2kg: 1.75Hz 불변(진자 지문).
%% 1g(이전): 1.75Hz 소멸했으나 z 0.15 이탈로 오염 - 이번엔 z 시계열 동시 추적.
%% 규칙: 구운 .slx 무수정(메모리 수술만), save_system 금지.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));

VMAX = 2.0; AMAX = 2.0; JMAX = 10.0;
dt = 0.01; T = 12; tStep = 3; A = 1.0;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
tau = min(max((tt-tStep)/0.67,0),1);
xk = A * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
smKill = traj_smoother(tt, [xk, zeros(N,1), ones(N,1)], VMAX, AMAX, JMAX);

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

waypoints = [0 0 1; A 0 1]';
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', smKill);
mws.assignin('spline_yaw', zeros(N,1));
set_param(mdl, 'StopTime', num2str(T));

scope = [mdl '/Scope'];
sigMap = {'In Bus Element','px'; 'In Bus Element2','pz'; ...
          'In Bus Element4','real_roll'; 'In Bus Element3','real_pitch'};
for i = 1:size(sigMap,1)
    twName = ['To Workspace ' sigMap{i,2}];
    oldTw = find_system(scope, 'SearchDepth', 1, 'Name', twName);
    if ~isempty(oldTw); delete_block(oldTw{1}); end
    twBlk = [scope '/' twName];
    add_block('simulink/Sinks/To Workspace', twBlk, 'VariableName', sigMap{i,2}, 'SaveFormat', 'StructureWithTime');
    srcPh = get_param([scope '/' sigMap{i,1}], 'PortHandles');
    twPh  = get_param(twBlk, 'PortHandles');
    add_line(scope, srcPh.Outport(1), twPh.Inport(1), 'autorouting', 'on');
end

pkgDensity = pkgDensity * 1e-6;   % 1kg -> 1mg (사실상 0)
fprintf('===== 짐 1mg (사실상 0kg) 비행 =====\n');
sim(mdl);

tu = (0:0.005:T)';
xg = interp1(px.time(:), px.signals.values(:), tu, 'linear', 'extrap');
pg = rad2deg(interp1(real_pitch.time(:), real_pitch.signals.values(:), tu, 'linear', 'extrap'));
rg = rad2deg(interp1(real_roll.time(:), real_roll.signals.values(:), tu, 'linear', 'extrap'));
zg = interp1(pz.time(:), pz.signals.values(:), tu, 'linear', 'extrap');
xrg = interp1(tt, smKill(:,1), tu, 'linear', 'extrap');
seg = @(t1,t2) (tu>=t1 & tu<t2);
rmsf = @(v) sqrt(mean((v-mean(v)).^2));
r1 = rmsf(pg(seg(6,9))); r2 = rmsf(pg(seg(9,12)));
pgP = pg(seg(6,12)); pgP = pgP - mean(pgP);
freq = sum(abs(diff(sign(pgP)))>0)/2/6;

fprintf('\n  시각 | 기준x  실제x |  P      R   |  z\n');
for ct = [0.5 1 2 3 3.5 4 4.5 5 6 7 8 9 10 11 12]
    [~,i2] = min(abs(tu-ct));
    fprintf('  t=%4.1f | %5.2f %6.3f | %+6.2f %+6.2f | %5.3f\n', tu(i2), xrg(i2), xg(i2), pg(i2), rg(i2), zg(i2));
end
fprintf('\n  도착 후: pitch RMS 6~9s %.3f도 / 9~12s %.3f도 / 주파수 %.2fHz / roll RMS %.3f도\n', ...
    r1, r2, freq, rmsf(rg(seg(6,12))));
fprintf('  (기준 1kg: 3.940/4.001도, 1.75Hz -> 짐 0에서 이 라인이 사라지면 진자 최종 확정)\n');
fprintf('  x오차 RMS %.2fcm | z 범위 %.3f~%.3fm\n', ...
    sqrt(mean((xrg(seg(6,12))-xg(seg(6,12))).^2))*100, min(zg), max(zg));
