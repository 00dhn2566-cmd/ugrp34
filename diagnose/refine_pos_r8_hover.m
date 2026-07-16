%% 위치 채널 8라운드: 호버 자세 지터 스윕 (관문1 발견 대응)
%% 발견: 24/10.8이 호버 자세 RMS 0.002 -> 0.26도로 퇴행 (16차 지터 킬 무효화).
%% 위치 kd가 호버 측정 잡음을 자세 명령으로 증폭하는 것으로 추정 - kd 의존성 확인.
%% 목적: 호버 지터가 기준선(~0.002도)급으로 유지되는 최대 게인 탐색. 대조군 8/3.2 포함.

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

dt = 0.01; T = 8;
N = round(T/dt) + 1;
tt = (0:N-1)' * dt;
hover = repmat([0 0 1], N, 1);
waypoints = [0 0 1; 1 0 1]';   % 시각화용 (짧은 쌍 거부 회피)
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('waypoints', waypoints);
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(waypoints));
mws.assignin('timespot_spl', tt);
mws.assignin('spline_data', hover);
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

% 후보: [kp kd] - kd 의존성 분리를 위해 같은 kp에서 kd 변화 포함. 첫 행 = 대조군.
pairs = [8 3.2; 12 4.8; 16 6.4; 20 8.0; 24 8.4; 24 9.6; 24 10.8; 22 7.7];
rows = nan(size(pairs,1), 7);
fprintf('===== 위치 8라운드: 호버 자세 지터 스윕 (ki=0.04, 자세 새 게인 하) =====\n');
fprintf('%5s %5s | %10s %8s %8s %8s\n','kp','kd','자세RMSdeg','피크deg','z최저','호버cm');
for a = 1:size(pairs,1)
    kp_position = pairs(a,1); kd_position = pairs(a,2); ki_position = 0.04;
    try
        sim(mdl);
    catch e
        fprintf('%5.1f %5.1f | 시뮬 실패: %s\n', pairs(a,1), pairs(a,2), e.message);
        continue;
    end
    tu = (0:0.005:T)';
    gi2 = @(s) interp1(s.time(:), s.signals.values(:), tu, 'linear', 'extrap');
    xg = gi2(px); zg = gi2(pz);
    pg = rad2deg(gi2(real_pitch)); rg = rad2deg(gi2(real_roll));
    seg = (tu >= 2);                          % 초기 과도 제외
    rmsf = @(v) sqrt(mean((v-mean(v)).^2));
    aRms = sqrt(rmsf(pg(seg))^2 + rmsf(rg(seg))^2);
    aPk  = max(max(abs(pg(seg))), max(abs(rg(seg))));
    zMin = min(zg);
    hcm  = rmsf(xg(seg))*100;
    rows(a,:) = [pairs(a,1), pairs(a,2), 0.04, aRms, aPk, zMin, hcm];
    fprintf('%5.1f %5.1f | %10.4f %8.2f %8.3f %8.3f\n', pairs(a,1), pairs(a,2), aRms, aPk, zMin, hcm);
end
fprintf('(판정: 자세RMS가 대조군(8/3.2)의 ~2배 이내인 최대 게인 채택. kd 의존성 주시)\n');

csvDir = fullfile(modelDir, 'diagnose', 'results');
if ~exist(csvDir, 'dir'); mkdir(csvDir); end
Tb = array2table(rows, 'VariableNames', ...
    {'kp','kd','ki','att_rms_deg','att_peak_deg','z_min_m','hover_cm'});
writetable(Tb, fullfile(csvDir, 'refine_pos_r8_hover.csv'));
fprintf('CSV 저장: %s\n', fullfile(csvDir, 'refine_pos_r8_hover.csv'));
