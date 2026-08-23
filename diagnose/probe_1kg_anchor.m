%% 1 kg 앵커 실측 — 구운 모델에 하드코딩된 상수를 읽는다 (2026-08-23)
%%
%% 왜 필요한가: 08-18 0 kg 재튜닝이 건드린 항목 중 `biasChassis`/`filtPz`/`altCmdSat`/
%% `filtM_att_meas`/`tiltLimit`/`nl_gmax` 는 이 머신의 parameters.m 에 **없다**
%% (08-18 판이 동기 안 됨). 질량 선형 보간을 하려면 1 kg 쪽 앵커가 있어야 하는데,
%% 추측하지 말고 모델에서 직접 읽는다.
modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir,'Scripts_Data'), fullfile(modelDir,'Models'), ...
        fullfile(modelDir,'Libraries'));
addpath(genpath(fullfile(modelDir,'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
if bdIsLoaded(mdl); close_system(mdl,0); end
load_system(mdl);

want = { 'Bias Chassis','Bias'; 'Filter pz','Denominator'; ...
         'Alt Cmd Sat','UpperLimit'; 'Filter Pitch','Denominator'; ...
         'Filter Roll','Denominator'; 'Pitch Limit','UpperLimit'; ...
         'Roll Limit','UpperLimit' };
fprintf('\n==== 구운 모델 하드코딩 상수 (1 kg 앵커) ====\n');
for k = 1:size(want,1)
    b = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Name',want{k,1});
    if numel(b) ~= 1
        fprintf('%-16s : 블록 %d개 (건너뜀)\n', want{k,1}, numel(b)); continue;
    end
    fprintf('%-16s %-12s = %s\n', want{k,1}, want{k,2}, get_param(b{1}, want{k,2}));
end

fprintf('\n==== parameters.m 계산값 (m_pkg_now = %g) ====\n', m_pkg_now);
nm = {'sA_mass','sZ_mass','kp_attitude','ki_attitude','kd_attitude','filtD_attitude', ...
      'limit_attitude','kp_position','kd_position','ki_position','posErrSat', ...
      'filtM_position','filtD_position','pos2attitude','kp_altitude','ki_altitude', ...
      'kd_altitude','filtM_altitude','filtD_altitude','limit_altitude', ...
      'kp_yaw','ki_yaw','kd_yaw','limit_yaw','sT','sQ'};
for k = 1:numel(nm)
    if evalin('base', sprintf("exist('%s','var')", nm{k}))
        v = evalin('base', nm{k});
        fprintf('%-16s = %s\n', nm{k}, mat2str(v, 6));
    else
        fprintf('%-16s = (없음)\n', nm{k});
    end
end
close_system(mdl, 0);
