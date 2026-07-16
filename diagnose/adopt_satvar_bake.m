%% [정식 채택 수술 2 - save_system 예외 승인: 사용자 지시 2026-07-15]
%% PosErr Sat X/Y/Z 한계를 리터럴(±0.15) -> 변수 posErrSat(=1.2/kp_position)로 교체 저장.
%% 목적: kp_position 튜닝 시 클램프 자동 연동 (P항 최대 명령 기울기 16.8도 불변식).
%% 절차: 교체 -> save -> 재로드 검증 -> (호출측에서 호버 회귀). 롤백: git checkout Models/...slx

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
fprintf('posErrSat 현재 평가값: %.4f m (0.15여야)\n', posErrSat);
if abs(posErrSat - 0.15) > 1e-12; error('posErrSat != 0.15 - 불변식 확인 필요. 중단'); end

mdl = 'quadcopter_package_delivery';
load_system(mdl);
pc = [mdl '/Maneuver Controller/Position Control'];
p = pc;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end

sats = {'PosErr Sat X', 'PosErr Sat Y', 'PosErr Sat Z'};
for i = 1:numel(sats)
    b = [pc '/' sats{i}];
    if isempty(find_system(pc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', sats{i}))
        error('%s 미발견 - 중단', sats{i});
    end
    old = get_param(b, 'UpperLimit');
    set_param(b, 'UpperLimit', 'posErrSat', 'LowerLimit', '-posErrSat');
    fprintf('%s: %s -> posErrSat\n', sats{i}, old);
end

save_system(mdl);
close_system(mdl, 0);
bdclose('all');

% 재로드 검증
load_system(mdl);
for i = 1:numel(sats)
    b = [pc '/' sats{i}];
    u = get_param(b, 'UpperLimit');
    if ~strcmp(u, 'posErrSat')
        error('재로드 후 %s UpperLimit=%s (posErrSat이어야) - 저장 실패, git 롤백할 것', sats{i}, u);
    end
end
fprintf('>> 영구 채택 완료: Sat X/Y/Z 전부 posErrSat 변수 참조. 다음: 호버 회귀\n');
