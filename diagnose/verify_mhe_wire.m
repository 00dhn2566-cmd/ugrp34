%% MHE 배선·컴파일 확인 (스크립트 — 파라미터가 base 에 있어야 한다)

%VERIFY_MHE_WIRE  MHE 삽입이 배선·컴파일까지 되는지 (시뮬은 안 돌린다).
%
%   문서가 경고한 함정 셋을 여기서 건진다:
%     1) 대수 루프 — S-function direct feedthrough 를 껐는지
%     2) 포트 폭 3 — 리드 보상기가 이것 때문에 죽었다 (Transfer Fcn 은 SISO)
%     3) Goto/From 태그 가시성 — 두 서브시스템의 공통 조상이 모델뿐이라 global 이어야
%
%   ★ save_system 금지. 메모리에서만 고치고 저장 없이 닫는다.

here = fileparts(mfilename('fullpath'));   %#ok<*NASGU>
root = fullfile(here, '..');
addpath(fullfile(root,'Scripts_Data'), fullfile(root,'Models'), ...
        fullfile(root,'Libraries'), genpath(fullfile(root,'CAD')));

mdl = 'quadcopter_package_delivery';
quadcopter_package_parameters;
if bdIsLoaded(mdl); close_system(mdl, 0); end
load_system(mdl);
% ★ save_system 금지 — 끝에서 저장 없이 닫는다

qctest.disable_drop(mdl);
qc_0kg_tuned_apply(mdl);                   % ★ 원래 채택 게인 그대로 (튜닝 안 건드림)
assignin('base', 'dly_att_s', 0.005);
assignin('base', 'dly_pos_s', 0.060);      % 60 ms — 예측기가 값어치를 하는 구간
% 궤적이 없으면 Cartesian Joint 의 PxPositionTargetValue 가 waypoints 를 못 찾아
% 컴파일이 죽는다 (MHE 와 무관한 실패). 3 m 이동을 심어둔다.
qctest.raised_cos_move(mdl, 12.0, 3.0, 1.0, 3.0, 3.927);
qc_delay_apply(mdl);
qc_mhe_apply(mdl);

fprintf('\n--- 삽입 결과 확인 ---\n');
for nm = {'MHE', 'MHE Att Mux', 'MHE From qcMheRoll', 'qcMheRoll', 'qcMhePitch', 'qcMheYaw'}
    h = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', nm{1});
    fprintf('  %-22s %d개\n', nm{1}, numel(h));
end

fprintf('\n--- 컴파일 (대수 루프·포트 폭이 여기서 걸린다) ---\n');
try
    eval([mdl '([],[],[],''compile'');']);
    hm = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Name','MHE');
    ph = get_param(hm{1}, 'CompiledPortWidths');
    fprintf('  MHE 포트 폭: 입력 [%s], 출력 [%s]\n', ...
            num2str(ph.Inport), num2str(ph.Outport));
    eval([mdl '([],[],[],''term'');']);
    fprintf('  컴파일 **성공**\n');
catch ME
    try
        eval([mdl '([],[],[],''term'');']);
    catch
    end
    fprintf(2, '  컴파일 실패: %s\n', ME.message);
    % 최상위 메시지는 뭉뚱그려 나온다 — 진짜 원인은 cause 사슬에 있다.
    ce = ME; d = 0;
    while ~isempty(ce.cause)
        ce = ce.cause{1}; d = d + 1;
        fprintf(2, '    [%d] %s\n', d, ce.message);
        if d > 6; break; end
    end
    fprintf(2, '    id: %s\n', ME.identifier);
end

close_system(mdl, 0);   % 저장 없이
