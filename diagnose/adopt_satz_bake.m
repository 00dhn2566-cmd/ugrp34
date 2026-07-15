%% [정식 채택 수술 - save_system 예외 승인: 사용자 지시 2026-07-15]
%% PosErr Sat Z(±0.15)를 Position Control에 영구 삽입해 .slx에 저장한다.
%% 근거(§W): z 오차 무클램프 누설 = 발산 메커니즘 ③. Sat Z로 yaw 오염 36→11도 실증.
%% ±0.15는 X/Y와 동급 - 정상 비행에서 z 오차는 mm급이라 평시 미발동 (최후방 안전핀).
%% 절차: 백업 완료 상태에서 실행 -> 삽입 -> save_system -> 재로드 검증.
%% 실패 시 롤백: git checkout Models/quadcopter_package_delivery.slx

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
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

% 이미 있으면 중단 (이중 삽입 방지)
if ~isempty(find_system(pc, 'SearchDepth', 1, 'Name', 'PosErr Sat Z'))
    error('PosErr Sat Z가 이미 존재 - 수술 불필요. 확인 후 재실행');
end

dmx = [pc '/PosErr Demux'];
mux = [pc '/PosErr Mux'];
dph = get_param(dmx, 'PortHandles');
l = get_param(dph.Outport(3), 'Line');
if l == -1; error('PosErr Demux out3 라인 없음 - 배선 예상 불일치. 중단'); end
delete_line(l);
satZ = [pc '/PosErr Sat Z'];
add_block('simulink/Discontinuities/Saturation', satZ, 'UpperLimit', '0.15', 'LowerLimit', '-0.15');
zph = get_param(satZ, 'PortHandles');
mph = get_param(mux, 'PortHandles');
add_line(pc, dph.Outport(3), zph.Inport(1), 'autorouting', 'on');
add_line(pc, zph.Outport(1), mph.Inport(3), 'autorouting', 'on');

% 삽입 검증 (저장 전): 연결 확인
lz1 = get_param(zph.Inport(1), 'Line');
lz2 = get_param(zph.Outport(1), 'Line');
if lz1 == -1 || lz2 == -1; error('Sat Z 배선 불완전 - 저장 중단'); end
fprintf('삽입 완료: PosErr Demux out3 -> [PosErr Sat Z ±0.15] -> PosErr Mux in3\n');

save_system(mdl);
fprintf('save_system 완료\n');
close_system(mdl, 0);
bdclose('all');

% 재로드 검증: 새로 연 파일에 블록이 실재하는지
load_system(mdl);
hit = find_system(pc, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', 'PosErr Sat Z');
if isempty(hit)
    error('재로드 후 PosErr Sat Z 미발견 - 저장 실패. git checkout으로 롤백할 것');
end
fprintf('재로드 검증 통과: %s (Upper=%s, Lower=%s)\n', ...
    strtrim(regexprep(hit{1}, '\s+', ' ')), get_param(hit{1},'UpperLimit'), get_param(hit{1},'LowerLimit'));
fprintf('>> 영구 채택 완료. 다음: 회귀 검증 (verify_hover + 성형 이동)\n');
