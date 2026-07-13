%% 지금까지 "act_yaw"라는 이름으로 로깅해온 신호가 실제로 어떤 In Bus Element에서
%% 왔는지 배선을 직접 추적해서 확인. (Scope의 In Bus Element 번호가 예상과 다름을
%% 발견했음: In Bus Element11 = Prop1.w 로 확인됨. act_yaw가 정말 이걸 참조하는지 검증.)

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');

mdl = 'quadcopter_package_delivery';
load_system(mdl);

scope = [mdl '/Scope'];
targets = find_system(scope, 'SearchDepth', 1, 'RegExp', 'on', 'Name', '^To Workspace (act_roll|act_pitch|act_yaw)$');

for i = 1:numel(targets)
    blk = targets{i};
    fprintf('=== %s ===\n', blk);
    ph = get_param(blk, 'PortHandles');
    lineH = get_param(ph.Inport(1), 'Line');
    if lineH == -1
        fprintf('  연결 안됨\n');
        continue
    end
    srcPortH = get_param(lineH, 'SrcPortHandle');
    srcBlk = get_param(srcPortH, 'Parent');
    fprintf('  소스 블록 = %s\n', srcBlk);
    try
        el = get_param(srcBlk, 'Element');
        fprintf('  Element = %s\n', el);
    catch
        fprintf('  (Element 속성 없음 - BusSelector/다른 타입일 수 있음)\n');
        try
            fprintf('  BlockType = %s\n', get_param(srcBlk, 'BlockType'));
        catch
        end
    end
end
