%% Quadcopter/Load 결합 구조 조사 (시뮬 없음): 짐이 어떤 조인트/유연 요소로 물려 있나
%% 배경: 1.75Hz 무감쇠 pitch 왕복 = 짐 진자로 동역학 확정 (질량 불변 주파수).
%% 진자 길이 예측 8.1cm - 결합부 지오메트리와 대조.
modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
addpath(fullfile(modelDir, 'Models'));
addpath(fullfile(modelDir, 'Libraries'));
addpath(genpath(fullfile(modelDir, 'CAD')));
load_system('quadcopter_library');
quadcopter_package_parameters;
mdl = 'quadcopter_package_delivery';
load_system(mdl);

loadSys = [mdl '/Quadcopter/Load'];
blks = find_system(loadSys, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
fprintf('===== %s 하위 %d블록: 조인트/스프링/힘 요소 =====\n', loadSys, numel(blks));
for i = 1:numel(blks)
    if strcmp(blks{i}, loadSys); continue; end
    nm = strtrim(regexprep(blks{i}, '\s+', ' '));
    rb = ''; mt = ''; bt = '';
    try; rb = get_param(blks{i}, 'ReferenceBlock'); catch; end
    try; mt = get_param(blks{i}, 'MaskType'); catch; end
    try; bt = get_param(blks{i}, 'BlockType'); catch; end
    key = lower([rb ' ' mt ' ' nm]);
    if contains(key, 'joint') || contains(key, 'spring') || contains(key, 'bushing') || ...
       contains(key, 'force') || contains(key, 'damper') || contains(key, 'gimbal') || ...
       contains(key, 'spherical') || contains(key, 'revolute') || contains(key, 'prismatic') || ...
       contains(key, 'planar') || contains(key, '6-dof') || contains(key, 'dof')
        fprintf('[%s]\n   ref: %s | mask: %s | type: %s\n', nm, rb, mt, bt);
        % 조인트면 스프링/댐퍼 파라미터도 시도
        pl = {'PxPrimitiveSpringStiffness','PxPrimitiveDamperCoefficient', ...
              'RxPrimitiveSpringStiffness','RxPrimitiveDamperCoefficient', ...
              'SPrimitiveSpringStiffness','SPrimitiveDamperCoefficient'};
        for pi2 = 1:numel(pl)
            try
                v = get_param(blks{i}, pl{pi2});
                fprintf('   %s = %s\n', pl{pi2}, v);
            catch
            end
        end
    end
end
fprintf('\n(위 목록에서 짐-기체 결합 조인트의 자유도/강성/감쇠 확인. 회전 자유도 + 저감쇠 = 진자 구조 확정)\n');
