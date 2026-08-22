function qc_delay_apply(mdl)
% qc_delay_apply  측정 경로에 전달 지연(Transport Delay) 삽입 — 센서/통신 지연 내성(스펙 T3/R14) 실측용. 메모리 수술, save 금지.
%
% 삽입 지점 (구운 모델 배선 실측, 08-18):
%   자세: Altitude and YPR Control / In Bus Element2 -> Filter Pitch, In Bus Element1 -> Filter Roll  (측정 pitch/roll)   지연 = dly_att_s
%   yaw : 동 / In Bus Element3 -> Filter Yaw                                                                              지연 = dly_att_s
%   z   : 동 / In Bus Element4 -> Filter pz                                                                               지연 = dly_pos_s
%   위치: Position Control / Mux p -> Subtract2 (측정 xyz)                                                                지연 = dly_pos_s
% base 워크스페이스 변수 dly_att_s / dly_pos_s [s] 를 참조 (0 이면 항등에 준함). 이미 삽입돼 있으면 재삽입하지 않음.
% 근거: 실기 예상 — IMU→제어기 1~3 ms + 모터 추력 응답 20~40 ms (자세 경로), VIO 30~100 ms (위치 경로). 시뮬 기본은 이상 센서(0).
spec = { ...
    'Filter Pitch', 1, 'Meas Delay Pitch', 'dly_att_s'; ...
    'Filter Roll',  1, 'Meas Delay Roll',  'dly_att_s'; ...
    'Filter Yaw',   1, 'Meas Delay Yaw',   'dly_att_s'; ...
    'Filter pz',    1, 'Meas Delay Z',     'dly_pos_s'; ...
    'Subtract2',    2, 'Meas Delay Pos',   'dly_pos_s'};
for k = 1:size(spec, 1)
    dstName = spec{k,1}; port = spec{k,2}; dlyName = spec{k,3}; var = spec{k,4};
    hits = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Name', dstName);
    if strcmp(dstName, 'Subtract2')
        hits = hits(contains(hits, 'Position Control'));
    end
    if numel(hits) ~= 1
        error('qc_delay_apply: %s 블록 %d개 (1개 예상) - 구운 모델 맞는지 확인', dstName, numel(hits));
    end
    dst = hits{1};
    parent = get_param(dst, 'Parent');
    if ~isempty(find_system(parent, 'SearchDepth', 1, 'Name', dlyName))
        continue;   % 이미 삽입
    end
    p = parent;
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            if any(strcmp(get_param(p, 'LinkStatus'), {'resolved','inactive'}))
                set_param(p, 'LinkStatus', 'none');
            end
        catch
        end
        p = get_param(p, 'Parent');
    end
    ph = get_param(dst, 'PortHandles');
    ln = get_param(ph.Inport(port), 'Line');
    if ln == -1
        error('qc_delay_apply: %s Inport %d 에 연결된 선이 없음', dstName, port);
    end
    src = get_param(ln, 'SrcPortHandle');
    delete_line(ln);
    blk = [parent '/' dlyName];
    pos = get_param(dst, 'Position');
    add_block('simulink/Continuous/Transport Delay', blk, ...
        'DelayTime', var, 'InitialOutput', '0', 'BufferSize', '65536', ...
        'Position', [pos(1)-90 pos(2)+40 pos(1)-50 pos(2)+70]);
    bph = get_param(blk, 'PortHandles');
    add_line(parent, src, bph.Inport(1), 'autorouting', 'on');
    add_line(parent, bph.Outport(1), ph.Inport(port), 'autorouting', 'on');
end
fprintf('[qc_delay_apply] 측정 지연 삽입: 자세/yaw = dly_att_s, 위치/z = dly_pos_s (base 워크스페이스)\n');
end
