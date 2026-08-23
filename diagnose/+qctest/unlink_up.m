function unlink_up(blk, mdl)
% 라이브러리 링크 해제 — blk 에서 mdl 까지 거슬러 올라가며 편집 가능하게 만든다.
% 구운 모델 메모리 수술의 공통 전제. save_system 금지 규칙은 호출자가 지킨다.
p = blk;
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if any(strcmp(get_param(p, 'LinkStatus'), {'resolved', 'inactive'}))
            set_param(p, 'LinkStatus', 'none');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
end
