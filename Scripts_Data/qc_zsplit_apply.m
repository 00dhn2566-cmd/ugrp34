function qc_zsplit_apply(mdl)
%QC_ZSPLIT_APPLY PosErr Sat Z를 posErrSatZ 변수로 분기 (메모리 수술, 18차 z분리)
%   구운 .slx는 PosErr Sat X/Y/Z 모두 posErrSat를 참조한다. agile 프로파일은
%   x/y와 z 클램프가 달라야 하므로(z분리) 시뮬 전에 이 함수를 호출해 Z만
%   posErrSatZ를 보게 바꾼다. save_system 금지 규칙 준수 - 메모리에서만 유효.
%   precision/balanced에서는 posErrSatZ == posErrSat 라 호출해도 무해 (거동 동일).
%   parameters.m이 posErrSatZ를 항상 정의하므로 어떤 프로파일에서든 안전.
%
%   사용: load_system(mdl) 후, sim() 전에 qc_zsplit_apply(mdl)

satZ = [mdl '/Maneuver Controller/Position Control/PosErr Sat Z'];
p = get_param(satZ, 'Parent');
while ~isempty(p) && ~strcmp(p, mdl)
    try
        if strcmp(get_param(p, 'LinkStatus'), 'resolved')
            set_param(p, 'LinkStatus', 'inactive');
        end
    catch
    end
    p = get_param(p, 'Parent');
end
set_param(satZ, 'UpperLimit', 'posErrSatZ', 'LowerLimit', '-posErrSatZ');
end
