classdef lane_index < uint8
% Description: 车道线编号
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        r_line(0)     	% 右车道线
        l_line(1)    	% 左车道线
        rr_line(2)      % 右侧相邻车道线
        ll_line(3)      % 左侧相邻车道线
        others(5)       % 未知
    end

    methods(Static)
        function y = getDefaultValue()
            y = lane_index.others;
        end
        function y = addClassNameToEnumNames()
            y = true;
        end
    end

end

