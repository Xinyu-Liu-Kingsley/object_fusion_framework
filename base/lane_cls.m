classdef lane_cls < uint8
% Description: 车道线类型
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
%         unknown(0)          % 未知
        singleDash(0)       % 单虚线
        singleSolid(1)      % 单实线
        lDashrSolid(2)      % 左虚右实
        lSolidrDash(3)      % 左实右虚
        doubleDash(4)       % 双虚线
        doubleSolid(5)      % 双实线
        curb(6)             % 车辆可行驶区域的最外沿与路基的明显交界线
        singleDashSP(7)     % 特殊线型，如可变导向车道线，减速标线，特殊的公交车道等，中心的主线为虚线
        singleSolidSP(8)    % 特殊线型，如可变导向车道线，减速标线，特殊的公交车道等，中心的主线为实线
        unknown(10)
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = lane_cls.unknown;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
    
end