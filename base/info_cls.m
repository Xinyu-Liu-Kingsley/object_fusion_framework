classdef info_cls < uint16
% Description: 显示信息的类型
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ %   
    enumeration
        anything(uint16(bin2dec('1111')))
        general(uint16(bin2dec('0001')))               % 一般信息
        lane(uint16(bin2dec('0010')))                  % 车道线相关
        obstacle(uint16(bin2dec('0100')))              % 障碍物相关
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = info_cls.general;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
    
end

