classdef info_spec < uint8
% Description: 显示信息的规格等级
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    
    enumeration
        general (uint8(bin2dec('0001')))               % 一般信息
        debug   (uint8(bin2dec('0010')))               % 调试信息
        warning (uint8(bin2dec('0100')))               % 警告信息
        error   (uint8(bin2dec('1000')))               % 错误信息
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = info_spec.general;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
    
end

