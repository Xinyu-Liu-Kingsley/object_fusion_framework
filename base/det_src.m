classdef det_src < uint8

% Description: % 检测结果来源
% Author :  Shane Liu
% Modify Note: 
% **************************************************************** %    
    enumeration
        unknown(0)          % 未知
%         camera(1)     	    % 摄像头
%         forwardRadar(2)     % 前向雷达
%         frontLeftRadar(3)   % 左前角雷达
%         frontRightRadar(4)  % 右前角雷达
%         rearLeftRadar(5)    % 左后角雷达
%         rearRightRadar(6)   % 右后角雷达
        front(1)            % 前向
        left_forward (2)    % 左前
        left(3)             % 左侧
        left_backward(4)    % 左后
        rear(5)             % 后向
        right_backward(6)   % 右后
        right(7)            % 右向
        right_forward(8)    % 右前
        panoramic(9)        % 环视
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = det_src.unknown;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

