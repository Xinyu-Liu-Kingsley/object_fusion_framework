classdef sensor_type < uint8
% Description: 检测结果来源
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ %    
    enumeration
      unknown_sensor_type(0)    % 未知传感器类型
      short_range_radar(1)      % 短距雷达
      long_range_radar(2)       % 长距雷达
      monocular_camera(3)       % 单目摄像头
      stereo_camera(4)          % 深度摄像头
      lidar(5)                  % 激光雷达
      ultrasonic(6)             % 超声波
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = sensor_type.unknown_sensor_type;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

