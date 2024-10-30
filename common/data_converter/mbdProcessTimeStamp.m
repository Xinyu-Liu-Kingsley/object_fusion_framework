function [frameData] = mbdProcessTimeStamp(frameData)
% Description: 雷达和摄像头时间戳对齐
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
TimeStamp_camera = double(frameData.CameraFrame.time_ns);
TimeStamp_radar = double(frameData.RadarFrame.time_ns);
   if abs(TimeStamp_radar-TimeStamp_camera)<=10  % 雷达和摄像头时间相差不超过10ms，不做处理
       return;
   elseif TimeStamp_radar-TimeStamp_camera>10    % 获得的雷达数据时间晚,相机目标纵向位置通过匀速模型进行补偿
       TimeGap = (TimeStamp_radar-TimeStamp_camera)/1000;
       for i=1:obstacle_const.max_obs_camera
%            frameData.CameraFrame.CameraObjectList(i).x = frameData.CameraFrame.CameraObjectList(i).x + frameData.CameraFrame.CameraObjectList(i).velo_x*TimeGap;
           frameData.CameraFrame.CameraObjectList(i).x = frameData.CameraFrame.CameraObjectList(i).x ;
           frameData.CameraFrame.CameraObjectList(i).y = frameData.CameraFrame.CameraObjectList(i).y + frameData.CameraFrame.CameraObjectList(i).velo_y*TimeGap;
       end
   elseif TimeStamp_camera-TimeStamp_radar>10   % 获得的摄像头数据时间晚，雷达目标纵向位置通过匀加速模型进行补偿，纵向速度通过加速度进行补偿
       TimeGap = (TimeStamp_camera-TimeStamp_radar)/1000;
       for i=1:obstacle_const.max_obs_radar
           frameData.RadarFrame.RadarObjectList(i).x = frameData.RadarFrame.RadarObjectList(i).x + frameData.RadarFrame.RadarObjectList(i).velo_x*TimeGap;%+1/2*frameData.RadarFrame.RadarObjectList(i).ax*TimeGap*TimeGap;
           frameData.RadarFrame.RadarObjectList(i).y = frameData.RadarFrame.RadarObjectList(i).y + frameData.RadarFrame.RadarObjectList(i).velo_y*TimeGap+1/2*frameData.RadarFrame.RadarObjectList(i).ay*TimeGap*TimeGap;
%            frameData.RadarFrame.RadarObjectList(i).velo_x = frameData.RadarFrame.RadarObjectList(i).velo_x + frameData.RadarFrame.RadarObjectList(i).ax*TimeGap;
           frameData.RadarFrame.RadarObjectList(i).velo_y = frameData.RadarFrame.RadarObjectList(i).velo_y + frameData.RadarFrame.RadarObjectList(i).ay*TimeGap;
       end
   end
end

