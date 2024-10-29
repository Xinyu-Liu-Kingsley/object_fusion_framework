
% Description: 计算目标的TTC
% Author : Wei Zeyi
% log:
% ...
%****************************************************************%
function obs_XY = get_TTC(obs_XY) 
global g_ego_status g_ego_params;
  if obs_XY.det_src==det_src.left_forward || obs_XY.det_src==det_src.left_backward ...
     || obs_XY.det_src==det_src.right_backward || obs_XY.det_src==det_src.right_forward
      if obs_XY.velo_y~=0 && abs(g_ego_status.yawrate)<30 || abs(g_ego_status.SW_angle)<5 %直行TTC计算
         obs_XY.TTC = obs_XY.y/-obs_XY.velo_y;
      elseif g_ego_status.yawrate>30 && g_ego_status.SW_angle<-5   %右转弯TTC计算
%          obs_XY.TTC = atan((obs_XY.y+6)/(2.4*g_ego_params.length-(obs_XY.x-1.2)))*180/pi/g_ego_status.yawrate;
           obs_XY.TTC = atan(sqrt((2.4*g_ego_params.length)^2-(2.4*g_ego_params.length-(obs_XY.x-1.2))^2)/(2.4*g_ego_params.length-(obs_XY.x-1.2)))*180/pi/g_ego_status.yawrate;
      end
  end
end