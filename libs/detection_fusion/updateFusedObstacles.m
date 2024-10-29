function [obstacles] = updateFusedObstacles(cameraAssociateFlag,radarAssociateFlag,frameData)
% 更新融合状态，并对目标的运动状态进行组合
% cameraAssociateFlag camera的匹配状态
% radarAssociateFlag  radar的匹配状态
% frameData 该帧的数据
[numC,~] = size(cameraAssociateFlag);
[numR,~] = size(radarAssociateFlag);
% coder.cstructname(frameData, 'FrameData')
obstacles = repmat(obstacle_const.FusedObject, 1, obstacle_const.max_obs_fuse);
% coder.cstructname(obstacle_const.FusedObjecet, 'Obstacles')
cnt = int32(0);
% update FuseObject
CameraObjectList = frameData.CameraFrame.CameraObjectList;
RadarObjectList =  frameData.RadarFrame.RadarObjectList;
% 遍历融合的目标
for j = 1:numC
    % 更新fused camera
    if cameraAssociateFlag(j,1)>0
        cnt = cnt+1;
        obstacles(cnt).fusedCamID = uint8(CameraObjectList(j).ID);
        obstacles(cnt).fusedRadID = uint16(RadarObjectList(cameraAssociateFlag(j,2)).ID);
        obstacles(cnt).y = double(RadarObjectList(cameraAssociateFlag(j,2)).y);           % 取毫米波雷达的纵向距离
        obstacles(cnt).x = double(CameraObjectList(j).x);                                 % 取摄像头的横向距离
        obstacles(cnt).cls = CameraObjectList(j).cls;                                     % 取摄像头的类别
        obstacles(cnt).velo_x = double(RadarObjectList(cameraAssociateFlag(j,2)).velo_x); % 取摄像头的横向速度
        obstacles(cnt).velo_y = double(RadarObjectList(cameraAssociateFlag(j,2)).velo_y); % 取雷达的纵向速度
        obstacles(cnt).ay = double(RadarObjectList(cameraAssociateFlag(j,2)).ay);         % 雷达的纵向加速度
        obstacles(cnt).obj_width = double(CameraObjectList(j).obj_width);
        obstacles(cnt).obj_det_prop = obs_det_prop.fused;                                 % 目标融合状态 0: undefined 1: sole-radar 2: sole-camera 3: fused
        obstacles(cnt).obj_exist_prop = CameraObjectList(j).obj_exist_prop;               %目标存在概率
        obstacles(cnt).obj_dyn_prop = CameraObjectList(j).obj_dyn_prop;
        obstacles(cnt).obj_lane = CameraObjectList(j).obj_lane;
        obstacles(cnt).obj_direction = CameraObjectList(j).obj_direction;
        obstacles(cnt).img_lane = CameraObjectList(j).img_lane;
        obstacles(cnt).x_error = abs(RadarObjectList(cameraAssociateFlag(j,2)).x-CameraObjectList(j).x);
        obstacles(cnt).y_error = abs(RadarObjectList(cameraAssociateFlag(j,2)).y-CameraObjectList(j).y);
        obstacles(cnt).moveType = RadarObjectList(cameraAssociateFlag(j,2)).moveType;
        obstacles(cnt).obsolute_res = RadarObjectList(cameraAssociateFlag(j,2)).obsolute_res;
        obstacles(cnt).det_src = det_src.front;
        obstacles(cnt).flag = uint8(1);
        % 更新camera only
    elseif CameraObjectList(j).flag>uint8(0)
        cnt = cnt+1;
        obstacles(cnt).fusedCamID = uint8(CameraObjectList(j).ID);
        obstacles(cnt).x = double(CameraObjectList(j).x);
        obstacles(cnt).y = double(CameraObjectList(j).y);
        obstacles(cnt).cls = CameraObjectList(j).cls;
        obstacles(cnt).velo_x = double(CameraObjectList(j).velo_x);
        obstacles(cnt).velo_y = double(CameraObjectList(j).velo_y);
        obstacles(cnt).ay = double(CameraObjectList(j).obj_y_arel);  % camera 只支持纵向加速度
        obstacles(cnt).obj_width = double(CameraObjectList(j).obj_width);
        obstacles(cnt).obj_det_prop = obs_det_prop.sole_camera;                        % 目标融合状态 0: undefined 1: sole-radar 2: sole-camera 3: fused
        obstacles(cnt).obj_exist_prop = CameraObjectList(j).obj_exist_prop;  %目标存在概率
        obstacles(cnt).obj_dyn_prop = CameraObjectList(j).obj_dyn_prop;
        obstacles(cnt).obj_lane = CameraObjectList(j).obj_lane;
        obstacles(cnt).obj_direction = CameraObjectList(j).obj_direction;
        obstacles(cnt).img_lane = CameraObjectList(j).img_lane;
        obstacles(cnt).det_src = det_src.front;
        obstacles(cnt).flag = uint8(1);
    end
end
%  更新radar only
for jj = 1:numR
    if radarAssociateFlag(jj,1)<=0  && RadarObjectList(jj).flag> uint8(0)
        cnt = cnt+1;
        obstacles(cnt).fusedRadID = uint16(RadarObjectList(jj).ID);
        obstacles(cnt).x = double(RadarObjectList(jj).x);             % 取毫米波雷达的横向距离
        obstacles(cnt).y = double(RadarObjectList(jj).y);             % 取毫米波雷达的纵向距离
        obstacles(cnt).cls = RadarObjectList(jj).cls;                 % 取毫米波雷达的类别
        obstacles(cnt).velo_x = double(RadarObjectList(jj).velo_x);   % 取毫米波的横向距离
        obstacles(cnt).velo_y = double(RadarObjectList(jj).velo_y);   % 取毫米波的横向速度
        obstacles(cnt).ay = double(RadarObjectList(jj).ay);           % 雷达的纵向加速度
        obstacles(cnt).obj_det_prop = obs_det_prop.sole_radar;        % 目标融合状态 0: undefined 1: sole-radar 2: sole-camera 3: fused
        obstacles(cnt).obj_exist_prop = RadarObjectList(jj).obj_exist_prop;  %目标存在概率
        obstacles(cnt).obj_lane = RadarObjectList(jj).obj_lane;
        obstacles(cnt).det_src = RadarObjectList(jj).det_src;
		obstacles(cnt).moveType = RadarObjectList(jj).moveType;
        obstacles(cnt).obsolute_res = RadarObjectList(jj).obsolute_res;
        obstacles(cnt).flag = uint8(1);
    end
end
