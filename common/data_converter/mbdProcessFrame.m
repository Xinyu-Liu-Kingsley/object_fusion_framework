function [frameData] = mbdProcessFrame(frameData)
% Description: 
%   1.筛选雷达和摄像头数据，只输出本车道和旁车道数据
%   2.将雷达的极坐标转成笛卡尔坐标系
%   3.将雷达和摄像头的ID都加1，避免ID为0的目标对目标跟踪的影响
% Author :  Shane Liu
% **************************************************************** % 

coder.cstructname(frameData, 'FrameData')
coder.cstructname(frameData.CameraFrame,'cameraFrame')
coder.cstructname(frameData.RadarFrame,'radarFrame')
coder.cstructname(frameData.CameraFrame.CameraObjectList,'cameraObject')
coder.cstructname(frameData.CameraFrame.LaneList,'Lane')
coder.cstructname(frameData.RadarFrame.RadarObjectList,'radarObject')
% coder.cstructname(obstacle_const.)
global g_ego_status
%% 车道线方程
C_left = zeros(1,3);  % 左车道线方程系数
C_right = zeros(1,3); % 右车道线方程系数
C_left_range = 0;     % 左车道
C_right_range = 0;    % 右车道
lane_list = frameData.CameraFrame.LaneList;
rightLane_valid = 0;
leftLane_valid = 0;
for i = 1:obstacle_const.max_lane_num
    if lane_list(i).flag> uint8(0)
        if lane_list(i).lane_locaiotn == lane_index.l_line  % 左车道
            C_left = [lane_list(i).C3,lane_list(i).C2,lane_list(i).C1,lane_list(i).C0];
            C_left_range = lane_list(i).view_range;
            if lane_list(i).C0 ~=0
                leftLane_valid = 1;
            end
        elseif lane_list(i).lane_locaiotn == lane_index.r_line  % 右车道
            C_right = [lane_list(i).C3,lane_list(i).C2,lane_list(i).C1,lane_list(i).C0];
            C_right_range = lane_list(i).view_range;
            if lane_list(i).C0 ~=0
                rightLane_valid = 1;
            end
        end
    end
end
%% --------------------------- 处理Radar数据-----------------------------------------------
radarObjects = repmat(obstacle_const.RadarObject,1,obstacle_const.max_obs_radar);
cntRadar = 0;
for i = 1:obstacle_const.max_obs_radar
    if frameData.RadarFrame.RadarObjectList(i).flag<uint8(1)
        continue
    end
    radarObject = frameData.RadarFrame.RadarObjectList(i);
    if frameData.RadarFrame.RadarObjectList(i).det_src == det_src.front
        %宝隆雷达信号处理
        if radarObject.cls==1
            radarObject.cls = obstacle_cls.pedestrian;
        elseif radarObject.cls==2
            radarObject.cls = obstacle_cls.bicycle;
        elseif radarObject.cls==3
            radarObject.cls = obstacle_cls.car;
        elseif radarObject.cls==4
            radarObject.cls = obstacle_cls.truck;
        elseif radarObject.cls==5
            radarObject.cls = obstacle_cls.leftGuard;
        elseif radarObject.cls==6
            radarObject.cls = obstacle_cls.rightGuard;
        else
            radarObject.cls = obstacle_cls.unknown;
        end
        angle = deg2rad(radarObject.obj_azimuth); % 角度转成弧度
        radarObject.x = radarObject.obj_range*sin(angle); % 横向位置
        radarObject.y = radarObject.obj_range*cos(angle); % 纵向位置
    else
        %角雷达信号处理
        if radarObject.cls==4
            radarObject.cls = obstacle_cls.pedestrian;
        elseif radarObject.cls==5
            radarObject.cls = obstacle_cls.tricycle;
        elseif radarObject.cls==6
            radarObject.cls = obstacle_cls.car;
        elseif radarObject.cls==7
            radarObject.cls = obstacle_cls.truck;
        else
            radarObject.cls = obstacle_cls.unknown;
        end
        %         radarObject.x = -radarObject.x;
        if (radarObject.det_src == det_src.right_backward || radarObject.det_src == det_src.right_forward)...
                && (radarObject.x <0 || (abs(radarObject.velo_y - g_ego_status.speed/3.6)<1.5 && radarObject.x<1 && radarObject.y<0&& radarObject.y>-9))   %剔除角雷达二次反射的无效数据
            radarObject.flag = uint8(0);
        end
    end
    radarObject.ID = radarObject.ID+uint16(1);         % 处理毫米波的ID 信息
    frameData.RadarFrame.RadarObjectList(i) = radarObject;
    %     flag = 1;
    [flag,obj_lane_radar] = ROIfilter(radarObject.y,radarObject.x,radarObject.cls, ...
        leftLane_valid,rightLane_valid,C_left,C_right,C_left_range,C_right_range);
    if flag>0 && radarObject.y~=0
        cntRadar = cntRadar+1;
        radarObject.obj_lane = obj_lane_radar;  %判断雷达的属性,为判断CIPV做准备
        radarObjects(cntRadar) = radarObject;
    end
end
frameData.RadarFrame.RadarObjectList = radarObjects;

%% ----------------------------处理Camera数据------------------------------------------------
% cameraObjects = repmat(obstacle_const.CameraObject,1,obstacle_const.max_obs_camera);
% cntCamera = 0;
for j = 1:obstacle_const.max_obs_camera
    if frameData.CameraFrame.CameraObjectList(j).flag<uint8(1)
        continue
    end
    camObject = frameData.CameraFrame.CameraObjectList(j);
    %Raimo信号处理
    %     if camObject.cls==0
    %         camObject.cls = obstacle_cls.car;
    %     elseif camObject.cls==1
    %         camObject.cls = obstacle_cls.truck;
    %     elseif camObject.cls==2
    %         camObject.cls = obstacle_cls.pedestrian;
    %     elseif camObject.cls==3
    %         camObject.cls = obstacle_cls.bus;
    %     elseif camObject.cls==4
    %         camObject.cls = obstacle_cls.bicycle;
    %     end
    %Mobileye信号处理
    if camObject.cls==0
        camObject.cls = obstacle_cls.car;
    elseif camObject.cls==1
        camObject.cls = obstacle_cls.truck;
    elseif camObject.cls==2
        camObject.cls = obstacle_cls.tricycle;
    elseif camObject.cls==3
        camObject.cls = obstacle_cls.pedestrian;
    elseif camObject.cls==4
        camObject.cls = obstacle_cls.bicycle;
    else
        camObject.cls = obstacle_cls.unknown;
    end
    camObject.ID = uint8(camObject.ID) + uint8(1);           % 处理摄像头的ID信息
    %     if rightLane_valid == 0 || leftLane_valid ==0  %Raimo
    %     if g_ego_status.yawrate <=2   %  处理EQ2摄像头Obj_lane 判断不准的问题
    [~,obj_lane_camera] = ROIfilter(camObject.y,camObject.x,camObject.cls, ...
        leftLane_valid,rightLane_valid,C_left,C_right,C_left_range,C_right_range);
    %     if camObject.CIPVFlag == uint8(1)
    
    camObject.obj_lane = obj_lane_camera;
    % test EQ2 CIPV判断是否正确
    %     if camObject.CIPVFlag == uint8(1)
    %         camObject.obj_lane = obj_lane.sameLane;
    %     end
    
    %     end
    frameData.CameraFrame.CameraObjectList(j) = camObject;
    
    %     flag = ROIfilter(camObject.y,camObject.x,C_left,C_right,C_left_range,C_right_range);
    %     if flag>0
    %         cntCamera = cntCamera+1;
    %         cameraObjects(cntCamera) = camObject;
    %     end
end
% frameData.CameraFrame.CameraObjectList = cameraObjects;
[frameData] = mbdProcessTimeStamp(frameData);
end

