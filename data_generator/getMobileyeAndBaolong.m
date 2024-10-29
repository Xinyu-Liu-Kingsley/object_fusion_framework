function [frmData,carSignals] = getMobileyeAndBaolong(CameraMatrix, RadarMatrix, vehiclematrix)
%处理Mobileye摄像头和保隆雷达数据---解放车

[m1,~] = size(CameraMatrix.ObstacleStatus__NumberOfObstacles);
frmData = repmat(obstacle_const.FrameData,m1,1);
% 摄像头不同目标之间时间对齐
 cnt = zeros(32,1);
 num_max = max(CameraMatrix.ObstacleStatus__NumberOfObstacles(:,2));
    for j = 1:num_max
        obj_ID = strcat('Obstacle',num2str(j-1),"DataA__Obstacle_ID");
         [k1,~] = size(CameraMatrix.(obj_ID));
       if k1 == m1
          continue
       else
        obj_ID = strcat('Obstacle',num2str(j-1),"DataA__Obstacle_ID");
        obj_y = strcat('Obstacle',num2str(j-1),"DataA__ObstaclePositionY");
        obj_x = strcat('Obstacle',num2str(j-1),"DataA__ObstaclePositonX");
        obj_cls = strcat('Obstacle',num2str(j-1),"DataA__ObstacleType");
        vleo_x = strcat('Obstacle',num2str(j-1),"DataA__ObstacleRelativeVelocityX");
        obj_width = strcat('Obstacle',num2str(j-1),"DataB__ObstacleWidth");
        obj_y_arel = strcat('Obstacle',num2str(j-1),"DataC__ObstacleAccelX");
        obj_of_lane = strcat('Obstacle',num2str(j-1),"DataB__ObstacleLane");
        ID = zeros(m1,2);
        y = zeros(m1,2);
        x = zeros(m1,2);
        cls = zeros(m1,2);
        vx = zeros(m1,2);
        width = zeros(m1,2);
        ay = zeros(m1,2);
        ObstacleLane = zeros(m1,2);
        for k = 1:m1-1
            [n,~]=find(CameraMatrix.ObstacleStatus__NumberOfObstacles(:,2)<=j-1);
            for k1=1:length(n)
                if k==n(k1)
                   cnt(j)=cnt(j)+1;
                    ID(k,1) = 0;         ID(k,2) = 255;
                    y(k,1) =  0;         y(k,2) =  0;
                    x(k,1) =  0;         x(k,2) =  0;
                    cls(k,1) =  0;       cls(k,2) =  0;
                    vx(k,1) =  0;        vx(k,2) =  0;
                    width(k,1) =  0;        width(k,2) =  0;
                    ay(k,1) =  0;        ay(k,2) =  0;
                    ObstacleLane(k,1) =  0;   ObstacleLane(k,2) =  0;
                   break
                elseif k-cnt(j)<=length(CameraMatrix.(obj_ID)(:,2))
                    ID(k,2) = CameraMatrix.(obj_ID)(k-cnt(j),2);
                    ID(k,1) = CameraMatrix.(obj_ID)(k-cnt(j),1);
                    y(k,2) =  CameraMatrix.(obj_y)(k-cnt(j),2);
                    y(k,1) =  CameraMatrix.(obj_y)(k-cnt(j),1);
                    x(k,2) =  CameraMatrix.(obj_x)(k-cnt(j),2);
                    x(k,1) =  CameraMatrix.(obj_x)(k-cnt(j),1);
                    cls(k,2) =  CameraMatrix.(obj_cls)(k-cnt(j),2);
                    cls(k,1) =  CameraMatrix.(obj_cls)(k-cnt(j),1);
                    vx(k,2) =  CameraMatrix.(vleo_x)(k-cnt(j),2);
                    vx(k,1) =  CameraMatrix.(vleo_x)(k-cnt(j),1);
                    width(k,2) =  CameraMatrix.(obj_width)(k-cnt(j),2);
                    width(k,1) =  CameraMatrix.(obj_width)(k-cnt(j),1);
                    ay(k,2) =  CameraMatrix.(obj_y_arel)(k-cnt(j),2);
                    ay(k,1) =  CameraMatrix.(obj_y_arel)(k-cnt(j),1);
                    ObstacleLane(k,2) =  CameraMatrix.(obj_of_lane)(k-cnt(j),2);
                    ObstacleLane(k,1) =  CameraMatrix.(obj_of_lane)(k-cnt(j),1);
                end
            end
        end
        CameraMatrix.(obj_ID) = ID;
        CameraMatrix.(obj_y) = y;
        CameraMatrix.(obj_x)  = x;
        CameraMatrix.(obj_cls)  = cls;
        CameraMatrix.(vleo_x) = vx;
        CameraMatrix.(obj_width) = width;
        CameraMatrix.(obj_y_arel) = ay;
        CameraMatrix.(obj_of_lane) = ObstacleLane;
       end
   end

for i = 1:m1-1
%     n1 = numel(CameraData(i).measures);
    % 存摄像头数据
    CameraDataSinlge  = frmData(i).CameraFrame.CameraObjectList;    % Camera的结构体形式
    CameraDataSinlge1  = frmData(i).CameraFrame.LaneList; 
%     CameraDataSinlge_process  = CameraData(i).measures;             % 采集数据
    for ii = 1:num_max
         obj_ID = strcat('Obstacle',num2str(ii-1),"DataA__Obstacle_ID");
       if uint8(CameraMatrix.(obj_ID)(i,2))==255
          continue
       end
        obj_y = strcat('Obstacle',num2str(ii-1),"DataA__ObstaclePositionY");
        obj_x = strcat('Obstacle',num2str(ii-1),"DataA__ObstaclePositonX");
        obj_cls = strcat('Obstacle',num2str(ii-1),"DataA__ObstacleType");
        vleo_x = strcat('Obstacle',num2str(ii-1),"DataA__ObstacleRelativeVelocityX");
        obj_width = strcat('Obstacle',num2str(ii-1),"DataB__ObstacleWidth");
        obj_y_arel = strcat('Obstacle',num2str(ii-1),"DataC__ObstacleAccelX");
        obj_of_lane = strcat('Obstacle',num2str(ii-1),"DataB__ObstacleLane");
        CameraDataSinlge(ii).ID = uint8(CameraMatrix.(obj_ID)(i,2));
        CameraDataSinlge(ii).x = double(-CameraMatrix.(obj_y)(i,2));
        CameraDataSinlge(ii).y = double(CameraMatrix.(obj_x)(i,2));

            if CameraMatrix.(obj_cls)(i,2)==1
               CameraDataSinlge(ii).cls = obstacle_cls.radarReflection; 
            elseif CameraMatrix.(obj_cls)(i,2)==2
               CameraDataSinlge(ii).cls = obstacle_cls.pedestrian;
            elseif CameraMatrix.(obj_cls)(i,2)==3
               CameraDataSinlge(ii).cls = obstacle_cls.bicycle; 
            elseif CameraMatrix.(obj_cls)(i,2)==4
               CameraDataSinlge(ii).cls = obstacle_cls.bus; 
            elseif CameraMatrix.(obj_cls)(i,2)==5
               CameraDataSinlge(ii).cls = obstacle_cls.car;
            elseif CameraMatrix.(obj_cls)(i,2)==6
               CameraDataSinlge(ii).cls = obstacle_cls.truck;
            elseif CameraMatrix.(obj_cls)(i,2)==7
               CameraDataSinlge(ii).cls = obstacle_cls.generalObject; 
            elseif CameraMatrix.(obj_cls)(i,2)==8
               CameraDataSinlge(ii).cls = obstacle_cls.animal;
            elseif CameraMatrix.(obj_cls)(i,2)==9
               CameraDataSinlge(ii).cls = obstacle_cls.tinyCar; 
            elseif CameraMatrix.(obj_cls)(i,2)==10
               CameraDataSinlge(ii).cls = obstacle_cls.uncertainVehicle;
            elseif CameraMatrix.(obj_cls)(i,2)==11
               CameraDataSinlge(ii).cls = obstacle_cls.tricycle;
            elseif CameraMatrix.(obj_cls)(i,2)==12
               CameraDataSinlge(ii).cls = obstacle_cls.leftGuard; 
            elseif CameraMatrix.(obj_cls)(i,2)==13
               CameraDataSinlge(ii).cls = obstacle_cls.rightGuard;
            elseif CameraMatrix.(obj_cls)(i,2)==15
               CameraDataSinlge(ii).cls = obstacle_cls.invalid;
            else
               CameraDataSinlge(ii).cls = obstacle_cls.unknown; 
            end

            CameraDataSinlge(ii).velo_y = double(CameraMatrix.(vleo_x)(i,2));
            CameraDataSinlge(ii).obj_width = double(CameraMatrix.(obj_width)(i,2));
            CameraDataSinlge(ii).obj_y_arel = double(CameraMatrix.(obj_y_arel)(i,2));

%             CameraDataSinlge(i).obj_lane = CameraMatrix.ObstacleB_Length(i,2);
            if CameraMatrix.(obj_of_lane)(i,2)==0
               CameraDataSinlge(ii).obj_lane = obj_lane.undefined;
            elseif CameraMatrix.(obj_of_lane)(i,2)==1
               CameraDataSinlge(ii).obj_lane = obj_lane.sameLane;
            elseif CameraMatrix.(obj_of_lane)(i,2)==2
               CameraDataSinlge(ii).obj_lane = obj_lane.leftLane;
            elseif CameraMatrix.(obj_of_lane)(i,2)==3
               CameraDataSinlge(ii).obj_lane = obj_lane.rightLane;
            end

            CameraDataSinlge(ii).flag = uint8(1);
    end
   
    frmData(i).CameraFrame.CameraObjectList = CameraDataSinlge;
    frmData(i).CameraFrame.time_ns = int64(CameraMatrix.ObstacleStatus__NumberOfObstacles(i,1)*1000);
%     存车道线信息
            CameraDataSinlge1(1).lane_locaiotn = lane_index.l_line;
            if CameraMatrix.LKA_LeftLaneA__LaneQuality(i,2) == 0 || CameraMatrix.LKA_LeftLaneA__LaneQuality(i,2) == 1
                CameraDataSinlge1(1).quality = lane_quality.low_quality;
            elseif CameraMatrix.LKA_LeftLaneA__LaneQuality(i,2) == 2 || CameraMatrix.LKA_LeftLaneA__LaneQuality(i,2) == 3
                CameraDataSinlge1(1).quality = lane_quality.high_quality;
            else
                CameraDataSinlge1(1).quality = lane_quality.low_quality;
            end
            if CameraMatrix.LKA_LeftLaneA__LaneType(i,2) == 0
                CameraDataSinlge1(1).lane_type = lane_cls.singleDash;
            elseif CameraMatrix.LKA_LeftLaneA__LaneType(i,2) == 1
                CameraDataSinlge1(1).lane_type = lane_cls.singleSolid;
            elseif CameraMatrix.LKA_LeftLaneA__LaneType(i,2) == 2
                CameraDataSinlge1(1).lane_type = lane_cls.lDashrSolid;
            end
            CameraDataSinlge1(1).width_marking = double(CameraMatrix.LKA_LeftLaneA__WidthLeftMarking(i,2));
            CameraDataSinlge1(1).C0 = double(CameraMatrix.LKA_LeftLaneA__PositionParameter_C0(i,2));
            CameraDataSinlge1(1).C1 = double(CameraMatrix.LKA_LeftLaneB__HeadingAngleParameter(i,2));
            CameraDataSinlge1(1).C2 = double(CameraMatrix.LKA_LeftLaneA__CurvatureParameter_C2(i,2));
            CameraDataSinlge1(1).C3 = double(CameraMatrix.LKA_LeftLaneA__CurvatureDerivativeParameter_C3(i,2));
            CameraDataSinlge1(1).view_range = double(CameraMatrix.LKA_LeftLaneB__ViewRange(i,2));
            CameraDataSinlge1(1).view_range_valid = uint8(CameraMatrix.LKA_LeftLaneB__ViewRangeAvailability(i,2));
            CameraDataSinlge1(1).flag = uint8(1);

            CameraDataSinlge1(2).lane_locaiotn = lane_index.r_line;
            if CameraMatrix.LKA_RightLaneA__LaneQuality(i,2) == 0 || CameraMatrix.LKA_RightLaneA__LaneQuality(i,2) == 1
                CameraDataSinlge1(2).quality = lane_quality.low_quality;
            elseif CameraMatrix.LKA_RightLaneA__LaneQuality(i,2) == 2 || CameraMatrix.LKA_RightLaneA__LaneQuality(i,2) == 3
                CameraDataSinlge1(2).quality = lane_quality.high_quality;
            else
                CameraDataSinlge1(2).quality = lane_quality.low_quality;
            end
            if CameraMatrix.LKA_RightLaneA__LaneType(i,2) == 0
                CameraDataSinlge1(2).lane_type = lane_cls.singleDash;
            elseif CameraMatrix.LKA_RightLaneA__LaneType(i,2) == 1
                CameraDataSinlge1(2).lane_type = lane_cls.singleSolid;
            elseif CameraMatrix.LKA_RightLaneA__LaneType(i,2) == 2
                CameraDataSinlge1(2).lane_type = lane_cls.lDashrSolid;
            end
            CameraDataSinlge1(2).width_marking = double(CameraMatrix.LKA_RightLaneA__WidthLeftMarking(i,2));
            CameraDataSinlge1(2).C0 = double(CameraMatrix.LKA_RightLaneA__PositionParameter_C0(i,2));
            CameraDataSinlge1(2).C1 = double(CameraMatrix.LKA_RightLaneB__HeadingAngleParameter(i,2));
            CameraDataSinlge1(2).C2 = double(CameraMatrix.LKA_RightLaneA__CurvatureParameter_C2(i,2));
            CameraDataSinlge1(2).C3 = double(CameraMatrix.LKA_RightLaneA__CurvatureDerivativeParameter_C3(i,2));
            CameraDataSinlge1(2).view_range = double(CameraMatrix.LKA_RightLaneB__ViewRange(i,2));
            CameraDataSinlge1(2).view_range_valid = uint8(CameraMatrix.LKA_RightLaneB__ViewRangeAvailability(i,2));
            CameraDataSinlge1(2).flag = uint8(1);
         
    frmData(i).CameraFrame.LaneList = CameraDataSinlge1;
end

    
%存雷达数据
[n2,~]= size(RadarMatrix.AK_Status_0__AK_Num_Far);
[n1,~]= size(RadarMatrix.AK_General_1__Object_ID);
[n3,~]= size(RadarMatrix.AK_General_2__Object_ID);
% RadarDataSinlge_process  = RadarData(i).frame; % 采集数据
cnt_j2=1;
cnt_num_obj=0;
%找第一帧有效雷达信息位置
for i_effective = 1:n1
     if abs(RadarMatrix.AK_Status_0__AK_Num_Far(1,1)-RadarMatrix.AK_General_1__Object_ID(i_effective,1))<0.0005
        break
     end
end
% i_effective = 1;
%对于雷达两帧报文相差一帧的情况，对齐雷达的两帧报文
 if n1<n3 && RadarMatrix.AK_General_1__Object_ID(1,2)~=RadarMatrix.AK_General_2__Object_ID(1,2)
    RadarMatrix.AK_General_2__Object_ID = RadarMatrix.AK_General_2__Object_ID(2:end,:);
    RadarMatrix.AK_General_2__Object_Ax = RadarMatrix.AK_General_2__Object_Ax(2:end,:);
    RadarMatrix.AK_General_2__Object_Ay = RadarMatrix.AK_General_2__Object_Ay(2:end,:);
    RadarMatrix.AK_General_2__Object_Confidence = RadarMatrix.AK_General_2__Object_Confidence(2:end,:);
    RadarMatrix.AK_General_2__Object_Vx = RadarMatrix.AK_General_2__Object_Vx(2:end,:);
    RadarMatrix.AK_General_2__Object_Vy = RadarMatrix.AK_General_2__Object_Vy(2:end,:);
 elseif n1>n3 && RadarMatrix.AK_General_1__Object_ID(1,2)~=RadarMatrix.AK_General_2__Object_ID(1,2)
    RadarMatrix.AK_General_1__Object_ID = RadarMatrix.AK_General_1__Object_ID(2:end,:);
    RadarMatrix.AK_General_1__Absolute_rest = RadarMatrix.AK_General_1__Absolute_rest(2:end,:);
    RadarMatrix.AK_General_1__MoveType = RadarMatrix.AK_General_1__MoveType(2:end,:);
    RadarMatrix.AK_General_1__Object_Azimuth = RadarMatrix.AK_General_1__Object_Azimuth(2:end,:);
    RadarMatrix.AK_General_1__Object_SNR = RadarMatrix.AK_General_1__Object_SNR(2:end,:);
    RadarMatrix.AK_General_1__Object_Speed = RadarMatrix.AK_General_1__Object_Speed(2:end,:);
    RadarMatrix.AK_General_1__Object_TrackStatus = RadarMatrix.AK_General_1__Object_TrackStatus(2:end,:);
    RadarMatrix.AK_General_1__Object_WarningStatus = RadarMatrix.AK_General_1__Object_WarningStatus(2:end,:);
    RadarMatrix.AK_General_1__Objective_Class = RadarMatrix.AK_General_1__Objective_Class(2:end,:);
 end

 for j1 = 1:n2-1
    num_far = RadarMatrix.AK_Status_0__AK_Num_Far(j1,2);
    num_near = RadarMatrix.AK_Status_0__AK_Num_Near(j1,2);
    num_total = num_far+num_near;
 
     for j2 = cnt_j2:m1-1
                RadarDataSinlge  = frmData(j2).RadarFrame.RadarObjectList;    % 定义的结构体形式
        if abs(RadarMatrix.AK_Status_0__AK_Num_Far(j1,1)-CameraMatrix.ObstacleStatus__NumberOfObstacles(j2,1))<0.03
                cnt_j2 = j2+1;
          for jj = 1:num_total
                RadarDataSinlge(jj).ID = uint8(RadarMatrix.AK_General_1__Object_ID(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).obj_range = double(RadarMatrix.AK_General_1__Object_Range(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).moveType = double(RadarMatrix.AK_General_1__MoveType(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).obj_speed = double(RadarMatrix.AK_General_1__Object_Speed(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).obj_azimuth = double(RadarMatrix.AK_General_1__Object_Azimuth(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).velo_x = double(RadarMatrix.AK_General_2__Object_Vx(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).velo_y = double(RadarMatrix.AK_General_2__Object_Vy(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).obj_trackStatus = double(RadarMatrix.AK_General_1__Object_TrackStatus(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).obj_warningStaus = double(RadarMatrix.AK_General_1__Object_WarningStatus(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).ax = double(RadarMatrix.AK_General_2__Object_Ax(cnt_num_obj+jj+i_effective-1,2));
                RadarDataSinlge(jj).ay = double(RadarMatrix.AK_General_2__Object_Ay(cnt_num_obj+jj+i_effective-1,2));

%                 RadarDataSinlge(jj).cls = RadarMatrix.Objective_Class(cnt+jj,2);
                if RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==1
                    RadarDataSinlge(jj).cls = obstacle_cls.radarReflection;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==2
                    RadarDataSinlge(jj).cls = obstacle_cls.pedestrian;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==3
                    RadarDataSinlge(jj).cls = obstacle_cls.bicycle;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==4
                    RadarDataSinlge(jj).cls = obstacle_cls.bus;  
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==5
                    RadarDataSinlge(jj).cls = obstacle_cls.car;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==6
                    RadarDataSinlge(jj).cls = obstacle_cls.truck;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==7
                    RadarDataSinlge(jj).cls = obstacle_cls.generalObject; 
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==8
                    RadarDataSinlge(jj).cls = obstacle_cls.animal;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==9
                    RadarDataSinlge(jj).cls = obstacle_cls.tinyCar;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==10
                    RadarDataSinlge(jj).cls = obstacle_cls.uncertainVehicle;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==11
                    RadarDataSinlge(jj).cls = obstacle_cls.tricycle; 
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==12
                    RadarDataSinlge(jj).cls = obstacle_cls.leftGuard;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==13
                    RadarDataSinlge(jj).cls = obstacle_cls.rightGuard;
                elseif RadarMatrix.AK_General_1__Objective_Class(cnt_num_obj+jj+i_effective-1,2)==15
                    RadarDataSinlge(jj).cls = obstacle_cls.invalid;
                else
                    RadarDataSinlge(jj).cls = obstacle_cls.unknown;
                end
        
                    RadarDataSinlge(jj).SNR = double(RadarMatrix.AK_General_1__Object_SNR(cnt_num_obj+jj+i_effective-1,2));
                    RadarDataSinlge(jj).flag = uint8(1);
          end
%         cnt = cnt+num_total;
        frmData(j2).RadarFrame.RadarObjectList = RadarDataSinlge;
        frmData(j2).RadarFrame.time_ns = int64(RadarMatrix.AK_Status_0__AK_Num_Far(j1,1)*1000);
        break
        end
     end
   cnt_num_obj=cnt_num_obj+num_total;
 end

 %存整车信息carSignals

    [k1,~] = size(vehiclematrix.CCVS1__CCVS1_WheelBasedVehicleSpeed);
    [k2,~] = size(vehiclematrix.Vehcle_Speed_st__Veh_Spd_yaw_rate);
    VehSpeed = zeros(m1,2);
    YawRate = zeros(m1,2);
    carSignals = zeros(m1,2);
        m_speed = 1;
        m_yaw = 1;
        for k = 1:m1-1
            for kk = m_speed:k1
%                 if abs(RadarMatrix.VehInfo1__VehSpeed(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
%                     VehSpeed(k,2) = RadarMatrix.VehInfo1__VehSpeed(kk,2);
%                     VehSpeed(k,1) = RadarMatrix.VehInfo1__VehSpeed(kk,1);
%                     YawRate(k,2) =  RadarMatrix.VehInfo1__YawRate(kk,2);
%                     YawRate(k,1) =  RadarMatrix.VehInfo1__YawRate(kk,1);
%                     m = kk;
%                     break
%                 end
                if abs(vehiclematrix.CCVS1__CCVS1_WheelBasedVehicleSpeed(kk,1)-CameraMatrix.ObstacleStatus__NumberOfObstacles(k,1))<0.06
                    VehSpeed(k,2) = vehiclematrix.CCVS1__CCVS1_WheelBasedVehicleSpeed(kk,2);
                    VehSpeed(k,1) = vehiclematrix.CCVS1__CCVS1_WheelBasedVehicleSpeed(kk,1);
                    m_speed = kk;
                    break
                end
            end
           for kk = m_yaw:k2
                if abs(vehiclematrix.Vehcle_Speed_st__Veh_Spd_yaw_rate(kk,1)-CameraMatrix.ObstacleStatus__NumberOfObstacles(k,1))<0.06
                    YawRate(k,2) =  vehiclematrix.Vehcle_Speed_st__Veh_Spd_yaw_rate(kk,2);
                    YawRate(k,1) =  vehiclematrix.Vehcle_Speed_st__Veh_Spd_yaw_rate(kk,1);
                    m_yaw = kk;
                    break
                end
            end
        end
        carSignals(:,1) = VehSpeed(:,2);
        carSignals(:,2) = YawRate(:,2);
end

