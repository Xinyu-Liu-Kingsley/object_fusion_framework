function [frmData,carSignals] = getRaimoAndBaolong(CameraMatrix, RadarMatrix, vehiclematrix)
%处理Raimo摄像头和保隆雷达数据---解放车

[m1,~] = size(CameraMatrix.Ext0_image_lane);
frmData = repmat(obstacle_const.FrameData,m1,1);
for i = 1:m1-1
%     n1 = numel(CameraData(i).measures);
    % 存摄像头数据
    CameraDataSinlge  = frmData(i).CameraFrame.CameraObjectList;    % Camera的结构体形式
    CameraDataSinlge1  = frmData(i).CameraFrame.LaneList; 
%     CameraDataSinlge_process  = CameraData(i).measures;             % 采集数据
    for j = 1:10
            obj_ID = strcat('Gen',num2str(j-1),"_obj_id");
             if uint8(CameraMatrix.(obj_ID)(i,2))>=255
                 continue
             end
            obj_y = strcat('Gen',num2str(j-1),"_obj_y");
            obj_x = strcat('Gen',num2str(j-1),"_obj_x");
            obj_cls = strcat('Gen',num2str(j-1),"_obj_class");
            vleo_x = strcat('Gen',num2str(j-1),"_obj_x_vrel");
            vleo_y = strcat('Gen',num2str(j-1),"_obj_y_vrel");
            obj_width = strcat('Ext',num2str(j-1),"_obj_width");
            obj_y_arel = strcat('Ext',num2str(j-1),"_obj_y_arel");
            obj_det_prop = strcat('Ext',num2str(j-1),"_obj_det_prop");
            obj_exist_prop1 = strcat('Ext',num2str(j-1),"_obj_exist_prop");
            obj_dyn_prop = strcat('Ext',num2str(j-1),"_obj_dyn_prop");
            obj_lane1 = strcat('Ext',num2str(j-1),"_obj_lane");
            obj_direction1 = strcat('Ext',num2str(j-1),"_obj_direction");
            img_lane = strcat('Ext',num2str(j-1),"_image_lane");
            CameraDataSinlge(j).ID = uint8(CameraMatrix.(obj_ID)(i,2));
            CameraDataSinlge(j).x = double(CameraMatrix.(obj_x)(i,2));
            CameraDataSinlge(j).y = double(CameraMatrix.(obj_y)(i,2));

%             CameraDataSinlge(j).cls = CameraMatrix.(obj_cls)(i,2); 
            if CameraMatrix.(obj_cls)(i,2)==1
               CameraDataSinlge(j).cls = obstacle_cls.radarReflection; 
            elseif CameraMatrix.(obj_cls)(i,2)==2
               CameraDataSinlge(j).cls = obstacle_cls.pedestrian;
            elseif CameraMatrix.(obj_cls)(i,2)==3
               CameraDataSinlge(j).cls = obstacle_cls.bicycle; 
            elseif CameraMatrix.(obj_cls)(i,2)==4
               CameraDataSinlge(j).cls = obstacle_cls.bus; 
            elseif CameraMatrix.(obj_cls)(i,2)==5
               CameraDataSinlge(j).cls = obstacle_cls.car;
            elseif CameraMatrix.(obj_cls)(i,2)==6
               CameraDataSinlge(j).cls = obstacle_cls.truck;
            elseif CameraMatrix.(obj_cls)(i,2)==7
               CameraDataSinlge(j).cls = obstacle_cls.generalObject; 
            elseif CameraMatrix.(obj_cls)(i,2)==8
               CameraDataSinlge(j).cls = obstacle_cls.animal;
            elseif CameraMatrix.(obj_cls)(i,2)==9
               CameraDataSinlge(j).cls = obstacle_cls.tinyCar; 
            elseif CameraMatrix.(obj_cls)(i,2)==10
               CameraDataSinlge(j).cls = obstacle_cls.uncertainVehicle;
            elseif CameraMatrix.(obj_cls)(i,2)==11
               CameraDataSinlge(j).cls = obstacle_cls.tricycle;
            elseif CameraMatrix.(obj_cls)(i,2)==12
               CameraDataSinlge(j).cls = obstacle_cls.leftGuard; 
            elseif CameraMatrix.(obj_cls)(i,2)==13
               CameraDataSinlge(j).cls = obstacle_cls.rightGuard;
            elseif CameraMatrix.(obj_cls)(i,2)==15
               CameraDataSinlge(j).cls = obstacle_cls.invalid;
            else
               CameraDataSinlge(j).cls = obstacle_cls.unknown; 
            end

            CameraDataSinlge(j).velo_x = double(CameraMatrix.(vleo_x)(i,2));
            CameraDataSinlge(j).velo_y = double(CameraMatrix.(vleo_y)(i,2));
            CameraDataSinlge(j).obj_width = double(CameraMatrix.(obj_width)(i,2));
            CameraDataSinlge(j).obj_y_arel = double(CameraMatrix.(obj_y_arel)(i,2));

%             CameraDataSinlge(j).obj_det_prop = CameraMatrix.(obj_det_prop)(i,2);
            if CameraMatrix.(obj_det_prop)(i,2)==0
               CameraDataSinlge(j).obj_det_prop = obs_det_prop.undefined;
            elseif CameraMatrix.(obj_det_prop)(i,2)==1
               CameraDataSinlge(j).obj_det_prop = obs_det_prop.sole_radar;
            elseif CameraMatrix.(obj_det_prop)(i,2)==2
               CameraDataSinlge(j).obj_det_prop =obs_det_prop.sole_camera;
            elseif CameraMatrix.(obj_det_prop)(i,2)==3
               CameraDataSinlge(j).obj_det_prop = obs_det_prop.fused;
            end

%             CameraDataSinlge(j).obj_exist_prop = CameraMatrix.(obj_exist_prop1)(i,2);
             if CameraMatrix.(obj_exist_prop1)(i,2)==0
               CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_25;
            elseif CameraMatrix.(obj_exist_prop1)(i,2)==1
               CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_50;
            elseif CameraMatrix.(obj_exist_prop1)(i,2)==2
               CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_75;
            elseif CameraMatrix.(obj_exist_prop1)(i,2)==3
               CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_99;
            end

            CameraDataSinlge(j).obj_dyn_prop = CameraMatrix.(obj_dyn_prop)(i,2);

%             CameraDataSinlge(j).obj_lane = CameraMatrix.(obj_lane1)(i,2);
            if CameraMatrix.(obj_lane1)(i,2)==0
               CameraDataSinlge(j).obj_lane = obj_lane.undefined;
            elseif CameraMatrix.(obj_lane1)(i,2)==1
               CameraDataSinlge(j).obj_lane = obj_lane.sameLane;
            elseif CameraMatrix.(obj_lane1)(i,2)==2
               CameraDataSinlge(j).obj_lane = obj_lane.leftLane;
            elseif CameraMatrix.(obj_lane1)(i,2)==3
               CameraDataSinlge(j).obj_lane = obj_lane.rightLane;
            end


%             CameraDataSinlge(j).obj_direction = CameraMatrix.(obj_direction1)(i,2);
            if CameraMatrix.(obj_direction1)(i,2)==0
               CameraDataSinlge(j).obj_direction = obj_direction.same_direction;
            elseif CameraMatrix.(obj_direction1)(i,2)==1
               CameraDataSinlge(j).obj_direction = obj_direction.reverse_direction;
            elseif CameraMatrix.(obj_direction1)(i,2)==2
               CameraDataSinlge(j).obj_direction = obj_direction.crosswise_direction;
            elseif CameraMatrix.(obj_direction1)(i,2)==3
               CameraDataSinlge(j).obj_direction = obj_direction.reserve;
            end

%             CameraDataSinlge(j).img_lane = CameraMatrix.(img_lane)(i,2);
            if CameraMatrix.(img_lane)(i,2)==0
               CameraDataSinlge(j).img_lane = obj_lane.undefined;
            elseif CameraMatrix.(img_lane)(i,2)==1
               CameraDataSinlge(j).img_lane = obj_lane.sameLane;
            elseif CameraMatrix.(img_lane)(i,2)==2
               CameraDataSinlge(j).img_lane = obj_lane.leftLane;
            elseif CameraMatrix.(img_lane)(i,2)==3
               CameraDataSinlge(j).img_lane = obj_lane.rightLane;
            end

            CameraDataSinlge(j).flag = uint8(1);
    end
    frmData(i).CameraFrame.CameraObjectList = CameraDataSinlge;
%     存车道线信息
            CameraDataSinlge1(1).lane_locaiotn = lane_index.l_line;
            if CameraMatrix.LLAQ_Lane_quality(i,2) == 0
                CameraDataSinlge1(1).quality = lane_quality.low_quality;
            elseif CameraMatrix.LLAQ_Lane_quality(i,2) == 1
                CameraDataSinlge1(1).quality = lane_quality.high_quality;
            end
            if CameraMatrix.LLAQ_Lane_type(i,2) == 0
                CameraDataSinlge1(1).lane_type = lane_cls.singleDash;
            elseif CameraMatrix.LLAQ_Lane_type(i,2) == 1
                CameraDataSinlge1(1).lane_type = lane_cls.singleSolid;
            end
            CameraDataSinlge1(1).width_marking = double(CameraMatrix.LLAQ_Lane_widthMark(i,2));
            CameraDataSinlge1(1).C0 = double(CameraMatrix.LLAQ_lane_C0(i,2));
            CameraDataSinlge1(1).C1 = double(CameraMatrix.LLAQ_lane_C1(i,2));
            CameraDataSinlge1(1).C2 = double(CameraMatrix.LLBQ_lane_C2(i,2));
            CameraDataSinlge1(1).view_range = double(CameraMatrix.LLBQ_View_range(i,2));
            CameraDataSinlge1(1).view_range_valid = uint8(CameraMatrix.LLBQ_View_range_val(i,2));
            CameraDataSinlge1(1).flag = uint8(1);

            CameraDataSinlge1(2).lane_locaiotn = lane_index.r_line;
            if CameraMatrix.RLAQ_Lane_quality(i,2) == 0
                CameraDataSinlge1(2).quality = lane_quality.low_quality;
            elseif CameraMatrix.RLAQ_Lane_quality(i,2) == 1
                CameraDataSinlge1(2).quality = lane_quality.high_quality;
            end
            if CameraMatrix.RLAQ_Lane_type(i,2) == 0
                CameraDataSinlge1(2).lane_type = lane_cls.singleDash;
            elseif CameraMatrix.RLAQ_Lane_type(i,2) == 1
                CameraDataSinlge1(2).lane_type = lane_cls.singleSolid;
            end
            CameraDataSinlge1(2).width_marking = double(CameraMatrix.RLAQ_Lane_widthMark(i,2));
            CameraDataSinlge1(2).C0 = double(CameraMatrix.RLAQ_lane_C0(i,2));
            CameraDataSinlge1(2).C1 = double(CameraMatrix.RLAQ_lane_C1(i,2));
            CameraDataSinlge1(2).C2 = double(CameraMatrix.RLBQ_lane_C2(i,2));
            CameraDataSinlge1(2).view_range = double(CameraMatrix.RLBQ_View_range(i,2));
            CameraDataSinlge1(2).view_range_valid = uint8(CameraMatrix.RLBQ_View_range_val(i,2));
            CameraDataSinlge1(2).flag = uint8(1);
         
    frmData(i).CameraFrame.LaneList = CameraDataSinlge1;
end
%             CameraDataSinlge(j).y = double(CameraDataSinlge_process(j).y);
%             CameraDataSinlge(j).velo_x = double(CameraDataSinlge_process(j).rel_vx);
%             CameraDataSinlge(j).velo_y = double(CameraDataSinlge_process(j).rel_vy);
%             CameraDataSinlge(j).cls = obstacle_cls.car;
%             CameraDataSinlge(j).obj_width = double(CameraDataSinlge_process(j).width);
%             CameraDataSinlge(j).flag = 1;
%         end
%     end
    
%     %存雷达数据
[n2,~]= size(RadarMatrix.AK_Num_Far);
[n1,~]= size(RadarMatrix.Object_ID);
[n3,~]= size(RadarMatrix.Object_ID_1);
% RadarDataSinlge_process  = RadarData(i).frame; % 采集数据
cnt = 0;
%找第一帧有效雷达信息位置
for i_effective = 1:n1
     if abs(RadarMatrix.AK_Num_Near(1,1)-RadarMatrix.Object_ID(i_effective,1))<0.006...
         || abs(RadarMatrix.AK_Num_Near(1,1)-RadarMatrix.Object_ID(i_effective,1))>3
        break
     end
end
%对于雷达两帧报文相差一帧的情况，对齐雷达的两帧报文
 if n1<n3 && RadarMatrix.Object_ID(1,2)~=RadarMatrix.Object_ID_1(1,2)
    RadarMatrix.Object_ID_1 = RadarMatrix.Object_ID_1(2:end,:);
    RadarMatrix.Object_Ax = RadarMatrix.Object_Ax(2:end,:);
    RadarMatrix.Object_Ay = RadarMatrix.Object_Ay(2:end,:);
    RadarMatrix.Object_Confidence = RadarMatrix.Object_Confidence(2:end,:);
    RadarMatrix.Object_Vx = RadarMatrix.Object_Vx(2:end,:);
    RadarMatrix.Object_Vy = RadarMatrix.Object_Vy(2:end,:);
 elseif n1>n3 && RadarMatrix.Object_ID(1,2)~=RadarMatrix.Object_ID_1(1,2)
    RadarMatrix.Object_ID = RadarMatrix.Object_ID(2:end,:);
    RadarMatrix.Absolute_rest = RadarMatrix.Absolute_rest(2:end,:);
    RadarMatrix.MoveType = RadarMatrix.MoveType(2:end,:);
    RadarMatrix.Object_Azimuth = RadarMatrix.Object_Azimuth(2:end,:);
    RadarMatrix.Object_SNR = RadarMatrix.Object_SNR(2:end,:);
    RadarMatrix.Object_Speed = RadarMatrix.Object_Speed(2:end,:);
    RadarMatrix.Object_TrackStatus = RadarMatrix.Object_TrackStatus(2:end,:);
    RadarMatrix.Object_WarningStatu = RadarMatrix.Object_WarningStatu(2:end,:);
    RadarMatrix.Objective_Class = RadarMatrix.Objective_Class(2:end,:);
end
 for j1 = 1:n2-1
    num_far = RadarMatrix.AK_Num_Far(j1,2);
    num_near = RadarMatrix.AK_Num_Near(j1,2);
    num_total = num_far+num_near;
    RadarDataSinlge  = frmData(j1).RadarFrame.RadarObjectList;    % 定义的结构体形式
    for jj = 1:num_total
        RadarDataSinlge(jj).ID = uint8(RadarMatrix.Object_ID(cnt+jj+i_effective-1,2));
        if uint8(RadarMatrix.Object_ID(cnt+jj+i_effective-1,2))>250
            continue
        end
        RadarDataSinlge(jj).obj_range = double(RadarMatrix.Object_Range(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).moveType = double(RadarMatrix.MoveType(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).obj_speed = double(RadarMatrix.Object_Speed(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).obj_azimuth = double(RadarMatrix.Object_Azimuth(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).velo_x = double(RadarMatrix.Object_Vx(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).velo_y = double(RadarMatrix.Object_Vy(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).obj_trackStatus = double(RadarMatrix.Object_TrackStatus(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).obj_warningStaus = double(RadarMatrix.Object_WarningStatu(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).ax = double(RadarMatrix.Object_Ax(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).ay = double(RadarMatrix.Object_Ay(cnt+jj+i_effective-1,2));

%         RadarDataSinlge(jj).cls = RadarMatrix.Objective_Class(cnt+jj,2);
        if RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==1
            RadarDataSinlge(jj).cls = obstacle_cls.radarReflection;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==2
            RadarDataSinlge(jj).cls = obstacle_cls.pedestrian;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==3
            RadarDataSinlge(jj).cls = obstacle_cls.bicycle;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==4
            RadarDataSinlge(jj).cls = obstacle_cls.bus;  
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==5
            RadarDataSinlge(jj).cls = obstacle_cls.car;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==6
            RadarDataSinlge(jj).cls = obstacle_cls.truck;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==7
            RadarDataSinlge(jj).cls = obstacle_cls.generalObject; 
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==8
            RadarDataSinlge(jj).cls = obstacle_cls.animal;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==9
            RadarDataSinlge(jj).cls = obstacle_cls.tinyCar;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==10
            RadarDataSinlge(jj).cls = obstacle_cls.uncertainVehicle;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==11
            RadarDataSinlge(jj).cls = obstacle_cls.tricycle; 
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==12
            RadarDataSinlge(jj).cls = obstacle_cls.leftGuard;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==13
            RadarDataSinlge(jj).cls = obstacle_cls.rightGuard;
        elseif RadarMatrix.Objective_Class(cnt+jj+i_effective-1,2)==15
            RadarDataSinlge(jj).cls = obstacle_cls.invalid;
        else
            RadarDataSinlge(jj).cls = obstacle_cls.unknown;
        end
        
        RadarDataSinlge(jj).SNR = double(RadarMatrix.Object_SNR(cnt+jj+i_effective-1,2));
        RadarDataSinlge(jj).flag = uint8(1);
    end
    cnt = cnt+num_total;
    frmData(j1).RadarFrame.RadarObjectList = RadarDataSinlge;
 end

 %存整车信息carSignals

    [k1,~] = size(vehiclematrix.CCVS1_WheelBasedVeh );
    [k2,~] = size(vehiclematrix.VehicleYawRate  );
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
                if abs(vehiclematrix.CCVS1_WheelBasedVeh(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
                    VehSpeed(k,2) = vehiclematrix.CCVS1_WheelBasedVeh(kk,2);
                    VehSpeed(k,1) = vehiclematrix.CCVS1_WheelBasedVeh(kk,1);
                    m_speed = kk;
                    break
                end
            end
           for kk = m_yaw:k2
                if abs(vehiclematrix.VehicleYawRate(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
                    YawRate(k,2) =  vehiclematrix.VehicleYawRate(kk,2);
                    YawRate(k,1) =  vehiclematrix.VehicleYawRate(kk,1);
                    m_yaw = kk;
                    break
                end
            end
        end
        carSignals(:,1) = VehSpeed(:,2);
        carSignals(:,2) = YawRate(:,2);

end

