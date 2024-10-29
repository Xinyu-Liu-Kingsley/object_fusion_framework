function [frmData,carSignals] = getJimuAndXingyidao(CameraMatrix, RadarMatrix, vehiclematrix)
%处理极目摄像头和行易道雷达数据---三一车

[mCam,~] =size(CameraMatrix.Ext0_image_lane);

frmData = repmat(obstacle_const.FrameData,mCam,1);
for i = 1:mCam-1
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

        if CameraMatrix.(obj_det_prop)(i,2)==0
            CameraDataSinlge(j).obj_det_prop = obs_det_prop.undefined;
        elseif CameraMatrix.(obj_det_prop)(i,2)==1
            CameraDataSinlge(j).obj_det_prop = obs_det_prop.sole_radar;
        elseif CameraMatrix.(obj_det_prop)(i,2)==2
            CameraDataSinlge(j).obj_det_prop = obs_det_prop.sole_camera;
        elseif CameraMatrix.(obj_det_prop)(i,2)==3
            CameraDataSinlge(j).obj_det_prop = obs_det_prop.fused;
        end
        %             CameraDataSinlge(j).obj_det_prop = CameraMatrix.(obj_det_prop)(i,2);

        if CameraMatrix.(obj_exist_prop1)(i,2)==0
            CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_25;
        elseif CameraMatrix.(obj_exist_prop1)(i,2)==1
            CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_50;
        elseif CameraMatrix.(obj_exist_prop1)(i,2)==2
            CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_75;
        elseif CameraMatrix.(obj_exist_prop1)(i,2)==3
            CameraDataSinlge(j).obj_exist_prop = obj_exist_prop.percent_99;
        end
        %             CameraDataSinlge(j).obj_exist_prop = CameraMatrix.(obj_exist_prop)(i,2);
        CameraDataSinlge(j).obj_dyn_prop = CameraMatrix.(obj_dyn_prop)(i,2);

        if CameraMatrix.(obj_lane1)(i,2)==0
            CameraDataSinlge(j).obj_lane = obj_lane.undefined;
        elseif CameraMatrix.(obj_lane1)(i,2)==1
            CameraDataSinlge(j).obj_lane = obj_lane.sameLane;
        elseif CameraMatrix.(obj_lane1)(i,2)==2
            CameraDataSinlge(j).obj_lane = obj_lane.leftLane;
        elseif CameraMatrix.(obj_lane1)(i,2)==3
            CameraDataSinlge(j).obj_lane = obj_lane.rightLane;
        end
        %             CameraDataSinlge(j).obj_lane = double(CameraMatrix.(obj_lane)(i,2));

        if CameraMatrix.(obj_direction1)(i,2)==0
            CameraDataSinlge(j).obj_direction = obj_direction.same_direction;
        elseif CameraMatrix.(obj_direction1)(i,2)==1
            CameraDataSinlge(j).obj_direction = obj_direction.reverse_direction;
        elseif CameraMatrix.(obj_direction1)(i,2)==2
            CameraDataSinlge(j).obj_direction = obj_direction.crosswise_direction;
        elseif CameraMatrix.(obj_direction1)(i,2)==3
            CameraDataSinlge(j).obj_direction = obj_direction.reserve;
        end
        %             CameraDataSinlge(j).obj_direction = CameraMatrix.(obj_direction)(i,2);

        if CameraMatrix.(img_lane)(i,2)==0
            CameraDataSinlge(j).img_lane = obj_lane.undefined;
        elseif CameraMatrix.(img_lane)(i,2)==1
            CameraDataSinlge(j).img_lane = obj_lane.sameLane;
        elseif CameraMatrix.(img_lane)(i,2)==2
            CameraDataSinlge(j).img_lane = obj_lane.leftLane;
        elseif CameraMatrix.(img_lane)(i,2)==3
            CameraDataSinlge(j).img_lane = obj_lane.rightLane;
        end
        %             CameraDataSinlge(j).img_lane = CameraMatrix.(img_lane)(i,2);

        CameraDataSinlge(j).flag = uint8(1);
    end
    frmData(i).CameraFrame.CameraObjectList = CameraDataSinlge;
    %  存车道线信息
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

% 存雷达数据
% 雷达不同目标之间的对齐
[m2,~] = size(RadarMatrix.Header__No_Obj);
num_add = 32- RadarMatrix.Header__No_Obj(:,2);
cnt = zeros(32,1);

for j = 1:32
    obj_ID = strcat('Target',num2str(j),"_ID_A");
    [k1,~] = size(RadarMatrix.(obj_ID));
    if k1 == m2
        continue
    else
        obj_ID = strcat('Target',num2str(j),"_ID_A");
        obj_y = strcat('Target',num2str(j),"_Pos_Y");
        obj_x = strcat('Target',num2str(j),"_Pos_X");
        obj_cls = strcat('Target',num2str(j),"_Type");
        vleo_x = strcat('Target',num2str(j),"_Vel_X");
        vleo_y = strcat('Target',num2str(j),"_Vel_Y");
        obj_ax = strcat('Target',num2str(j),"_Accel_X");
        obj_dyn_prop = strcat('Target',num2str(j),"_DynProp");
        ID = zeros(m2,2);
        y = zeros(m2,2);
        x = zeros(m2,2);
        cls = zeros(m2,2);
        vx = zeros(m2,2);
        vy = zeros(m2,2);
        ax = zeros(m2,2);
        DynProp = zeros(m2,2);
        for k = 1:m2-1
            [n,~]=find(num_add>=32-j+1);
            for k1=1:length(n)
                if k==n(k1)
                   cnt(j)=cnt(j)+1;
                    ID(k,1) = 0;         ID(k,2) = 0;
                    y(k,1) =  0;         y(k,2) =  0;
                    x(k,1) =  0;         x(k,2) =  0;
                    cls(k,1) =  0;       cls(k,2) =  0;
                    vx(k,1) =  0;        vx(k,2) =  0;
                    vy(k,1) =  0;        vy(k,2) =  0;
                    ax(k,1) =  0;        ax(k,2) =  0;
                    DynProp(k,1) =  0;   DynProp(k,2) =  0;
                   break
                elseif k-cnt(j)<=length(RadarMatrix.(obj_ID)(:,2))
                    ID(k,2) = RadarMatrix.(obj_ID)(k-cnt(j),2);
                    ID(k,1) = RadarMatrix.(obj_ID)(k-cnt(j),1);
                    y(k,2) =  RadarMatrix.(obj_y)(k-cnt(j),2);
                    y(k,1) =  RadarMatrix.(obj_y)(k-cnt(j),1);
                    x(k,2) =  RadarMatrix.(obj_x)(k-cnt(j),2);
                    x(k,1) =  RadarMatrix.(obj_x)(k-cnt(j),1);
                    cls(k,2) =  RadarMatrix.(obj_cls)(k-cnt(j),2);
                    cls(k,1) =  RadarMatrix.(obj_cls)(k-cnt(j),1);
                    vx(k,2) =  RadarMatrix.(vleo_x)(k-cnt(j),2);
                    vx(k,1) =  RadarMatrix.(vleo_x)(k-cnt(j),1);
                    vy(k,2) =  RadarMatrix.(vleo_y)(k-cnt(j),2);
                    vy(k,1) =  RadarMatrix.(vleo_y)(k-cnt(j),1);
                    ax(k,2) =  RadarMatrix.(obj_ax)(k-cnt(j),2);
                    ax(k,1) =  RadarMatrix.(obj_ax)(k-cnt(j),1);
                    DynProp(k,2) =  RadarMatrix.(obj_dyn_prop)(k-cnt(j),2);
                    DynProp(k,1) =  RadarMatrix.(obj_dyn_prop)(k-cnt(j),1);
                end
            end
        end
        RadarMatrix.(obj_ID) = ID;
        RadarMatrix.(obj_y) = y;
        RadarMatrix.(obj_x)  = x;
        RadarMatrix.(obj_cls)  = cls;
        RadarMatrix.(vleo_x) = vx;
        RadarMatrix.(vleo_y) = vy;
        RadarMatrix.(obj_ax) = ax;
        RadarMatrix.(obj_dyn_prop) = DynProp;
    end
    
end

% 雷达和摄像头的时间戳对齐

for j = 1:32
    obj_ID = strcat('Target',num2str(j),"_ID_A");
    obj_y = strcat('Target',num2str(j),"_Pos_Y");
    obj_x = strcat('Target',num2str(j),"_Pos_X");
    obj_cls = strcat('Target',num2str(j),"_Type");
    vleo_x = strcat('Target',num2str(j),"_Vel_X");
    vleo_y = strcat('Target',num2str(j),"_Vel_Y");
    obj_ax = strcat('Target',num2str(j),"_Accel_X");
    obj_dyn_prop = strcat('Target',num2str(j),"_DynProp");
    [k1,~] = size(RadarMatrix.(obj_ID));
    ID = zeros(mCam,2);
    y = zeros(mCam,2);
    x = zeros(mCam,2);
    cls = zeros(mCam,2);
    vx = zeros(mCam,2);
    vy = zeros(mCam,2);
    ax = zeros(mCam,2);
    DynProp = zeros(mCam,2);
        m = 1;
        for k = 1:mCam
            for kk = m:k1
                if abs(RadarMatrix.(obj_ID)(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
                    ID(k,2) = RadarMatrix.(obj_ID)(kk,2);
                    ID(k,1) = RadarMatrix.(obj_ID)(kk,1);
                    y(k,2) =  RadarMatrix.(obj_y)(kk,2);
                    y(k,1) =  RadarMatrix.(obj_y)(kk,1);
                    x(k,2) =  RadarMatrix.(obj_x)(kk,2);
                    x(k,1) =  RadarMatrix.(obj_x)(kk,1);
                    cls(k,2) =  RadarMatrix.(obj_cls)(kk,2);
                    cls(k,1) =  RadarMatrix.(obj_cls)(kk,1);
                    vx(k,2) =  RadarMatrix.(vleo_x)(kk,2);
                    vx(k,1) =  RadarMatrix.(vleo_x)(kk,1);
                    vy(k,2) =  RadarMatrix.(vleo_y)(kk,2);
                    vy(k,1) =  RadarMatrix.(vleo_y)(kk,1);
                    ax(k,2) =  RadarMatrix.(obj_ax)(kk,2);
                    ax(k,1) =  RadarMatrix.(obj_ax)(kk,1);
                    DynProp(k,2) =  RadarMatrix.(obj_dyn_prop)(kk,2);
                    DynProp(k,1) =  RadarMatrix.(obj_dyn_prop)(kk,1);
                    m = kk;
                    break
                end
            end
        end
        RadarMatrix.(obj_ID) = ID;
        RadarMatrix.(obj_y) = y;
        RadarMatrix.(obj_x)  = x;
        RadarMatrix.(obj_cls)  = cls;
        RadarMatrix.(vleo_x) = vx;
        RadarMatrix.(vleo_y) = vy;
        RadarMatrix.(obj_ax) = ax;
        RadarMatrix.(obj_dyn_prop) = DynProp;
end

% 存雷达信息
for i = 1:mCam-1
    RadarDataSinlge  = frmData(i).RadarFrame.RadarObjectList;
    for j = 1:32
        obj_ID = strcat('Target',num2str(j),"_ID_A");
        if uint8(RadarMatrix.(obj_ID)(i,2))>=255||uint8(RadarMatrix.(obj_ID)(i,2))==0
            continue
        end
        obj_y = strcat('Target',num2str(j),"_Pos_Y");
        obj_x = strcat('Target',num2str(j),"_Pos_X");
        obj_cls = strcat('Target',num2str(j),"_Type");
        vleo_x = strcat('Target',num2str(j),"_Vel_X");
        vleo_y = strcat('Target',num2str(j),"_Vel_Y");
        obj_ax = strcat('Target',num2str(j),"_Accel_X");
        obj_dyn_prop = strcat('Target',num2str(j),"_DynProp");
        %             ProbofExist = strcat('Target',num2str(j),"_ProbOfExis");
        RadarDataSinlge(j).ID = uint8(RadarMatrix.(obj_ID)(i,2));
        RadarDataSinlge(j).y = double(RadarMatrix.(obj_x)(i,2));
        RadarDataSinlge(j).x = double(-RadarMatrix.(obj_y)(i,2));

        if RadarMatrix.(obj_cls)(i,2)==0
            RadarDataSinlge(j).cls = obstacle_cls.unknown;
        elseif RadarMatrix.(obj_cls)(i,2)==1
            RadarDataSinlge(j).cls = obstacle_cls.pedestrian;
        elseif RadarMatrix.(obj_cls)(i,2)==2
            RadarDataSinlge(j).cls = obstacle_cls.bicycle;
        elseif RadarMatrix.(obj_cls)(i,2)==3
            RadarDataSinlge(j).cls = obstacle_cls.car;
        elseif RadarMatrix.(obj_cls)(i,2)==4
            RadarDataSinlge(j).cls = obstacle_cls.truck;
        end

        RadarDataSinlge(j).velo_y = double(RadarMatrix.(vleo_x)(i,2));
        RadarDataSinlge(j).velo_x = double(RadarMatrix.(vleo_y)(i,2));
        RadarDataSinlge(j).ay = double(RadarMatrix.(obj_ax)(i,2));
        RadarDataSinlge(j).moveType = double(RadarMatrix.(obj_dyn_prop)(i,2));
        RadarDataSinlge(j).flag = uint8(1);
        %             RadarDataSinlge(j).obj_y_arel = double(RadarMatrix.(obj_y_arel)(i,2));
        frmData(i).RadarFrame.RadarObjectList = RadarDataSinlge;
    end

end

%存整车信息carSignals

    [k1,~] = size(vehiclematrix.EBC2_FrontAxleSpeed);
    [k2,~] = size(vehiclematrix.VDC2__VDC2_YawRate);
    VehSpeed = zeros(mCam,2);
    YawRate = zeros(mCam,2);
    carSignals = zeros(mCam,2);
        m_speed = 1;
        m_yaw = 1;
        for k = 1:mCam-1
            for kk = m_speed:k1
%                 if abs(RadarMatrix.VehInfo1__VehSpeed(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
%                     VehSpeed(k,2) = RadarMatrix.VehInfo1__VehSpeed(kk,2);
%                     VehSpeed(k,1) = RadarMatrix.VehInfo1__VehSpeed(kk,1);
%                     YawRate(k,2) =  RadarMatrix.VehInfo1__YawRate(kk,2);
%                     YawRate(k,1) =  RadarMatrix.VehInfo1__YawRate(kk,1);
%                     m = kk;
%                     break
%                 end
                if abs(vehiclematrix.EBC2_FrontAxleSpeed(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
                    VehSpeed(k,2) = vehiclematrix.EBC2_FrontAxleSpeed(kk,2);
                    VehSpeed(k,1) = vehiclematrix.EBC2_FrontAxleSpeed(kk,1);
                    m_speed = kk;
                    break
                end
            end
           for kk = m_yaw:k2
                if abs(vehiclematrix.VDC2__VDC2_YawRate(kk,1)-CameraMatrix.Ext0_image_lane(k,1))<0.06
                    YawRate(k,2) =  vehiclematrix.VDC2__VDC2_YawRate(kk,2);
                    YawRate(k,1) =  vehiclematrix.VDC2__VDC2_YawRate(kk,1);
                    m_yaw = kk;
                    break
                end
            end
        end
        carSignals(:,1) = VehSpeed(:,2);
        carSignals(:,2) = YawRate(:,2);
% speed = VehSpeed(:,2);
% yawrate = YawRate(:,2);
% carSignals=table(speed,yawrate);
end

