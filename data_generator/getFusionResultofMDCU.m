function [frmDataMDCU] = getFusionResultofMDCU(MDCU_fusion,CameraMatrix)
%处理MDCU融合输出结果
[m1,~] = size(CameraMatrix.ObstacleStatus__NumberOfObstacles);
[jjj,~] = size(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeY);
frmDataMDCU = repmat(obstacle_const.FrameData,m1,1);
MDCU_ID = zeros(m1,2);
MDCU_x = zeros(m1,2);
MDCU_y = zeros(m1,2);
MDCU_vx = zeros(m1,2);
MDCU_vy = zeros(m1,2);
m_MDCU=1;
for i = 1:m1-1
    % MDCU_SOC融合结果与摄像头时间对齐
    for ii= m_MDCU:jjj
               if abs(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeY(ii,1)-CameraMatrix.ObstacleStatus__NumberOfObstacles(i,1))<0.06
                    MDCU_ID(i,2) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsId(ii,2);
                    MDCU_ID(i,1) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsId(ii,1);
                    MDCU_x(i,2) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeX(ii,2);
                    MDCU_x(i,1) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeX(ii,1);
                    MDCU_y(i,2) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeY(ii,2);
                    MDCU_y(i,1) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeY(ii,1);
                    MDCU_vx(i,2) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVx(ii,2);
                    MDCU_vx(i,1) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVx(ii,1);
                    MDCU_vy(i,2) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVy(ii,2);
                    MDCU_vy(i,1) = MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVy(ii,1);
                    m_MDCU = ii;
                    break
               end
    end
end
        MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsId = MDCU_ID;
        MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeX = MDCU_x;
        MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeY = MDCU_y;
        MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVx = MDCU_vx;
        MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVy = MDCU_vy;
%存MDCU融合结果
for i_MDCU = 1:m1-1
    CameraDataSinlge  = frmDataMDCU(i_MDCU).CameraFrame.CameraObjectList;    
    
    CameraDataSinlge(1).ID = uint8(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsId(i_MDCU,2));
    CameraDataSinlge(1).x = double(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeX(i_MDCU,2));
    CameraDataSinlge(1).y = double(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsRangeY(i_MDCU,2));
    CameraDataSinlge(1).velo_x = double(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVx(i_MDCU,2));
    CameraDataSinlge(1).velo_y = double(MDCU_fusion.SOC_Fusion_Msg1__SOC_Fusion_Msg1_ObsVy(i_MDCU,2));

    CameraDataSinlge(1).flag = uint8(1);

    frmDataMDCU(i_MDCU).CameraFrame.CameraObjectList = CameraDataSinlge;
end
end

