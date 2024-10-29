clear

%% ================ 请确认传感器类型和是否保存 ======================= %%
sensorFlag = 3;  % 0-处理Raimo摄像头和保隆雷达数据;1-处理极目摄像头和行易道雷达数据;
                 % 2-处理Mobileye摄像头和保隆雷达数据;3-处理为升科角雷达数据；
                 % 4-处理黑芝麻摄像头和保隆雷达数据
MDCU_flag = 0;   % 0-不处理MDCU融合结果；1-处理实车MDCU_SOC融合输出结果
saveFlag = 1;    % 0-不保存数据；1-保存数据

%% ====================== 请设置数据路径 ============================== %%
base_path ='Z:\规划控制部\为升科';       %场景库地址
scene_path = [base_path, '\20231218'];   %场景地址
data_path = [scene_path, '\mat'];            %源数据地址
save_path = [scene_path, '\frmData'];        %处理后的数据存放地址
blf_path = [scene_path, '\blf'];             %报文地址

%% ======================== 批量处理数据 ============================== %%
files = dir(fullfile(blf_path, '*.blf'));
data_start = int32(1);
data_end = int32(length(files));
% data_No = data_start:data_end;              %要处理的数据序号
data_No = 16;  
for No=data_No(1):data_No(end)
    [~, name, ~] = fileparts(files(No).name);
    raw_data = load(fullfile(data_path, name));                %加载原始数据（blf———>mat）
%     radar_name = strcat(name,"_radarmatrix");
%     camera_name = strcat(name,"_cameramatrix");
%     vehicle_name = strcat(name,"_vehiclematrix");
    frmData_name = strcat(name,"_frmData");
    carSignals_name = strcat(name,"_carSignals");
%     CameraMatrix = load(fullfile(data_path, camera_name));      %加载摄像头数据
%     RadarMatrix = load(fullfile(data_path, radar_name));        %加载雷达数据
%     vehiclematrix = load(fullfile(data_path, vehicle_name));    %加载整车数据
%     if sensorFlag == 3
%     CornerRadar_name = strcat(name,"_weishengkematrix");
%     CornerRadarfrmData_name = strcat(name,"_cornerradarfrmData");
%     CornerRadarMatrix=load(fullfile(data_path, CornerRadar_name));           %加载角雷达数据
%     end
    if MDCU_flag == 1
    MDCU_name = strcat(name,"_MDCU");
    MDCUfrmData_name = strcat(name,"_MDCUfrmData");
    MDCU_fusion=load(fullfile(data_path, MDCU_name));           %加载MDCU数据
    end
%% 处理Raimo摄像头和保隆雷达数据
if sensorFlag==0
    [frmData,carSignals] = getRaimoAndBaolong(CameraMatrix, RadarMatrix, vehiclematrix);
end

%% 处理极目摄像头和行易道雷达数据
if sensorFlag==1
    [frmData,carSignals] = getJimuAndXingyidao(CameraMatrix, RadarMatrix, vehiclematrix);
end

%% 处理Mobileye摄像头和保隆雷达数据
if sensorFlag==2
    [frmData,carSignals] = getMobileyeAndBaolong(CameraMatrix, RadarMatrix, vehiclematrix);
end

%% 处理为升科角雷达数据
if sensorFlag==3
    [frmData,carSignals] = getWeiShengKeRadar(raw_data);
end

%% 处理黑芝麻摄像头和保隆雷达数据
if sensorFlag==4
    [frmData,carSignals] = getHeiZhiMaAndBaoLong(raw_data);
end

%% 处理MDCU融合输出结果
if MDCU_flag==1
    [frmDataMDCU] = getFusionResultofMDCU(MDCU_fusion,CameraMatrix);
end 

%% 保存数据
if saveFlag==1
   save(fullfile(save_path, frmData_name),'frmData');
   save(fullfile(save_path, carSignals_name),'carSignals');
  if MDCU_flag == 1
   save(fullfile(save_path, MDCUfrmData_name),'frmDataMDCU');
  end
end

end