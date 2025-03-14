function [cameraAssociateFlag, radarAssociateFlag] = assignRadarToCamera(matchM)
% 处理已关联的radar和camera目标
% 输入matchM: 关联矩阵
% 输入frameData:当前帧camera和radar的数据 row(行)：camera Data   cloumn(列)：radarData 
% cameraAssociateFlag:记录radar关联与否，cloumn1：1-关联，0-未关联  cloumn2：关联的index号
% radarAssociateFlag:记录camera关联与否，cloumn1：1-关联，0-未关联  cloumn2：关联的index号

%初始化
numC = size(matchM, 1); % cameraNum
% numR = size(obstacle_const.max_obs_radar, 2); % RadarNum front+rear
cameraAssociateFlag = zeros(numC, 2, 'uint8'); %1.是否关联  2.关联的ID号 
radarAssociateFlag = zeros(obstacle_const.max_obs_radar,2, 'uint8');   %1.是关联
% update flag
for i = 1:numC % 按行遍历
    if (any(matchM(i,:) > 0))  % 该行存在非零元素
        trsId = find(matchM(i,:) > 0); % 该行非零元素的列索引
    else
        continue;
    end
    cameraAssociateFlag(i,1) = uint8(1); % 1 表示关联，0 表示未关联
    cameraAssociateFlag(i,2) = trsId(1); % 关联的雷达目标的索引
    radarAssociateFlag(trsId(1),1) = uint8(1); % 1 表示关联，0 表示未关联
    radarAssociateFlag(trsId(1),2) = uint8(i); % 关联的相机目标的索引
end
end

