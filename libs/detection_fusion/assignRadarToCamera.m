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
for i = 1:numC
    if (any(matchM(i,:) > 0))
        trsId = find(matchM(i,:) > 0);
    else
        continue;
    end
    cameraAssociateFlag(i,1) = uint8(1);
    cameraAssociateFlag(i,2) = trsId(1);
    radarAssociateFlag(trsId(1),1) = uint8(1);
    radarAssociateFlag(trsId(1),2) = uint8(i);
end
end

