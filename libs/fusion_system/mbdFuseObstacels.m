function obstacles= mbdFuseObstacels(frameData)
% Description: 对雷达和摄像头的数据进行融合，返回融合的ObjecetList
% Author :  Shane Liu
% **************************************************************** %
% obstacles_front = repmat(obstacle_const.FusedObject, 1, obstacle_const.max_obs_fuse);
obstacles = repmat(obstacle_const.FusedObject, 1, obstacle_const.max_obs_fuse);
coder.cstructname(obstacles, 'Obstacles')
global g_tracks_auto g_cfg_auto
% ID  Assign
matchID = assignByID(frameData,g_tracks_auto,g_cfg_auto);
% calculate costMatrix
costMatrix = computeCostMatrix(frameData,matchID);
% use munkreas to match
fprintf('munkres >> start : [%+6.2f, %+6.2f]\n',costMatrix(1,1),costMatrix(1,2));
[matching, ~] = munkres(costMatrix);
fprintf('munkres >> end : [%+6.2f, %+6.2f]\n',matching(1),matching(2));
% output match matrix
matchM = mergeMatching(matching,matchID,costMatrix);
% update assgined and unassgined camera and tracks
[cameraAssociateFlag, radarAssociateFlag] = assignRadarToCamera(matchM);
% update obstacle struct
[obstacles] = updateFusedObstacles(cameraAssociateFlag,radarAssociateFlag,frameData);
end

function costMatrix = computeCostMatrix(frameData,matchID)
% 计算损失函数
global g_cfg_auto
costMatrix = ones(obstacle_const.max_obs_camera, obstacle_const.max_obs_radar_front)*10000;
CameraObjectList = frameData.CameraFrame.CameraObjectList;
RadarObjectList =  frameData.RadarFrame.RadarObjectList(1: obstacle_const.max_obs_radar_front);
radarID = 0;  % 记录速度阈值不满足但是距离满足的radarID值
% camID = 0;
costDisRe = 0;  % 需要重置的损失矩阵
fprintf('costMatrixStart >>CameraObjectList y and x : [%+6.2f, %+6.2f]',CameraObjectList(1).y,CameraObjectList(1).x);
for i = 1:length(CameraObjectList)
    cntVelDis = 0; % 距离和速度同时满足阈值的radar个数
    cntDis = 0;   % 只有距离满足阈值的radar个数
    if (any( matchID(i,:) > 0))  % 如果ID assign匹配上,则不参与后续计算
        continue
    end
    for j = 1:length(RadarObjectList)
        if (any(matchID(:,j)>0)) % 如果ID匹配上
            continue
        end
        if RadarObjectList(j).det_src==det_src.left_forward || RadarObjectList(j).det_src==det_src.left_backward ||...
                RadarObjectList(j).det_src==det_src.right_backward || RadarObjectList(j).det_src==det_src.right_forward
            continue
        end % 角雷达数据不参与计算损失矩阵
        if CameraObjectList(i).flag == uint8(1) && RadarObjectList(j).flag == uint8(1) && matchID(i,j)<1                                                % 只计算ID未匹配上的目标
            % 只计算有效数据
            ay = interp1(g_cfg_auto.y,g_cfg_auto.a_y,CameraObjectList(i).y,'linear',g_cfg_auto.a_y(end));
            %             disCost_camRad = abs(CameraObjectList(i).x-RadarObjectList(j).x)+abs(CameraObjectList(i).y-RadarObjectList(j).y);
            velCost_camRad = abs(CameraObjectList(i).velo_x-RadarObjectList(j).velo_x)*0.3 +abs(CameraObjectList(i).velo_y-RadarObjectList(j).velo_y);  % 调整横向速度的阈值
            costDist = computeCost(CameraObjectList(i), RadarObjectList(j),g_cfg_auto.weight);                                                          % 距离和速度限制
            if costDist<= ay && abs(CameraObjectList(i).x-RadarObjectList(j).x)<=2
                if velCost_camRad<ay*0.5 %distance thresHold
                    costMatrix(i, j) = costDist;
                    cntVelDis = cntVelDis+1;
                    %todo 开始计数
                elseif velCost_camRad>ay*0.5 && CameraObjectList(i).y<=25 %
                    % 调整关联阈值，解决航迹新建的时候，关联不上的问题
                    cntDis = cntDis+1;
                    radarID = j;
                    costDisRe = costDist;
                end
            end
        end
    end
    if cntVelDis == 0 && cntDis ==1
        costMatrix(i, radarID) = costDisRe;
    end
end
% 对分裂目标进行重新处理，选择最近的目标做为损失输入
costMatrix = soveSplitingRadarObjects(costMatrix,CameraObjectList,RadarObjectList);
fprintf('costMatrixSucess >> CameraObjectList y and x : [%+6.2f, %+6.2f]',CameraObjectList(1).y,CameraObjectList(1).x);
fprintf('==============================');
end

function cost = computeCost(cameraObject, radarObject,weight)
% 计算马氏距离
% cameraObject: 摄像头目标输出
% radarObject:  radar目标输出
% weight: 权重
% 关联距离计算待细化
x = [cameraObject.x; cameraObject.y; cameraObject.velo_x; cameraObject.velo_y];
y = [radarObject.x; radarObject.y; radarObject.velo_x; radarObject.velo_y];
weightMatrix = (x-y)'.*weight;
% covMatrix = [cameraObject.cov_x, cameraObject.cov_xy, cameraObject.cov_vx, cameraObject.cov_vxy;
%     cameraObject.cov_xy, cameraObject.cov_y, cameraObject.cov_vxy, cameraObject.cov_vy;
%     cameraObject.cov_vx, cameraObject.cov_vxy, cameraObject.cov_vx, cameraObject.cov_vxy;
%     cameraObject.cov_vxy, cameraObject.cov_vy, cameraObject.cov_vxy, cameraObject.cov_vy];
% cost = sqrt((x - y)' * inv(covMatrix) * (x - y));
cost = norm(weightMatrix);
end

function matchM = mergeMatching(matching,matchID,costMatrix)
% matching:匹配结果，匈牙利出来的二进制矩阵，需要构建B指示矩阵滤除干扰
% matchID: ID assign的结果
% costMatrix:距离和速度的结果，越小越相近。

B = zeros(size(costMatrix), 'uint8');
B(costMatrix < 10000) = uint8(1);
matching = B .* uint8(matching);
% 最终匹配结果
matchM = matching | matchID; % 取ID assign和匈牙利
end

function  costMatrix = soveSplitingRadarObjects(costMatrix,CameraObjectList,RadarObjectList)
%% 针对目标分裂做修正，找到分裂目标中，距离最近的目标
% 策略
% S1 找到满足阈值条件的雷达目标ID和个数，
% S2 如果找到的雷达个数大于2，则判断每个雷达目标是否有满足关联阈值的其他摄像头目标（否则会影响全局最近邻匹配）
% S3 若无满足关联阈值的其他摄像头目标，判断雷达目标的速度一致性
% S4 如果一致，则选择最近的雷达目标作为输入
global g_ego_status
for i = 1:size(costMatrix,1)
    % S1 找到满足阈值条件的雷达目标ID和个数
    if CameraObjectList(i).flag == uint8(1)
        indices = find(costMatrix(i, :) < 1000);
    else
        continue
    end
    % S2 判断每个雷达目标是否有满足关联阈值的其他摄像头目标
    if length(indices)<2
        continue
    else
        check_num = 0;
        for j = 1:length(indices)
            indices_camera = find(costMatrix(:,indices(j))<1000);
            if length(indices_camera)==1
                check_num = check_num +1;
            end
        end
        %S3 若无满足关联阈值的其他摄像头目标，判断雷达目标的速度一致性
        %         indices_diff = [];
        %         if check_num == length(indices)
        %             % 初始化变量来存储结果
        % %             indices_diff = [];
        %             for k  = 1:check_num
        %                 for m = k+1:check_num
        %                     diff = abs(RadarObjectList(indices(k)).velo_y-RadarObjectList(indices(m)).velo_y);
        %                     if diff < 1  %temp thresHold
        %                         indices_diff = [indices_diff,k,m];
        %                     end
        %                 end
        %             end
        %             indices_diff = unique(indices_diff);
        %         end
        if check_num >0
            indices_diff = zeros(1, 300);
            index = 1;
            if check_num == length(indices)
                for k = 1:check_num
                    for m = k+1:check_num
                        diff = abs(RadarObjectList(indices(k)).velo_y - RadarObjectList(indices(m)).velo_y);
                        %                         tracks.output(idxR).velo_y + g_ego_status.speed/3.6 >=3
                        if diff < 1  &&  RadarObjectList(indices(k)).velo_y + g_ego_status.speed/3.6 >=3%temp thresHold
                            indices_diff(index) = k;
                            indices_diff(index+1) = m;
                            index = index + 2;
                        end
                    end
                end
            end
            if index>1 % 如果找到同速不同距的目标
                indices_diff = unique(indices_diff(1:index-1));
                %S4 如果一致，则选择最近的雷达目标作为输入
                minRange = 255;
                minindex =255;
                % 找到距离最近的雷达索引
                if ~isempty(indices_diff)
                    for kk = 1:length(indices_diff)
                        if indices_diff(kk) ~=0 && RadarObjectList(indices(indices_diff(kk))).y <minRange
                            minRange = RadarObjectList(indices(indices_diff(kk))).y;
                            minindex = indices(indices_diff(kk));
                        end
                    end
                end
                % 将其他雷达索引的损失置默认值
                if minindex ~= 255 && minRange ~=255
                    for kkk = 1:length(indices_diff)
                        if indices(kkk) ~= minindex
                            costMatrix(i,indices(kkk)) = 1000;
                        end
                    end
                end
            end
        end
    end
end
end
