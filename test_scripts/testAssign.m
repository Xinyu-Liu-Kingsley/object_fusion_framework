% 生成随机的雷达和相机数据
radarData = generateRadarData();
cameraData = generateCameraData();

% 初始化障碍物数组
obstacles = struct('x', [], 'y', [], 'vx', [], 'vy', [], 'ax', [], 'ay', [], 'velo_x', [], 'velo_y', [], 'obj_det_prop', [], 'img_lane', [], 'flag', []);

% 将雷达数据添加到障碍物数组
for i = 1:length(radarData.x)
    obstacles(i).x = radarData.x(i);
    obstacles(i).y = radarData.y(i);
    obstacles(i).vx = radarData.vx(i);
    obstacles(i).vy = radarData.vy(i);
    obstacles(i).ax = radarData.ax(i);
    obstacles(i).ay = radarData.ay(i);
    obstacles(i).velo_x = radarData.velo_x(i);
    obstacles(i).velo_y = radarData.velo_y(i);
    obstacles(i).obj_det_prop = obs_det_prop.sole_radar;
    obstacles(i).img_lane = obj_lane.undefined;
    obstacles(i).flag = 0;
end

% 使用匈牙利算法进行雷达和相机目标的匹配
costMatrix = computeCostMatrix(radarData, cameraData);
assignment = munkres(costMatrix);
invalid_idx = -1;
assignment(assignment <= 0) = invalid_idx;
% 将匹配结果添加到障碍物数组
for i = 1:length(assignment)
    if assignment(i) == invalid_idx
        % 如果索引无效，则将所有字段设置为默认值
        obstacles(i).x = NaN;
        obstacles(i).y = NaN;
        obstacles(i).vx = NaN;
        obstacles(i).vy = NaN;
        obstacles(i).img_lane = obj_lane.undefined;
        obstacles(i).flag = 0;
    else
        obstacles(i).x = cameraData.x(assignment(i));
        obstacles(i).y = cameraData.y(assignment(i));
        obstacles(i).vx = cameraData.vx(assignment(i));
        obstacles(i).vy = cameraData.vy(assignment(i));
        obstacles(i).img_lane = obj_lane.undefined;
        obstacles(i).flag = 1;
    end
end

% 可视化匹配结果
hold on;
markerSize = 10;

% 绘制雷达目标
for i = 1:length(radarData.x)
    plot(radarData.x(i), radarData.y(i), 'ro', 'MarkerSize', markerSize);
    text(radarData.x(i), radarData.y(i), num2str(i), 'Color', 'r');
end

% 绘制相机目标
for i = 1:length(cameraData.x)
    plot(cameraData.x(i), cameraData.y(i), 'bo', 'MarkerSize', markerSize);
    text(cameraData.x(i), cameraData.y(i), num2str(i), 'Color', 'b');
end

% 绘制匹配上的目标
for i = 1:length(assignment)
    plot(obstacles(i).x, obstacles(i).y, 'gx', 'MarkerSize', markerSize);
    text(obstacles(i).x, obstacles(i).y, num2str(i), 'Color', 'g');
end

% 绘制未匹配上的目标
for i = length(assignment)+1:length(obstacles)
    if obstacles(i).obj_det_prop == obs_det_prop.sole_radar
        plot(obstacles(i).x, obstacles(i).y, 'ms', 'MarkerSize', markerSize);
        text(obstacles(i).x, obstacles(i).y, num2str(i), 'Color', 'm');
    else
        plot(obstacles(i).x, obstacles(i).y, 'cs', 'MarkerSize', markerSize);
        text(obstacles(i).x, obstacles(i).y, num2str(i), 'Color', 'c');
    end
end

hold off;

% 设置图例
legend('Radar Targets', 'Camera Targets', 'Matched Targets', 'Unmatched Radar Targets', 'Unmatched Camera Targets');

% 设置坐标轴和标题
axis equal;
xlabel('X');
ylabel('Y');
title('Matching Results');

% 生成随机的雷达数据
function radarData = generateRadarData()
    numTargets = 5;
    radarData.x = rand(1, numTargets) * 100;
    radarData.y = rand(1, numTargets) * 100;
    radarData.vx = rand(1, numTargets) * 10 - 5;
    radarData.vy = rand(1, numTargets) * 10 - 5;
    radarData.ax = rand(1, numTargets) * 2 - 1;
    radarData.ay = rand(1, numTargets) * 2 - 1;
    radarData.velo_x = rand(1, numTargets) * 10 - 5;
    radarData.velo_y = rand(1, numTargets) * 10 - 5;
end

% 生成随机的相机数据
function cameraData = generateCameraData()
    numTargets = 5;
    cameraData.x = rand(1, numTargets) * 100;
    cameraData.y = rand(1, numTargets) * 100;
    cameraData.vx = rand(1, numTargets) * 10 - 5;
    cameraData.vy = rand(1, numTargets) * 10 - 5;
    cameraData.ax = rand(1, numTargets) * 2 - 1;
    cameraData.ay = rand(1, numTargets) * 2 - 1;
    cameraData.velo_x = rand(1, numTargets) * 10 - 5;
    cameraData.velo_y = rand(1, numTargets) * 10 - 5;
end

% 计算匹配代价矩阵
function costMatrix = computeCostMatrix(radarData, cameraData)
    numRadarTargets = length(radarData.x);
    numCameraTargets = length(cameraData.x);
    costMatrix = zeros(numRadarTargets, numCameraTargets);
    for i = 1:numRadarTargets
        for j = 1:numCameraTargets
            % 这里使用目标之间的欧氏距离作为匹配代价
            dx = radarData.x(i) - cameraData.x(j);
            dy = radarData.y(i) - cameraData.y(j);
            costMatrix(i, j) = sqrt(dx^2 + dy^2);
        end
    end
end



