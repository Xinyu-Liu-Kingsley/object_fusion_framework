function tracks = updateMotionStatus(tracks,index)
% rel_status
%    1. 静止
%    2. 靠近
%    3. 远离    
%    4. 停止
%    5. 横穿 
% 判断目标的运动状态

coder.inline('always');
global g_ego_status;
% maxTargetVx = 150; % kph
% g_ego_status;
y_history = tracks.status(index).y_history;
x_history = tracks.status(index).x_history;
vx_history = tracks.status(index).vx_history;
vy_history = tracks.status(index).vy_history;
tracks.status(index).y_history(end) = tracks.estimated(index).y;
tracks.status(index).y_history(1:end-1) = y_history(2:end);
tracks.status(index).x_history(end) = tracks.estimated(index).x;
tracks.status(index).x_history(1:end-1) = x_history(2:end);
tracks.status(index).vy_history(end) = tracks.estimated(index).velo_y;
tracks.status(index).vy_history(1:end-1) = vy_history(2:end);
tracks.status(index).vx_history(end) = tracks.estimated(index).velo_x;
tracks.status(index).vx_history(1:end-1) = vx_history(2:end);
deltaD_longitude = tracks.status(index).y_history(end)-tracks.status(index).y_history(1);
deltaD_latitude = abs(tracks.status(index).x_history(end)-tracks.status(index).x_history(1));
frmCNT = 10;
moveD = mean(g_ego_status.vx_history)*(obstacle_const.cfg.deltaT)*frmCNT;
maxCNT = 15;
cntThres = 10;
%  设定动静判断的阈值，行人和车辆分开
if tracks.status(index).fusedCls == obstacle_cls.pedestrian
    thres_moving =  obstacle_const.cfg.thres_ped_moving;
else
    thres_moving =  obstacle_const.cfg.thres_veh_moving;
end

if deltaD_longitude < thres_moving % 目标车速小于自车速
    if abs(deltaD_longitude+moveD)<2 ||... 
        abs(mean(g_ego_status.vx_history)+mean(vy_history))<2      %目标车静止
        tracks.output(index).motionStatus = obstacle_motionStatus.stationary;
        rel_status = uint8(1); % 静止
    else
        tracks.output(index).motionStatus = obstacle_motionStatus.moving;
        tracks.output(index).motionCategory = obstacle_motionCategory.MOVING_IN; %靠近
        rel_status = uint8(2); % 靠近
    end
else  % 目标车速大于等于自车速
    rel_status = uint8(3);     % 远离
    tracks.output(index).motionStatus = obstacle_motionStatus.moving;
    tracks.output(index).motionCategory = obstacle_motionCategory.MOVING_OUT; % 远离
end

%判断为停止目标
absolute_velocity = tracks.status(index).vy_history+g_ego_status.vx_history;
cntThresDec = 0;    % 减速帧数
cntThresStand = 0;  % 停止帧数
for i = 1:cntThres-1
    delta_velocity = absolute_velocity(i+1) - absolute_velocity(i);
    if delta_velocity < 0 && abs(delta_velocity)>=2  % 减速，相邻量两帧减速大于2
        cntThresDec = cntThresDec+1;
    elseif abs(delta_velocity)<2
        cntThresStand = cntThresStand+1;
    elseif delta_velocity>0 && abs(delta_velocity)>=2
        cntThresStand = cntThresStand-1;
    end
end
% 减速帧数大于5，停止帧数大于3，认为是停止状态
% 或者之前是停止状态，当前时刻静止，认为是停止状态
% try
if cntThresDec>=5 && cntThresStand >=3 || ...
        (tracks.status(index).rel_status(2)==uint8(4) && cntThresStand >=6)
    rel_status = uint8(4);  % 停止状态
    tracks.output(index).motionStatus = obstacle_motionStatus.stopped; % 停止状态
end
% catch
%     continue;
% end

% 横穿状态
% 横向平均速度大于0.9, 10帧移动距离大于0.4(只在yawate小于5度或者车静止的时候)
if abs(g_ego_status.yawrate)<=5 || abs(g_ego_status.speed)<=2
    crossDis =  mean(tracks.status(index).vx_history)*(obstacle_const.cfg.deltaT)*frmCNT;
    if abs(crossDis)>0.4 &&  abs(mean(tracks.status(index).vx_history) )>0.9 && abs(deltaD_latitude)>0.6
        rel_status = uint8(5);
         tracks.output(index).motionCategory = obstacle_motionCategory.CROSSING; % 横穿状态
         tracks.output(index).motionStatus = obstacle_motionStatus.moving;
    end
end


% 记录上一帧的运动状态
if  tracks.status(index).rel_status(2)==rel_status || tracks.status(index).rel_status(2)==uint8(0)
    tracks.status(index).rel_status(2) = rel_status;
    tracks.status(index).rel_status(1) = min(tracks.status(index).rel_status(1)+uint8(1),maxCNT);
else
    if tracks.status(index).rel_status(1)>cntThres
        tracks.status(index).rel_status(1) = tracks.status(index).rel_status(1)-uint8(1);
    else
        tracks.status(index).rel_status(2) = rel_status;
        tracks.status(index).rel_status(1) = uint8(1);
    end
end

% 使用rel_status
% if tracks.status(i).rel_status(2)==uint8(1) &&  tracks.status(i).rel_status(1)>cntThres
%     tracks.estimated(i).velo_y = -g_ego_status.vx;
% end

% if  direction == uint8(1) % 同向
%     if tracks.status(i).rel_status(2)==uint8(2) &&  tracks.status(i).rel_status(1)>cntThres  %rel_vx<0
%         tracks.estimated(i).rel_vx = max(tracks.estimated(i).rel_vx,-g_ego_status.vx);
%
%     elseif tracks.status(i).rel_status(2)==uint8(3) &&  tracks.status(i).rel_status(1)>cntThres %rel_vx>0
%         tracks.estimated(i).rel_vx = min(tracks.estimated(i).rel_vx,maxTargetVx/3.6-g_ego_status.vx);
%     end
% end

end


