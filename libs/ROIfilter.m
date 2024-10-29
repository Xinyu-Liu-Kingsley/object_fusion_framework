function [flag,obj_lane_radar] = ROIfilter(y,x,cls,leftLane_valid,rightLane_valid,C_Left,C_Right,C_left_range,C_right_range)

% 筛选有效的障碍物
% 筛选原则，左右最近目标，radar_only(应对切入和切出目标), 其他融合状态输出fused
% global  g_cfg_lane
minXL = 150;
minXR = 150;
def_wid_half = 3;
flag = 0;
%% 拟合左右车道线

xl = -def_wid_half;
xr = def_wid_half;
obj_lane_radar = obj_lane.undefined;
if  y<minXL || y<minXR
    % 左右车道线界限
    if C_left_range >= 10
        xl = polyval(C_Left, y);
    end
    if C_right_range>= 10
        xr = polyval(C_Right, y);
    end
end
cls_flg = (cls~= obstacle_cls.leftGuard && cls~= obstacle_cls.rightGuard);
if cls_flg
    flag = 1;
end
%% 判断数据是否有效 判断逻辑 当前车道 或者左车道，右车道
if x < xl && x > (xl-def_wid_half*2) && y <= minXL && cls_flg
    flag = 1;
elseif x > xr && x <(xr+def_wid_half*2) && y <= minXR && cls_flg
    flag = 1;
elseif x > xl && x < xr && cls_flg
    flag = 1;
elseif y >= minXL && cls_flg  % 大于100m
    flag = 1;
end
if leftLane_valid>0 && rightLane_valid>0 % 当左右车道线都没问题
    if x > xl && x < xr && cls_flg       %缩小左右车道阈值，给毫米波雷达赋值
        obj_lane_radar = obj_lane.sameLane;
    end
elseif x > xl/2 && x < xr/2 && cls_flg       %缩小左右车道阈值，给毫米波雷达赋值
    obj_lane_radar = obj_lane.sameLane;  % 在当前车道，给毫米波雷达赋值
end
