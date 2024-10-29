% Description: 级联匹配配定义常量参数-非机动车
% Author: Huang ChunShu
% log:
% 20230310 : created 
%****************************************************************%
function param = getCasConstPeds()
% 级联匹配配定义常量参数
% 非机动车匹配参数

param.maxDist               = single(1e4);                      % 默认的世界坐标下最大距离
param.maxLostAge            = uint8(20);                        % ≥ LST
param.stableSta             = uint8(track_status.stable);       % 目标跟踪的稳定状态
param.uvThreshold           = single(12);                       % uv坐标下的距离阈值
param.featureWeight         = [100, 100, 1, 1, 10, 10, 1, 1];    % 相似权重[x,y,h,w,u,v,at,s]
param.lateralDistThres      = 1.5;                              % 关联门横向大小；
param.cosThreshold          = 0.99;                             % 相似度阈值
param.IOUThreshold          = 0.20;                             % Iou阈值.

end