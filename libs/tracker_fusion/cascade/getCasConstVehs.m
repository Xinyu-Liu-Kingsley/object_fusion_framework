% Description: 级联匹配配定义常量参数-机动车
% Author: Zhou YanZong
% log:
% 20230727 : created
%****************************************************************%
function param = getCasConstVehs()
% 级联匹配配定义常量参数
% 机动车匹配参数

param.maxDist              = single(1e4);                      % 默认的世界坐标下最大距离
param.maxLostAge           = uint8(20);                        % ≥ LST
param.stableSta            = uint8(track_status.stable);       % 目标跟踪的稳定状态
param.featureWeight        = [0.9, 0.9, 0.1, 0.1];             % 相似权重[x,y,vx,vy]
param.lateralDistThres     = 2.0;                              % 关联门横向大小；
param.cosThreshold         = 10;                               % 相似度阈值
param.y =  [0   20  50  90  150 200];                          % 纵向距离表
param.s_y = [0.12 0.15 0.15 0.15 0.12 0.1]*0.5;                % scale x查表值
param.s_x = [0.8 0.8 0.8 0.7 0.6 0.6];                         % scale y查表值
param.a_y = [8 10 12 20 25 35];                                % 单位：m;关联纵向阈值查表

end