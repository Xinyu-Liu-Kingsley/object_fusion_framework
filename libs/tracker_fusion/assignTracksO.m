% Description: 机动车匹配算法
% Author: Zhou Yanozong
% log:
% 20221130 : created
% 20221221: 1.numM修改为obstacle_const.max_obs，清除不需要的distMatrix
%           2.横向限制在2.0m
% 20230313：更新级联匹配，适合机动车与非机动车共用
%****************************************************************%
function [tracks, measflags, trsFlags] = assignTracksO(tracks, obs, measflags)
% 车辆匹配
% tracks:跟踪库
% obs:观测量detections, 以flag==1提取
% flags:detections是否关联的标记, 1-关联，0-未关联

% 定义常量
numM  = obstacle_const.max_obs_fuse;
numT  = obstacle_const.track_width;
param = getCasConstVehs();


% 转换级联匹配模块输入
[tracks_cas, dets_cas] = busConvert2cas(tracks, obs, numT, numM);

% 级联匹配模块
matchM = cascadeFun(tracks_cas, dets_cas, numT, numM, param);

% update associated objs flag
[measflags, trsFlags, tracks] = flagAssociatedTracks(tracks, obs, matchM, measflags);

end