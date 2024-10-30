function  mbdInitialGlobalVars()
%MBDINITIALGLOBALVARS 此处显示有关此函数的摘要
%   此处显示详细说明
global g_tracks_auto g_ego_status g_cfg_auto
g_tracks_auto = obstacle_const.MatObstacles;
g_ego_status = ego_const.status;
g_cfg_auto = obstacle_const.cfg;
% track_auto = g_tracks_auto;
% tracks_ego = g_ego_status;
end

