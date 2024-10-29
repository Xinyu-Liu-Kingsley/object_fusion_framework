% Description: 创建全局变量
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
global s_check_arg;                     % 是否检查参数
global s_check_arg_type;                % 是否检查参数类型
global s_check_arg_range;               % 是否检查参数有效范围
global s_check_var_length;              % 是否检查数组长度

global g_disp_info_cls;                 % 显示信息分类
global g_disp_info_spec;                % 显示信息规格

global const_obstacle_max_obs;          % 单帧检测障碍物最大数量
global const_obstacle_track_depth;      % 障碍物追踪深度
global const_obstacle_track_width;      % 障碍物追踪宽度
global const_obstacle_len_1st;          %
global const_obstacle_len_2nd;          %
global const_max_frames_per_file;       % 单个数据文件保存最多帧数

global g_tracks_auto;                   % 障碍物追踪库
global g_tracks_VRU;                    % VRU追踪库

global g_cfg_auto;                 	    % 障碍物车辆检测配置
global g_cfg_VRU;                       % VRU检测配置

global g_ego_params;                   	% 自车参数
global g_ego_status;                    % 自车状态


global g_polynomial_flag;
g_polynomial_flag = uint8(0);

s_check_arg = uint8(1);
s_check_arg_type = uint8(1);
s_check_arg_range = uint8(0);
s_check_var_length = uint8(1);

g_disp_info_cls = uint16(info_cls.general);
g_disp_info_spec = uint8(info_spec.general);



const_obstacle_max_obs = obstacle_const.max_obs;
const_obstacle_track_depth = obstacle_const.track_depth;
const_obstacle_track_width = obstacle_const.track_width;
const_obstacle_len_1st = obstacle_const.len_1st;
const_obstacle_len_2nd = obstacle_const.len_2nd;


const_max_frames_per_file = aux_const.max_frames;

g_tracks_auto = obstacle_const.MatObstacles;
% g_tracks_VRU = peds_const.MatObstacles;


g_cfg_auto = obstacle_const.cfg;
% g_cfg_VRU = peds_const.cfg;

g_ego_params = ego_const.params;
g_ego_status = ego_const.status;

