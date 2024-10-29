# 背景

该框架适合单模态跟踪以及双模态融合跟踪

# 程序运行

1.主函数为app/main.m文件

2.测试数据处于dataset中；

3.移植过程中，需修改load_data/get_local_path_const.m中数据路径：

` dat_path = 'G:\02_Prj\code\new_fusion\dataset\2024-09-25\';  % 源数据路径：.mat, .asc`

4.app/main.m中更新自车车身参数

5.display/initialVideo.m中修改frmID_end，frmID_start在main.m中定义

6.

# 注释

% Description: 
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 