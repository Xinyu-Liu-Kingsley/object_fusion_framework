function [raw_path,dat_path,vid_path] = get_local_path_const(dat_path,raw_path)
% 获取本地路径常量
%   为避免因不同计算机路径设置造成不必要的git仓库变更，路径常量应以mat文件保存
%   至本地。若该文件不存在则通过用户选择创建之。
%   路径配置文件名称为local_path.mat，该函数管理路径包括：
%   1. raw_path 原始数据路径
%   2. dat_path data文件路径
%   3. vid_path 视频文件路径
% 功能：
%   1. 返回路径常量
%   2. 将路径常量导入工作区
root_prj = matlab.project.rootProject;
if isempty(root_prj)
    [ST, I] = dbstack('-completenames', 1);
    [path, ~, ~] = fileparts(ST.file);
    const_file = fullfile(path, 'local_path.mat');
else
    prj = currentProject;
    const_file = fullfile(prj.RootFolder, 'local_path.mat');
end

% const_file = 'local_path.mat';
if ~exist('dat_path')
    dat_path = 'G:\02_Prj\code\new_fusion\dataset\2024-09-25\';  % 源数据路径：.mat, .asc
    raw_path = [dat_path, 'frmData'];
end
vid_path = [dat_path, 'video'];
% try
%     if ~exist(const_file, "file")
%         warning('没有找到本地路径配置文件（%s）！', const_file)
%     else
%         load(const_file, 'raw_path', 'dat_path', 'vid_path');
%     end
% catch
%     warning('装载本地路径配置文件失败!')
% end
% pre_raw_path = raw_path;
% pre_dat_path = dat_path;
% pre_vid_path = vid_path;
% 
% if ~exist('raw_path', 'var') || ~isfolder(raw_path)
%     sel_path = uigetdir(pwd, '请选择源数据路径...');
%     if sel_path ~= 0, raw_path = sel_path; end
% end

% if ~exist('dat_path', 'var') || ~isfolder(dat_path)
%     sel_path = uigetdir(pwd, '请选择data文件路径...');
%     if sel_path ~= 0, dat_path = sel_path; end
% end
% 
% if ~exist('vid_path', 'var') || ~isfolder(vid_path)
%     sel_path = uigetdir(pwd, '请选择视频文件路径...');
%     if sel_path ~= 0, vid_path = sel_path; end
% end
% 
% if exist(raw_path, 'dir') && exist(dat_path, 'dir') && exist(vid_path, 'dir')
%     assignin("base", 'raw_path', raw_path)
%     assignin("base", 'dat_path', dat_path)
%     assignin("base", 'vid_path', vid_path)
%     if ~strcmp(pre_raw_path, raw_path) || ~strcmp(pre_dat_path, dat_path) || ~strcmp(pre_vid_path, vid_path)
%         try
%             save(const_file, 'raw_path', 'dat_path', 'vid_path', '-mat')
%             fprintf('本地路径配置文件被更新：\n  1. raw_path - %s\n  2. dat_path - %s\n  3. vid_path - %s\n', raw_path, dat_path, vid_path)
%         catch
%             warning('保存本地路径配置文件失败！')
%         end
%     end
% else
%     warning('没有正确设置本地路径配置文件！')
% end
% 
% 
% end

