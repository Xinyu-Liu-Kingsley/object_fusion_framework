function extraStoringMatDataModel(video_file,file,dat_path)
% 用于解析来自模型推理的结果，其中主要补充车道线高度信息；

% 将神经网联检测结果（.mat）保存为本地文件（.datLane、.idxLane、.datObs、.idxObs）
% 将神经网络输出解码后的数据进行存储为本地二进制文件

% 当前视觉组采用大小图的方式，车道线检测结果为points_crop、points_resize
% 目标框检测结果为detections_crop、points_resize;

% mat文件格式：
%   车道线数据文件：
%       points_crop     -   1 x N cell
%           points_crop{i}  -   N x 9   matrix
%                               [conf u v ID cls theta feature1 feature2 feature3]
%       points_resize   -   同points_crop
%   障碍物数据文件：
%       detections      -   1 x N cell
%           detections{i}   -   N x 11  matrix
%                               [conf cls u1 u2 v1 v2 ,r]
%
% idxLane文件格式。每行由10个int32组成，分别为：帧号、车道线数量、第1车道线点数、...、第8车道线点数
% frameID   pts1    pts2
%
% datLane文件格式。每行由9个int32组成，每行对应一个车道线检测点
% [conf     u     v     ID    cls   theta     feature1    feature2    feature3] * aux_const.factor4loading
%
%
% idxObs文件格式。每行由2个int32组成，分别为：帧号、障碍物数量 detections_crop detections_resize
% frameID   ObsNum1 ObsNum2
%
% datObs文件格式。每行由7个int32组成，每行对应一个障碍物
% [conf     u1  v1  u2  v2  cls     r] * aux_const.factor4loading

% corpSize文件格式，每行由4个int32组成，每行对应一组裁剪尺寸；

narginchk(3, 4);

% 视频文件路径
[path, name, ~] = fileparts(video_file);
% video_file
% vid = VideoReader(video_file);
% num = round(vid.Duration * vid.FrameRate);  % 帧数
if nargin < 3
    dat_path = path;
end

datLaneID = fopen(fullfile(dat_path, [name, '.datLane']), 'w+');
if datLaneID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.datLane'])); end

idxLaneID = fopen(fullfile(dat_path, [name, '.idxLane']), 'w+');
if idxLaneID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.idxLane'])); end

datObsID = fopen(fullfile(dat_path, [name, '.datObs']), 'w+');
if datObsID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.datObs'])); end

idxObsID = fopen(fullfile(dat_path, [name, '.idxObs']), 'w+');
if idxObsID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.idxObs'])); end


datPedsID = fopen(fullfile(dat_path, [name, '.datPeds']), 'w+');
if datPedsID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.datPeds'])); end

idxPedsID = fopen(fullfile(dat_path, [name, '.idxPeds']), 'w+');
if idxPedsID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.idxPeds'])); end

% 新增 抠图尺寸
% cropSizeLaneID = fopen(fullfile(dat_path, [name, '.cropSizeLane']), 'w+');
% if cropSizeLaneID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.cropSizeLane'])); end
cropSizeObsID = fopen(fullfile(dat_path, [name, '.cropSizeObs']), 'w+');
if cropSizeObsID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.cropSizeObs'])); end

if ~isempty(file)
    load(file,'lane_crop','lane_resize','veh_crop','veh_resize','ped_crop','ped_resize','param_veh')
    assert(numel(lane_crop) == numel(lane_resize))
    assert(numel(veh_crop) == numel(veh_resize))
    assert(size(ped_crop,1) == numel(ped_resize))
    n = numel(lane_resize);

    idx_start = 1;
    for i = idx_start:n
        %% ================= 处理车道线 =======================================
        if ~isempty(lane_resize{i})
            ptsResize = [lane_resize{i},zeros(size(lane_resize{i},1),1)];
        else
            ptsResize=[];
        end
        if ~isempty(lane_crop{i})
            ptsCrop = [lane_crop{i},zeros(size(lane_crop{i},1),1)];
        else
            ptsCrop = [];
        end
        points_lane = [ptsResize; ptsCrop];
        for j=1:size(points_lane,1)
            fwrite(datLaneID, int32(points_lane(j,:) * aux_const.factor4storing), 'int32');
        end
        fwrite(idxLaneID, int32([i, size(lane_resize{i},1), size(lane_crop{i},1)]), 'int32');
        %% ============= 处理车辆 ===================================================
        points_veh = [veh_resize{i}; veh_crop{i}];
        for j=1:size(points_veh,1)
            fwrite(datObsID, int32(points_veh(j,:) * aux_const.factor4storing), 'int32');
        end
        fwrite(idxObsID, int32([i, size(veh_resize{i},1), size(veh_crop{i},1)]), 'int32');
        %% ============ 处理行人 ===================================================
        if ~isempty(ped_resize)&&~isempty(ped_crop)
            points_ped = [ped_resize{i}; ped_crop{i}]; %
            for j=1:size(points_ped,1)
                fwrite(datPedsID, int32(points_ped(j,:) * aux_const.factor4storing), 'int32');
            end
            fwrite(idxPedsID, int32([i, size(ped_resize{i},1), size(ped_crop{i},1)]), 'int32');
        end
        %% ====================detections_crop_size=================================
        temp = param_veh{1}.pos_crop_veh;
        temp(3) = param_veh{1}.pos_crop_veh(3)-param_veh{1}.pos_crop_veh(1);
        temp(4) = param_veh{1}.pos_crop_veh(4)-param_veh{1}.pos_crop_veh(2);
        fwrite(cropSizeObsID,[int32(i),int32(temp)]* aux_const.factor4storing, 'int32');
    end

    fclose(datLaneID);
    fclose(idxLaneID);
    fclose(datObsID);
    fclose(idxObsID);
    fclose(idxPedsID);
    fclose(datPedsID);
    fclose(cropSizeObsID);

end






