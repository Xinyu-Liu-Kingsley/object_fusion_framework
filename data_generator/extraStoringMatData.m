function extraStoringMatData(bin_file,file,dat_path)
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
[path, name, ~] = fileparts(bin_file);
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

datHeaderID = fopen(fullfile(dat_path, [name, '.datHeader']), 'w+');
if datHeaderID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.datHeader'])); end

datCarInfo = fopen(fullfile(dat_path,[name,'.datCarInfo']),'w+');
if datCarInfo ==-1, error('打开%s失败！', fullfile(dat_path, [name, '.datCarInfo'])); end


% 新增 抠图尺寸
% cropSizeLaneID = fopen(fullfile(dat_path, [name, '.cropSizeLane']), 'w+');
% if cropSizeLaneID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.cropSizeLane'])); end


if ~isempty(file)
    load(file,'lane_crop','lane_resize','veh_crop','veh_resize','ped_crop','ped_resize', ...
        'param_veh','param_lane','param_ped','lane_cross_seg','header_info','time_stamp')
%     assert(numel(lane_crop) == numel(lane_resize))
%     assert(numel(veh_crop) == numel(veh_resize))
%     assert(numel(ped_crop) == numel(ped_resize))
    n = numel(lane_resize);
    % 应对protobuf 生成mat过程中异常；
    if numel(size(ped_crop))==3 && size(ped_crop,3)==0||numel(ped_crop)==0
        ped_crop = cell(n,1);
    end
    if numel(size(ped_resize))==3 && size(ped_resize,3)==0||numel(ped_resize)==0
        ped_resize = cell(n,1);
    end
    % 读取、写入版本号
    soft_version = header_info{1};
    fwrite(datHeaderID, soft_version, 'char');
    % 按照版本号判断文件名
    if sum(soft_version) >= sum('1.0.5')
        cropSizeAndSegID = fopen(fullfile(dat_path, [name, '.cropSizeAndSeg']), 'w+');
        if cropSizeAndSegID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.cropSizeAndSeg'])); end
    else
        cropSizeAndSegID = fopen(fullfile(dat_path, [name, '.cropSizeObs']), 'w+');
        if cropSizeAndSegID == -1, error('打开%s失败！', fullfile(dat_path, [name, '.cropSizeObs'])); end
    end

    idx_start = 1;
    for i = idx_start:n
        %% ================= 处理车道线 =======================================
        if ~isempty(lane_resize) && ~isempty(lane_crop)
            points_lane = [lane_resize{i}; lane_crop{i}];
        elseif isempty(lane_resize) && ~isempty(lane_crop)
            lane_crop_size = size(lane_crop);
            lane_resize_temp = zeros(lane_crop_size(1),lane_crop_size(2));
            points_lane = [lane_resize_temp; lane_crop{i}];
        elseif ~isempty(lane_resize) && isempty(lane_crop)
            lane_resize_size = size(lane_resize);
            lane_crop_temp = zeros(lane_resize_size(1),lane_resize_size(2));  % 补零；
            points_lane = [lane_resize{i}; lane_crop_temp];
        else
            points_lane = [lane_resize{i}; lane_crop{i}];   % ???           
        end
        
        for j=1:size(points_lane,1)
            fwrite(datLaneID, int32(points_lane(j,:) * aux_const.factor4storing), 'int32');
        end
        fwrite(idxLaneID, int32([i, size(lane_resize{i},1), size(lane_crop_temp,1)]), 'int32');
        %% ============= 处理车辆 ===================================================
        if ~isempty(veh_resize) && ~isempty(veh_crop)
            veh_crop_temp = veh_crop{i};
            points_veh = [veh_resize{i}; veh_crop_temp];
        elseif isempty(veh_resize) && ~isempty(veh_crop)
            veh_crop_temp = veh_crop{i};
            points_veh = [veh_resize{i}; veh_crop_temp];
        elseif ~isempty(veh_resize) && isempty(veh_crop)
            veh_resize_size = size(veh_resize{i});
            veh_crop_temp = zeros(veh_resize_size(1),veh_resize_size(2));
            points_veh = [veh_resize{i}; veh_crop_temp];
        else
            veh_crop_temp = veh_crop{i};
            points_veh = [veh_resize{i}; veh_crop_temp];
        end

        for j=1:size(points_veh,1)
            fwrite(datObsID, int32(points_veh(j,:) * aux_const.factor4storing), 'int32');
        end
        fwrite(idxObsID, int32([i, size(veh_resize{i},1), size(veh_crop_temp,1)]), 'int32');
        %% ============ 处理行人 ===================================================
        if ~isempty(ped_resize) && ~isempty(ped_crop)
            ped_crop_temp = ped_crop{i};
            points_ped = [ped_resize{i}; ped_crop_temp];
        elseif isempty(ped_resize) && ~isempty(ped_crop)
            ped_crop_temp = ped_crop{i};
            points_ped = [ped_resize{i}; ped_crop_temp];
        elseif ~isempty(ped_resize) && isempty(ped_crop)
            ped_resize_size = size(ped_resize);
            ped_crop_temp = zeros(ped_resize_size(1),ped_resize_size(2));
            points_ped = [ped_resize{i}; ped_crop_temp];
        else
            ped_crop_temp = ped_crop{i};
            points_ped = [ped_resize{i}; ped_crop_temp];
        end

        for j=1:size(points_ped,1)
            fwrite(datPedsID, int32(points_ped(j,:) * aux_const.factor4storing), 'int32');
        end
        fwrite(idxPedsID, int32([i, size(ped_resize{i},1), size(ped_crop_temp,1)]), 'int32');
        %% ====================detections_crop_size=================================
        %temp = param_veh{1}.pos_crop_veh;
        temp_veh = [param_veh{i,1},param_veh{i,2},param_veh{i,3},param_veh{i,4}];
        if sum(soft_version) >= sum('1.0.5')
            temp_lane_cross_seg = [lane_cross_seg{i,1},lane_cross_seg{i,2}];
            temp_ped = [param_ped{i,1},param_ped{i,2},param_ped{i,3},param_ped{i,4}];
            temp_lane = [param_lane{i,1},param_lane{i,2},param_lane{i,3},param_lane{i,4}];
            fwrite(cropSizeAndSegID,[int32(i),int32(temp_veh),int32(temp_ped),int32(temp_lane),int32(temp_lane_cross_seg)]...
            * aux_const.factor4storing, 'int32');
        else
            fwrite(cropSizeAndSegID,[int32(i),int32(temp_veh)]* aux_const.factor4storing, 'int32');
        end   
        
    end
    %% 记录附加信息
    % 帧数偏置
    if sum(soft_version) >= sum('1.0.5')
        if exist('time_stamp')
            time_stamp = time_stamp{1}; 
            fwrite(cropSizeAndSegID,[int32(i+1),int32(time_stamp)]* aux_const.factor4storing, 'int32');
        end
    end
    

    fclose(datHeaderID);
    fclose(datLaneID);
    fclose(idxLaneID);
    fclose(datObsID);
    fclose(idxObsID);
    fclose(idxPedsID);
    fclose(datPedsID);
    % fclose(cropSizeLaneID);
    fclose(cropSizeAndSegID);

end






