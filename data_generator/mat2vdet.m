function mat2vdet(video_file,lane_file,obstacle_file)
% 将陈睐保存的检测结果转换成vdet格式
% 当前视觉组采用大小图的方式，车道线检测结果为points_crop、points_resize
% mat文件格式：
%   车道线数据文件：
%       points_crop     -   1 x N cell
%           points_crop{i}  -   N x 9   matrix
%                               [conf u v ID cls theta feature1 feature2 feature3]
%       points_resize   -   同points_crop
%   障碍物数据文件：
%       detections      -   1 x N cell
%           detections{i}   -   N x 11  matrix
%                               [ ... ]
% 
% vdet文件格式：
%    变量：
%           info - 包含字段：MajorVer, MinorVer, Patch, Describes
%           objects         - conf, cls, bbox, r, cbox
%           lanes           - L_Num, lines
%                               lines   - P_Num, pts, src, rho, feature, cls
%           ParamDetObj     - name, height, width, conf_thres
%           ParamDetLane    - name, height, width, conf_thres
%           ParamVideo      - file, height, width

if ~exist(video_file, 'file')
    return
end

info = struct('MajorVer', 0, 'MinorVer', 0, 'Patch', 0, 'Describes', '');

% 视频文件路径
[path, name, ~] = fileparts(video_file);

vid = VideoReader(video_file);
ParamVideo = struct('file', video_file, 'height', vid.Height, 'width', vid.Width);
num = round(vid.Duration * vid.FrameRate);  % 帧数

ParamDetLane = struct('name','', 'height', 0, 'width', 0, 'conf_thres', 0);
lanes = cell(1, num);
if ~isempty(lane_file)
    load(lane_file, 'points_crop', 'points_resize');
    assert(num == numel(points_crop) && num == numel(points_resize))
    for i=1:num
        lanes_uv1 = NNs2LaneUV(points_crop{i}, det_src.overall);
        lanes_uv2 = NNs2LaneUV(points_resize{i}, det_src.partial);
        lanes_uv = repmat(lane_const.UV, 1, lane_const.max_lns);
        cnt = 0;
        for j=1:lane_const.max_lns
            if lanes_uv1(j).len > 0
                if cnt >= lane_const.max_lns, break; end
                cnt = cnt+1;
                lanes_uv(cnt) = lanes_uv1(j);
            end
            if lanes_uv2(j).len > 0
                if cnt >= lane_const.max_lns, break; end
                cnt = cnt+1;
                lanes_uv(cnt) = lanes_uv2(j);
            end
        end
        
        L_Num = 0;
        for j=1:lane_const.max_lns
            if lanes_uv(j).len > 0, L_Num = L_Num+1; end
        end
        if L_Num > 0, lines = cell(1, L_Num); else, lines = []; end
        
        cnt = 0;
        for j=1:lane_const.max_lns
            if lanes_uv(j).len > 0
                cnt = cnt+1;
                P_Num = lanes_uv(j).len;
                pts = [lanes_uv(j).u(1:lanes_uv(j).len), lanes_uv(j).v(1:lanes_uv(j).len)];
                src = repmat(lanes_uv(j).src,lanes_uv(j).len,1);
                rho = zeros(lanes_uv(j).len,1);
                feature = lanes_uv(j).feature(1:lanes_uv(j).len,:);
                cls = lanes_uv(j).cls(1:lanes_uv(j).len,1);
                lines{cnt} = struct('P_Num', P_Num, 'pts', pts, 'src', src, 'rho', rho, 'feature', feature, 'cls', cls);
            end
        end
        lanes{i} = struct('L_Num', L_Num, 'lines', {lines});
    end
end

ParamDetObj = struct('name','', 'height', 0, 'width', 0, 'conf_thres', 0);
objects = cell(1, num);
for i=1:num
    objects{i} = struct('conf', [], 'cls', [], 'bbox', [], 'r', [], 'cbox', []);
end
if ~isempty(obstacle_file)
    load(obstacle_file, 'detections');
    assert(num == numel(detections))
    for i=1:num
        tmp = detections{i};
        Obj_N = size(tmp, 1);
        conf = zeros(Obj_N, 1);
        cls = zeros(Obj_N, 1);
        bbox = zeros(Obj_N, 4);
        r = zeros(Obj_N, 1);
        cbox = zeros(Obj_N, 2);
        for j=1:Obj_N
            conf(j) = tmp(j, 1);
            cls(j) = tmp(j, 2);
            bbox(j,:) = [tmp(j,3), tmp(j,5), tmp(j,4), tmp(j,6)];
            if isnan(tmp(j,7))
                r(j) = 0;
            else
                if tmp(j,10) == 1, K = 1; else, K = -1; end
                r(j) = K*tmp(j,9);
            end
        end
        objects{i} = struct('conf', conf, 'cls', cls, 'bbox', bbox, 'r', r, 'cbox', cbox);
    end
end

save(fullfile(path, [name, '.vdet']), 'info', 'ParamVideo', 'lanes', 'objects', 'ParamDetObj', 'ParamDetLane')

end

function lanes_uv = NNs2LaneUV(points, src)
    if nargin < 2, src = det_src.overall; end
    lanes_uv = repmat(lane_const.UV, 1, lane_const.max_lns);
    maxid = max(points(:,4));
    for i=1:min([(maxid+1), lane_const.max_lns])
        k = find(points(:,4)==(i-1));
        tmp = points(k,:);
        len = length(k);
        lanes_uv(i).len = len;
        lanes_uv(i).u(1:len) = tmp(:,2);
        lanes_uv(i).v(1:len) = tmp(:,3);
        lanes_uv(i).src = src;
        lanes_uv(i).theta(1:len) = tmp(:,6);
        lanes_uv(i).feature(1:len,:) = tmp(:, 7:9);
        lanes_uv(i).cls(1:len,:) = uint8(tmp(:,5));
    end
end
