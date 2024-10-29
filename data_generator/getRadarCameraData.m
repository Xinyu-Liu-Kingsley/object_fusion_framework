
CameraMatrix = load('testData\CameraMatrix.mat');
RadarMatrix = load('testData\RadMatrix.mat');
CameraData = CameraMatrix.cameraMatrix;
RadarData = RadarMatrix.radArray;
m1 = numel(CameraData);
m2 = numel(RadarData);
if m1>= m2
    m = m2;
else
    m = m1;
end
frmData = repmat(obstacle_const.FrameData,m,1);
for i = 1:m
    n1 = numel(CameraData(i).measures);
    % 存摄像头数据
    CameraDataSinlge  = frmData(i).CameraFrame.CameraObjectList;    % 定义的结构体形式
    CameraDataSinlge_process  = CameraData(i).measures; % 采集数据
    for j = 1:n1
        if CameraDataSinlge_process(j).ID>0
            CameraDataSinlge(j).ID = uint8(CameraDataSinlge_process(j).ID);
            CameraDataSinlge(j).x = double(CameraDataSinlge_process(j).x);
            CameraDataSinlge(j).y = double(CameraDataSinlge_process(j).y);
            CameraDataSinlge(j).velo_x = double(CameraDataSinlge_process(j).rel_vx);
            CameraDataSinlge(j).velo_y = double(CameraDataSinlge_process(j).rel_vy);
            CameraDataSinlge(j).cls = obstacle_cls.car;
            CameraDataSinlge(j).obj_width = double(CameraDataSinlge_process(j).width);
            CameraDataSinlge(j).flag = 1;
        end
    end
    frmData(i).CameraFrame.CameraObjectList = CameraDataSinlge;
    %存雷达数据
    n2 = numel(RadarData(i).frame);
    RadarDataSinlge  = frmData(i).RadarFrame.RadarObjectList;    % 定义的结构体形式
    RadarDataSinlge_process  = RadarData(i).frame; % 采集数据
    cnt = 0;
    for j1 = 1:n2
        % 数据清洗
        if RadarDataSinlge_process(j1).ID>0 && RadarDataSinlge_process(j1).x<180 &&...
                RadarDataSinlge_process(j1).y>-5 && RadarDataSinlge_process(j1).y<5
            cnt = cnt+1;
            RadarDataSinlge(cnt).ID = uint8(RadarDataSinlge_process(j1).ID);
            RadarDataSinlge(cnt).x = double(RadarDataSinlge_process(j1).x);
            RadarDataSinlge(cnt).y = double(RadarDataSinlge_process(j1).y);
            RadarDataSinlge(cnt).velo_x = double(RadarDataSinlge_process(j1).vx);
            RadarDataSinlge(cnt).velo_y = double(RadarDataSinlge_process(j1).vy);
            RadarDataSinlge(cnt).ax = double(RadarDataSinlge_process(j1).ax);
            RadarDataSinlge(cnt).ay = double(RadarDataSinlge_process(j1).ay);
            RadarDataSinlge(cnt).cls = obstacle_cls.car;
            RadarDataSinlge(cnt).SNR = double(RadarDataSinlge_process(j1).RCS);
            RadarDataSinlge(cnt).flag = 1;
        end
    end
    frmData(i).RadarFrame.RadarObjectList = RadarDataSinlge;
end

