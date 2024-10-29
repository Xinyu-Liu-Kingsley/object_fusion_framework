function plotResultsFusion(ax,frameData,eobstacles, stitle)
%PLOTRESULTS 此处显示有关此函数的摘要

cla(ax)
hold(ax, 'on')
wdLines = [2, 2, 2, 1, 1];
% 由颜色表征车道线左右属性；

%% 画车道线
% %  红色（左线） 天蓝色（右线）玫红色（左左） 蓝色（右右）
crLines = { [1 0 0], [0 1 1], [1 0 1], [0 0 1]}; 
% % 跟踪状态：初始过程，建立过程，稳定跟踪，不稳定
% stLines = {'-', '--', '-.', ':', '--'}; 
lane_list = frameData.CameraFrame.LaneList;
for i = 1:obstacle_const.max_lane_num
    if lane_list(i).flag>0
         x = linspace(0, lane_list(i).view_range, 50);
%          x = linspace(0, 120, 50);
         C = [lane_list(i).C3,lane_list(i).C2,lane_list(i).C1,lane_list(i).C0];
%          C = [lane_list(i).C2,lane_list(i).C1,lane_list(i).C0];
         y = polyval(C, x);
         plot(ax, y, x, 'color', crLines{lane_list(i).lane_locaiotn+1},'linewidth',2)
    end
end


global g_cfg_auto;
global g_tracks_auto;
% global g_cfg_VRU;

% 单帧检测到的障碍物
x_camera = zeros(1, obstacle_const.max_obs_camera);
y_camera = zeros(1, obstacle_const.max_obs_camera);
x_radar = zeros(1, obstacle_const.max_obs_radar);
y_radar = zeros(1, obstacle_const.max_obs_radar);
sid_radar = cell(1,obstacle_const.max_obs_radar);
cameraObjects = frameData.CameraFrame.CameraObjectList;
radarObjects = frameData.RadarFrame.RadarObjectList;

% 被跟踪的目标
ex = zeros(1, obstacle_const.track_width);
ey = zeros(1, obstacle_const.track_width);
ec = zeros(1, obstacle_const.track_width, 'uint8');
sid = cell(1, obstacle_const.track_width);

% 跟踪库中相对速度
vx = zeros(2, obstacle_const.track_width);
vy = zeros(2, obstacle_const.track_width);

% 计数器
cntCam = 0;    % 单帧检测到的Camera数量
cntRadar = 0;  % 单帧检测到的Radar数量
ecnt = 0;   % 跟踪库中的有效目标
vcnt = 0;   % 跟踪库中的稳定跟踪目标
for j=1:obstacle_const.max_obs_camera
    if cameraObjects(j).flag
        cntCam = cntCam+1;
        x_camera(cntCam) = cameraObjects(j).x;
        y_camera(cntCam) = cameraObjects(j).y;
%         if cameraObjects(j).CIPVFlag == uint8(1)
%             plot(cameraObjects(j).x,cameraObjects(j).y,"x",'MarkerSize',15,'LineStyle',"-.",'MarkerEdgeColor',[1 0 0.2]);
%         end
    end
end

% cnt = 0;
for j=1:obstacle_const.max_obs_radar
    if radarObjects(j).flag
        cntRadar = cntRadar+1;
        x_radar(cntRadar) = radarObjects(j).x;
        y_radar(cntRadar) = radarObjects(j).y;
        sid_radar{cntRadar} = sprintf('%s%d', char(160),radarObjects(j).ID);
    end
end


df = 1;
for i= 1:obstacle_const.cfg.EOut_vehs_num
    if eobstacles(i).flag>0
        ID = eobstacles(i).ID;
        ecnt = ecnt + 1;
%         sid{ecnt} = sprintf('%d(%03d, -%1d)', ...
%             ID, g_tracks_auto.status(ID).CNT, g_tracks_auto.status(ID).LST);
        if eobstacles(i).CIPV == uint8(1)
             sid{ecnt} = sprintf(' \n \n \n%d(%s)', ...
            ID, 'CIPV');
        else
            sid{ecnt} = sprintf(' \n \n \n%d(%1d, %1d)', ...
                ID, g_tracks_auto.output(ID).fusedCamID, g_tracks_auto.output(ID).fusedRadID);
        end
        ex(ecnt) = g_tracks_auto.output(ID).x;
        ey(ecnt) = g_tracks_auto.output(ID).y;
        ec(ecnt) = g_tracks_auto.output(ID).state;
        if g_tracks_auto.status(ID).state >= track_status.stable
            vcnt = vcnt + 1;
            vx(:, vcnt) = [ex(ecnt), ex(ecnt) + g_tracks_auto.output(ID).velo_x * df];
            vy(:, vcnt) = [ey(ecnt), ey(ecnt) + g_tracks_auto.output(ID).velo_y * df];
        end
    end
end
% 输出 Vehicle


c_sta = [ ...
    [0.7 0.7 0.7]; ...  % initial   [gray]
    [0.8 0.8 0]; ...    % growing   []
    [0 1 0]; ...        % stable    [green]
    [1 0 0] ...         % unstable  [red]
    ];
c_cls = [...
    [0.7 0.7 0.7]; ... % unknown 灰色
    [0 1 0]; ...       % minicar 浅绿色
    [0 0.5 0]; ...     % car 深绿色
    [1 0.9 0]; ...     % minibus 黄色
    [1 0.7 0];         % midibus 橘黄
    [1 0.4 0]; ...     % bus 橘红
    [0,0.8,1];...      % truckhead; 青蓝色
    [0 0 1];...        % minitruck；蓝色
    [0 0 0.8];...      % miditruck；深蓝色
    [0 0 0.4]; ...     % truck；深深蓝色
    [0 0 0.2];...      % specialvehicle 深蓝近黑色
    [1 0 1];...        % tricycle:magenta
    
    [1 0 0]; ...       % pedestrian 红
    [0.8 0 0] ...      % rider 深红色
    ];

% x(cnt+1:end) = [];
% y(cnt+1:end) = [];
% cl(cnt+1:end) = [];
ex(ecnt+1:end) = [];
ey(ecnt+1:end) = [];
ec(ecnt+1:end) = [];
sid(ecnt+1:end) = [];
x_radar(cntRadar+1:end) = [];
y_radar(cntRadar+1:end) = [];
sid_radar(cntRadar+1:end) = [];
x_camera(cntCam+1:end) = [];
y_camera(cntCam+1:end) = [];
vx(:, vcnt+1:end) = [];
vy(:, vcnt+1:end) = [];

% 跟踪库中的有效目标
if ecnt > 0 
    scatter(ax, ex, ey, 120, c_sta(ec,:), 'pentagram', 'linewidth', 2,'MarkerFaceColor',[1 0.4 0])
    text(ax, ex, ey, sid, 'interpreter', 'latex','Color','m','FontSize',8)
end

% 单帧摄像头数据
if cntCam >0
    scatter(ax,x_camera,y_camera,'MarkerEdgeColor',[0 0 0.2]);
end

% 单帧雷达数据
if cntRadar>0
    scatter(ax,x_radar,y_radar,'square','MarkerEdgeColor',[0.8 0 0])
    text(ax,x_radar,y_radar,sid_radar,'interpreter', 'none','Color', 'b','FontSize',6)

end
if vcnt > 0    % 跟踪库中的稳定跟踪目标
    line(vx, vy, 'color', 'r', 'linewidth', 1)
end 


% set(ax, 'xdir', 'reverse', 'TickLabelInterpreter', 'latex'); 
%set(gca,'YTickLabel',{'-4','0','10','20','40','60','80','100','120','140','160','180'});
title(ax, stitle, 'Interpreter', 'latex')
xlim(ax, g_cfg_auto.x_coverage)
ylim(ax, [-50,100])
box(ax, 'on')
grid(ax, 'on')


end



