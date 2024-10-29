function plotResults(ax, lanes, lanes_xy, obs_XYs,eobstacles,peds, radTar, stitle)
%PLOTRESULTS 此处显示有关此函数的摘要

leg = cell(lane_const.max_lns*2,1);
cnt = 0;
cnt1 = 0;
cnt2 = 0;

cla(ax)
hold(ax, 'on')

% 画自车模型
global g_ego_params; 
rectangle(ax, 'Position', [-g_ego_params.width/2, -g_ego_params.length, g_ego_params.width, g_ego_params.length], 'EdgeColor', 'r') ;
plot(ax,[g_ego_params.width/2,-g_ego_params.width/2],[-g_ego_params.front_overhang,-g_ego_params.front_overhang],'-','Color','b');


if ~isempty(lanes_xy)
    if isfield(lanes_xy, 'x') && isfield(lanes_xy, 'y')
        for j=1:lane_const.max_lns
            %len = lanes_xy(j).len;
            idx = (lanes_xy(j).conf==-1);
            len = lanes_xy(j).len-sum(idx);
            if len > 0
                scatter(ax, lanes_xy(j).y(1:len), lanes_xy(j).x(1:len))%, 'filled'
                cnt = cnt+1;
                cnt1 = cnt1+1;
                leg{cnt} = sprintf('Cluster.%d', cnt1);
                scatter(ax, lanes_xy(j).y(idx), lanes_xy(j).x(idx),3,'o','MarkerEdgeColor',[0 .5 .5]);
            end
        end
    end
end

wdLines = [2, 2, 2, 1, 1];
% 由颜色表征车道线左右属性；
% 灰色（ohter） 红色（左线） 天蓝色（右线）玫红色（左左） 蓝色（右右）
crLines = {[0.5 0.5 0.5], [1 0 0], [0 1 1], [1 0 1], [0 0 1]}; 
% 跟踪状态：初始过程，建立过程，稳定跟踪，不稳定
stLines = {'-', '--', '-.', ':', '--'}; 
if ~isempty(lanes)
    if isfield(lanes, 'C') && isfield(lanes, 'range') && isfield(lanes, 'flag')
        for j=1:lane_const.max_lns
            if lanes(j).flag
                y = linspace(-g_ego_params.front_overhang, lanes(j).range(2), lane_const.max_pts);
                x = polyval(lanes(j).C(4:-1:1), y);
                plot(ax, x, y, 'linestyle', stLines{lanes(j).index+1}, 'linewidth', wdLines(lanes(j).index+1), 'color', crLines{lanes(j).index+1})
%                 scatter(ax, x, y, [], crLines{lanes(j).index+1}, 'filled')
                cnt = cnt+1;
                cnt2 = cnt2+1;
                leg{cnt} = sprintf('Line.%d', cnt2);
            end
        end
    end
    if isfield(lanes, 'C') && isfield(lanes, 'range') && isfield(lanes, 'state')
        for j=1:lane_const.max_lns
            if lanes(j).state > track_status.initial
                y = linspace(-g_ego_params.front_overhang, lanes(j).range(2), lane_const.max_pts);
                x = polyval(lanes(j).C(4:-1:1), y);
                plot(ax,x, y, 'linestyle', stLines{lanes(j).state}, 'linewidth', wdLines(lanes(j).index+1), 'color', crLines{lanes(j).index+1})
                cnt = cnt+1;
                cnt2 = cnt2+1;
                leg{cnt} = sprintf('Line.%d', cnt2);
            elseif lanes(j).state == track_status.initial
                y = linspace(-g_ego_params.front_overhang, lanes(j).range(2), lane_const.max_pts);
                x = polyval(lanes(j).C(4:-1:1), y);
                plot(ax,x, y, 'linestyle', stLines{lanes(j).state}, 'linewidth', wdLines(end), 'color', crLines{lanes(j).index+1})
                cnt = cnt+1;
                cnt2 = cnt2+1;
                leg{cnt} = sprintf('Line.%d', cnt2);

            end
        end
    end
end

% % plot g_tracks_lane.l_line.C & g_tracks_lane.r_line.C ;
% global g_tracks_lane
% if g_tracks_lane.l_line.C(1)~=0
%     y = linspace(0, max(g_tracks_lane.l_line.range(2),100), lane_const.max_pts);
%     x = polyval(g_tracks_lane.l_line.C(4:-1:1), y);
%     plot(x, y, 'linestyle','--', 'linewidth', 0.5, 'color','red' )
% end
% if g_tracks_lane.r_line.C(1)~=0
%     y = linspace(0, max(g_tracks_lane.r_line.range(2),100), lane_const.max_pts);
%     x = polyval(g_tracks_lane.r_line.C(4:-1:1), y);
%     plot(x, y, 'linestyle','--' , 'linewidth', 0.5, 'color','blue' )
% end


global g_cfg_auto;
global g_tracks_auto;
% global g_cfg_VRU;
global g_tracks_VRU g_cfg_lane;

% 单帧检测到的障碍物
x = zeros(1, obstacle_const.max_obs);
y = zeros(1, obstacle_const.max_obs);
cl = zeros(1, obstacle_const.max_obs, 'uint8');

% 被跟踪的目标
ex = zeros(1, obstacle_const.track_width);
ey = zeros(1, obstacle_const.track_width);
ec = zeros(1, obstacle_const.track_width, 'uint8');
sid = cell(1, obstacle_const.track_width);

% % kalman滤波结果
% kx = zeros(1, obstacle_const.track_width);
% ky = zeros(1, obstacle_const.track_width);
% 
vx = zeros(2, obstacle_const.track_width);
vy = zeros(2, obstacle_const.track_width);


cnt = 0;    % 单帧检测到的目标数量
ecnt = 0;   % 跟踪库中的有效目标
vcnt = 0;   % 跟踪库中的稳定跟踪目标
for j=1:obstacle_const.max_obs
    if isfield(obs_XYs, 'flag')
        if obs_XYs(j).flag
            cnt = cnt + 1;
            x(cnt) = obs_XYs(j).x;
            y(cnt) = obs_XYs(j).y;
            cl(cnt) = obs_XYs(j).cls+uint8(1);
        end
    end
end
% 行人
for j=1:peds_const.max_obs
    if isfield(peds, 'flag')
        if peds(j).flag
            cnt = cnt + 1;
            x(cnt) = peds(j).x;
            y(cnt) = peds(j).y;
            cl(cnt) = peds(j).cls+uint8(1);
        end
    end
end

df = 1;
for j=1:peds_const.track_width
    if ecnt < peds_const.track_width && g_tracks_auto.output(j).state >= track_status.initial
        ecnt = ecnt + 1;

        sid{ecnt} = sprintf('%d(%03d, -%1d, %1d)', ...
            j, g_tracks_auto.status(j).CNT, g_tracks_auto.status(j).LST, g_tracks_auto.output(j).laneIndex);
        ex(ecnt) = g_tracks_auto.output(j).x;
        ey(ecnt) = g_tracks_auto.output(j).y;
        ec(ecnt) = g_tracks_auto.output(j).state;
        if g_tracks_auto.status(j).state >= track_status.stable
            vcnt = vcnt + 1;
            vx(:, vcnt) = [ex(ecnt), ex(ecnt) + g_tracks_auto.output(j).rel_vx * df];
            vy(:, vcnt) = [ey(ecnt), ey(ecnt) + g_tracks_auto.output(j).rel_vy * df];
        end
    end
    if ecnt < peds_const.track_width && g_tracks_VRU.output(j).state >= track_status.initial
        ecnt = ecnt + 1;

        sid{ecnt} = sprintf('%d(%03d, -%1d)', ...
            j, g_tracks_VRU.status(j).CNT, g_tracks_VRU.status(j).LST);  %  生命计数器  丢帧计数器
        ex(ecnt) = g_tracks_VRU.output(j).x;
        ey(ecnt) = g_tracks_VRU.output(j).y;
        ec(ecnt) = g_tracks_VRU.output(j).state;
        if g_tracks_VRU.status(j).state >= track_status.stable
            vcnt = vcnt + 1;
            vx(:, vcnt) = [ex(ecnt), ex(ecnt) + g_tracks_VRU.output(j).rel_vx * df];
            vy(:, vcnt) = [ey(ecnt), ey(ecnt) + g_tracks_VRU.output(j).rel_vy * df];
        end
    end
end
% 输出 Vehicle
for ii = 1:obstacle_const.cfg.EOut_vehs_num
    if eobstacles(ii).ID>0
        plot(eobstacles(ii).y,eobstacles(ii).x,Marker="square",MarkerSize=15,LineStyle="-.",MarkerEdgeColor=[0.8 0.8 0]);
    end
end

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

x(cnt+1:end) = [];
y(cnt+1:end) = [];
cl(cnt+1:end) = [];
ex(ecnt+1:end) = [];
ey(ecnt+1:end) = [];
ec(ecnt+1:end) = [];
sid(ecnt+1:end) = [];
vx(:, vcnt+1:end) = [];
vy(:, vcnt+1:end) = [];


if cnt > 0   % 单帧检测到的目标数量  
    scatter(ax, y, x, [], c_cls(cl,:), 'filled')
end
if ecnt > 0 % 跟踪库中的有效目标
    scatter(ax, ey, ex, 120, c_sta(ec,:), 's', 'linewidth', 2)
    text(ax, ey, ex, sid, 'interpreter', 'latex')
end
if vcnt > 0    % 跟踪库中的稳定跟踪目标
    line(vy, vx, 'color', 'r', 'linewidth', 1)
end

if ~isempty(radTar)
    N = numel(radTar);
    x = zeros(1,N);
    y = zeros(1,N);
    vx = zeros(1,N);
    vy = zeros(1,N);
    for i=1:N
        x(i) = radTar(i).x;
        y(i) = radTar(i).y;
        vx(i) = radTar(i).vx;
        vy(i) = radTar(i).vy;
    end
    scatter(ax, y, x, 'p')
end

set(ax, 'xdir', 'reverse', 'TickLabelInterpreter', 'latex'); 
%set(gca,'YTickLabel',{'-4','0','10','20','40','60','80','100','120','140','160','180'});
title(ax, stitle, 'Interpreter', 'latex')
xlim(ax, g_cfg_auto.y_coverage)
ylim(ax, g_cfg_auto.x_coverage+ [g_cfg_lane.min_stitcher_x*2,0])
box(ax, 'on')
grid(ax, 'on')


end

