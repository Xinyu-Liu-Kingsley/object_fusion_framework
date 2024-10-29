function showImage(ax, frame, lanes, obstacles,peds,cropSize,laneCrossSeg)
% 显示照片及车道线、障碍物

if nargin < 4
    obstacles = [];
end
if nargin < 3
    lanes = [];
end

cla(ax)
hold(ax, 'on')
image(ax, frame, 'AlphaData', 0.8)

if ~isempty(lanes)
    if isfield(lanes, 'u')        % lane_const.UV格式数据
        for i=1:numel(lanes)
            len = lanes(i).len;
            if len > 0
                scatter(ax, lanes(i).u(1:len), lanes(i).v(1:len), 12, 'filled')
            end
        end
    elseif isfield(lanes, 'src')  % lane_const.NN格式数据
        scatter(ax, lanes.u(1:lanes.len), lanes.v(1:lanes.len), 'filled')
    end
end

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
    [1 0 1];...
    
    [1 0 0]; ...       % pedestrian 红
    [0.8 0 0] ...      % rider 深红色
    ];
% 画出裁剪框，一阶段为实线 一阶段为虚线
% cropSize
 rectangle(ax, 'Position', [cropSize(1), cropSize(2),cropSize(3) ,cropSize(4)], 'EdgeColor', 'k')
 
 laneCrossSeg_u = [1,1920];
 laneCrossSeg_v1 = [laneCrossSeg(1),laneCrossSeg(1)];
 laneCrossSeg_v2 = [laneCrossSeg(2),laneCrossSeg(2)];
 plot(laneCrossSeg_u,laneCrossSeg_v1,'Color','r','LineWidth',2)
 plot(laneCrossSeg_u,laneCrossSeg_v2,'Color','b','LineWidth',2)

 if ~isempty(obstacles)
     if isfield(obstacles, 'flag')
         for i=1:obstacle_const.max_obs
             if obstacles(i).flag
                 rectangle(ax, 'Position', [obstacles(i).u, obstacles(i).v, obstacles(i).w, obstacles(i).h], 'EdgeColor', c_cls(obstacles(i).cls+1,:))

                 %strCls= strcat('cls:', num2str([obstacles(i).cls,obstacles(i).cls2]));
                 if obstacles(i).cls ~= obstacles(i).cls2
                     text(obstacles(i).u+1, obstacles(i).v+obstacles(i).h*0.1 , num2str([obstacles(i).cls,obstacles(i).cls2]),'color','r','FontSize',7);
                 end
                 if  any([obstacles(i).u1,obstacles(i).w1])
                     rectangle(ax, 'Position', [obstacles(i).u1, obstacles(i).v1, obstacles(i).w1, obstacles(i).h1], 'EdgeColor', c_cls(obstacles(i).cls+1,:),'LineStyle','--') 
                 end

                 if abs(obstacles(i).r) > 0.1
                     xx = [obstacles(i).r+obstacles(i).u,obstacles(i).r+obstacles(i).u];
                     yy =  [obstacles(i).v, obstacles(i).v+obstacles(i).h];
                     if obstacles(i).r > 0
                         cl = [0, 1, 1];
                     else
                         cl = [1, 0, 1];
                     end
                     line(ax, xx, yy, 'LineStyle', '--', 'Color', cl, 'LineWidth', 1)
                 end
                 if any([obstacles(i).wheelLine_u1,obstacles(i).wheelLine_v1,obstacles(i).wheelLine_u2,obstacles(i).wheelLine_v2])
                     xx = [obstacles(i).u1+obstacles(i).wheelLine_u1,obstacles(i).u1+obstacles(i).wheelLine_u2];
                     yy = [obstacles(i).v1+obstacles(i).wheelLine_v1,obstacles(i).v1+obstacles(i).wheelLine_v2];
                     line(ax, xx, yy, 'LineStyle', '--', 'Color', [0, 1, 1], 'LineWidth', 1.5);
                 end

             end
         end
     elseif isfield(obstacles, 'state')
        for i=1:obstacle_const.max_obs
            if obstacles(i).state ~= track_status.untracking
                rectangle(ax, 'Position', [obstacles(i).u, obstacles(i).v, obstacles(i).w, obstacles(i).h], 'EdgeColor', [1, 0, 0])
%                 plot(obstacles(i).left_line(1),obstacles(i).left_line(2),'hc', 'MarkerSize', 5); %zyz
%                 plot(obstacles(i).right_line(1),obstacles(i).right_line(2),'hc', 'MarkerSize', 5); %zyz
%                 line(obstacles(i).right_line,obstacles(i).left_line,'LineStyle', '-', 'LineWidth', 2)
            end
        end
    end
end
if ~isempty(peds)
    if isfield(peds, 'flag')
        for i=1:peds_const.max_obs
            if peds(i).flag 
                rectangle(ax, 'Position', [peds(i).u, peds(i).v, peds(i).w, peds(i).h], 'EdgeColor', [1 0 0])
%                 plot(obstacles(i).left_line(1),obstacles(i).left_line(2),'hc', 'MarkerSize', 5); %zyz
%                 plot(obstacles(i).right_line(1),obstacles(i).right_line(2),'hc', 'MarkerSize', 5);%zyz
%                 axis([0 1 0 1])
%                 x = [obstacles(i).left_line(1),obstacles(i).right_line(1)];
%                 y = [obstacles(i).left_line(2),obstacles(i).right_line(2)];
%                 line(x,y,'LineStyle', '-', 'LineWidth', 2);
%                 if abs(peds(i).r) > 0.1
%                     xx = repmat(peds(i).u+(sign(peds(i).r)<0)*peds(i).w+peds(i).r*peds(i).w, 1, 2);
%                     yy = [peds(i).v, peds(i).v+peds(i).h];
%                     if peds(i).r > 0
%                         cl = [0, 1, 1];
%                     else
%                         cl = [1, 0, 1];
%                     end
%                     line(ax, xx, yy, 'LineStyle', '--', 'Color', cl, 'LineWidth', 1)
%                 end
            end
        end
    elseif isfield(peds, 'state')
        for i=1:peds_const.max_obs
            if peds(i).state ~= track_status.untracking
                rectangle(ax, 'Position', [peds(i).u, peds(i).v, peds(i).w, peds(i).h], 'EdgeColor', [1, 0, 0])
%                 plot(obstacles(i).left_line(1),obstacles(i).left_line(2),'hc', 'MarkerSize', 5); %zyz
%                 plot(obstacles(i).right_line(1),obstacles(i).right_line(2),'hc', 'MarkerSize', 5); %zyz
%                 line(obstacles(i).right_line,obstacles(i).left_line,'LineStyle', '-', 'LineWidth', 2)
            end
        end
    end
end

global g_cam_status;
if g_cam_status.uv(1) ~= 0 && g_cam_status.uv(2) ~= 0
    plot(g_cam_status.uv(1), g_cam_status.uv(2), '+r', 'MarkerSize', 20)
end


% xlim([0 g_cam_params.img_size(2)])
% ylim([0 g_cam_params.img_size(1)])

set(ax, 'ydir', 'reverse')
axis(ax, 'off')
axis(ax, 'equal')

end

