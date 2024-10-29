% Description: find main targets & dynamic display 
% Author : hanhua                     
% log:
% 20220913: plotResults跟踪库中的左右道线画图,滤波后参数保存--hanhua; 
%****************************************************************% 

%% ====================== 存储信息 ====================== %%
% radar 检测结果
% if ~isempty(radTable)&& frmIDs(i)<=size(radTable,1)  %&&frmIDs(i)>offsetFrm
%     if numel(radTable.frame{frmIDs(i)})== numel(AIF.frmInfo)
%         radTar = [];
%     else
%         radTar = radArray(frmIDs(i)).frame;
%     end
%     radTime = radArray(frmIDs(i)).tm;
% else
%     radTar = [];
% end
radTar = [];
% 存储cipv轨迹信息并显示（输出cipv, 原始检测obs,radar左中右车道目标）
[cipvTrack,lTrack,rTrack] = saveCIPV(cipvTrack,g_tracks_auto.output,frmIDs(i),lTrack,rTrack);
[obsCIPV,obsL,obsR] = save_frame_VehCIPV(obsCIPV,obsL,obsR,frmIDs(i));

[cippTrack,obsCIPP] = saveCIPP(cippTrack,frmIDs(i),obsCIPP);

if radarFlag && frmIDs(i)<length(T)        
    [cRadar,lRadar,rRadar,radarTracksMat] = selectRadarTracks(cRadar,lRadar,rRadar,radarTracksMat,frmIDs(i), ...
        cippTrack(frmIDs(i),:),lTrack(frmIDs(i),:),rTrack(frmIDs(i),:),T);
end
% 瞬态校正值
pitches(i) = current_pitch();

%保存C0-C3的值
[CLeftMatrix,CRightMatrix,laneParams]=saveLaneC(CLeftMatrix,CRightMatrix,lanes_me,frmIDs(i),laneParams,lanes_eo);

%% ====================== 画图 ====================== %%
if  displayFlag
    % 显示画面
    ax = subplot(2,2,1);
    title(['frmIdx = ',num2str(frmIDs(i))]);
    
    showImage(ax, frm, lanes_uv, obstacles_uv,pedstrains_uv,cropSizeObs(frmIDs(i), :),laneCrossSeg(frmIDs(i), :))
    % showPixelTracker(ax,frm);
    dispLanes(ax,frm,lanes_nn,lanes_xy,lanes_uv_2nd,frmIDs(i));

    ax = subplot(2,2,[2 4]);
    plotResults(ax, lanes_eo.line, lanes_xy, obstacles, eobstacles,pedstrains,radTar, sprintf('No.%d pitch - %.2f', frmIDs(i), pitches(i)))

    % 显示cipv
    hold on;
    if  lTrack(frmIDs(i),1)~=0
        plot(lTrack(frmIDs(i),3),lTrack(frmIDs(i),2),'m<');
    end
    if  rTrack(frmIDs(i),1)~=0
        plot(rTrack(frmIDs(i),3),rTrack(frmIDs(i),2),'m>');
    end
    if  cipvTrack(frmIDs(i),1)~=0
        plot(cipvTrack(frmIDs(i),3),cipvTrack(frmIDs(i),2),'r^');
    end
    if  any(lRadar.data(frmIDs(i),2:3))  % [time id x,y,vx,vy,RCS]
        plot(lRadar.data(frmIDs(i),4),lRadar.data(frmIDs(i),3),'ms');
    end
    if  any(rRadar.data(frmIDs(i),2:3))  % [time id x,y,vx,vy,RCS]
        plot(rRadar.data(frmIDs(i),4),rRadar.data(frmIDs(i),3),'ms');
    end
    if  any(cRadar.data(frmIDs(i),2:3))  % [time id x,y,vx,vy,RCS]
        plot(cRadar.data(frmIDs(i),4),cRadar.data(frmIDs(i),3),'rs');
    end
    % 显示瞬态校正值
    ax = subplot(2,2,3);
    plot(ax, pitches,'.-.')
    ylim([-3 3]);
    set(gca,'YTick',(-3:1:3));
    ylabel('dynamic pitch angle $\deg$', 'Interpreter', 'latex')
    grid on
    box on
end
function  dispLanes(ax,frm,lanes_nn,lanes_xy,lanes_uv_2nd,frmID)
global g_cam_params;
crLines = {[0.4660 0.6740 0.1880],[0.5 0.5 0.5], [1 0 0], [0 1 1], [1 0 0], [0.8 0.8 0.8],[0.9290 0.6940 0.1250],[1 0 1]};
hold(ax, 'on')
[K_Rw2c,~] = lookup_R();
mode = 1; % 画图方式:0 lanes_xy 点，1：基于方程投影lanes_uv_2nd;2 原始检测点

if mode==0
    for i=1:lane_const.max_lns
        if lanes_xy(i).len >0
            len = lanes_xy(i).len;
            x = lanes_xy(i).x(1:len);
            y = lanes_xy(i).y(1:len);
            % uv = xy2uv([y' -x'], K_Rw2c, g_cam_params.T, current_pitch());
            uv = xy2uv([x -y], K_Rw2c, g_cam_params.T);
            plot(uv(:,1),uv(:,2),'o','markersize',6,'lineWidth',1.5,'color',crLines{i});
        end
    end    
end
if  mode == 1
    for i=1:lane_const.max_lns
        if lanes_uv_2nd(i).len >0
           len = lanes_uv_2nd(i).len;
           plot(lanes_uv_2nd(i).u(1:len),lanes_uv_2nd(i).v(1:len),'o','markersize',3,'lineWidth',0.8,'color',[0.4940 0.1840 0.5560]); 
        end       
    end
end
if mode == 2
    idx = 1:lanes_nn.len;
    %scatter(ax,lanes_nn.u(idx),lanes_nn.v(idx),'o');
    plot(lanes_nn.u(idx),lanes_nn.v(idx),'o','markersize',2);
end

% % 真实环境下 4 6 8 10m点投影到图像上的位置
% pointD = [10,0; 10,1.8; 8,0; 8,1.8; 6,0;6,1.8; 4,0; 4,1.8];
% uv_point = xy2uv(pointD, K_Rw2c, g_cam_params.T);
% plot(uv_point(:,1),uv_point(:,2),'rs','markersize',10,'lineWidth',1.5);

title(['frmIdx = ',num2str(frmID)]);
end
%% 存储CIPV
function [cipvTrack,lTrack,rTrack] = saveCIPV(cipvTrack,eobstacles,frmidx,lTrack,rTrack)
global g_tracks_auto
m = numel(eobstacles);
idx = 255;
for  i = 1:m 
    if eobstacles(i).x == 0
        continue;        
    end
    if eobstacles(i).CIPV == 1
        idx = i; 
        break;
    end
end
if idx~=255
    cipvTrack(frmidx,1) = idx ; % [id x y vx xy state]
    cipvTrack(frmidx,2) = eobstacles(idx).x;
    cipvTrack(frmidx,3) = eobstacles(idx).y;
    cipvTrack(frmidx,4) = eobstacles(idx).rel_vx;
    cipvTrack(frmidx,5) = eobstacles(idx).rel_vy;
    cipvTrack(frmidx,6) = eobstacles(idx).state;
    cipvTrack(frmidx,7) = eobstacles(idx).cls;
    cipvTrack(frmidx,8) = g_tracks_auto.status(idx).cls1;
    cipvTrack(frmidx,9) = eobstacles(idx).TTC;
    cipvTrack(frmidx,10) = g_tracks_auto.status(idx).rel_status(2);
    % lstm
    cipvTrack(frmidx,11) = g_tracks_auto.lstmPredictor(idx).x;
    cipvTrack(frmidx,12) = g_tracks_auto.lstmPredictor(idx).vx;
    %EKF
    cipvTrack(frmidx,13) = g_tracks_auto.ekfOutput(idx).x;
    cipvTrack(frmidx,14) = g_tracks_auto.ekfOutput(idx).vx;

else
    cipvTrack(frmidx,9) = obstacle_const.defTTC; 
end
%=================左右车道最近轨迹========================
global g_tracks_lane g_cfg_lane
minXL = 200; idxL = 255;
minXR = 200; idxR = 255;
for i = 1:m
    if eobstacles(i).x == 0
        continue;
    end
    if  eobstacles(i).x<minXL || eobstacles(i).x<minXR
        % 左右车道线界限
        if g_tracks_lane.l_line.state >= track_status.stable
            yl = polyval(g_tracks_lane.l_line.C(4:-1:1), eobstacles(i).x);
        else
            yl = g_cfg_lane.def_wid_half;
        end
        if g_tracks_lane.r_line.state >= track_status.stable
            yr = polyval(g_tracks_lane.r_line.C(4:-1:1), eobstacles(i).x);
        else
            yr = -g_cfg_lane.def_wid_half;
        end
        % 左侧
        if eobstacles(i).y>yl && eobstacles(i).y<(yl+lane_const.cfg.def_wid_half*2)&&...
                eobstacles(i).x<minXL
            minXL = eobstacles(i).x;
            idxL = i;
        end
        % 右侧
        if eobstacles(i).y<yr && eobstacles(i).y>(yr-lane_const.cfg.def_wid_half*2)&&...
                eobstacles(i).x<minXR
            minXR = eobstacles(i).x;
            idxR = i;
        end
    end
end
if idxL~=255
    lTrack(frmidx,1) = idxL ; % [id x y vx xy state]
    lTrack(frmidx,2) = eobstacles(idxL).x;
    lTrack(frmidx,3) = eobstacles(idxL).y;
    lTrack(frmidx,4) = eobstacles(idxL).rel_vx;
    lTrack(frmidx,5) = eobstacles(idxL).rel_vy;
    lTrack(frmidx,6) = eobstacles(idxL).state;
end
if idxR~=255
    rTrack(frmidx,1) = idxR ; % [id x y vx xy state]
    rTrack(frmidx,2) = eobstacles(idxR).x;
    rTrack(frmidx,3) = eobstacles(idxR).y;
    rTrack(frmidx,4) = eobstacles(idxR).rel_vx;
    rTrack(frmidx,5) = eobstacles(idxR).rel_vy;
    rTrack(frmidx,6) = eobstacles(idxR).state;
end

end

function [CLeftMatrix,CRightMatrix,laneParams] = saveLaneC(CLeftMatrix,CRightMatrix,lanes_me,frmidx,laneParams,lanes_eo)
% save lanes info
for i = 1:lane_const.max_lns
    if lanes_me(i).index== lane_index.l_line
        CLeftMatrix(frmidx,1:4) =  lanes_me(i).C;
        CLeftMatrix(frmidx,9) = lanes_me(i).cls;
        CLeftMatrix(frmidx,10) = lanes_me(i).conf;
    end
    if lanes_me(i).index==lane_index.r_line  
        CRightMatrix(frmidx,1:4) =  lanes_me(i).C;
        CRightMatrix(frmidx,9) =  lanes_me(i).cls;
        CRightMatrix(frmidx,10) =  lanes_me(i).conf;
    end
end

% filter
global g_tracks_lane
CLeftMatrix(frmidx,5:8) = g_tracks_lane.l_line.C;
CRightMatrix(frmidx,5:8) = g_tracks_lane.r_line.C;

% 延长至车轮下的y
global g_ego_params
for j = 1:lane_const.max_lns
    if lanes_eo.line(j).index == lane_index.l_line && lanes_eo.line(j).conf>90
        CLeftMatrix(frmidx,11) = polyval(flip(lanes_eo.line(j).C),-g_ego_params.front_overhang);
    end
    if lanes_eo.line(j).index == lane_index.r_line && lanes_eo.line(j).conf>90
        CRightMatrix(frmidx,11) = polyval(flip(lanes_eo.line(j).C),-g_ego_params.front_overhang); 
    end
end

% 车道信息
laneParams(frmidx,1:4) = [g_tracks_lane.host_width,...
g_tracks_lane.radius,...
g_tracks_lane.yawrate,...
g_tracks_lane.lateral_speed];
end

function [cippTrack,obsCIPP] = saveCIPP(cippTrack,frmidx,obsCIPP)
% 筛选CIPP轨迹,CIPP单帧检测结果；
% obsCIPP:[x,u v,w,h,cls]
% TODO:筛选横穿行人
global g_tracks_VRU

idx = 255;
for  i = 1:peds_const.track_peds 
    if g_tracks_VRU.output(i).x == 0
        continue;        
    end
    if g_tracks_VRU.output(i).CIPP == 1
        idx = i; 
        break;
    end
end
if idx~=255
    cippTrack(frmidx,1) = idx ; % [id x y vx xy state]
    cippTrack(frmidx,2) = g_tracks_VRU.output(idx).x;
    cippTrack(frmidx,3) = g_tracks_VRU.output(idx).y;
    cippTrack(frmidx,4) = g_tracks_VRU.output(idx).rel_vx;
    cippTrack(frmidx,5) = g_tracks_VRU.output(idx).rel_vy;
    cippTrack(frmidx,6) = g_tracks_VRU.output(idx).state;
    cippTrack(frmidx,7) = g_tracks_VRU.output(idx).cls;
    cippTrack(frmidx,8) = g_tracks_VRU.status(idx).cls1;
    cippTrack(frmidx,9) = g_tracks_VRU.output.TTC;

    if g_tracks_VRU.cursor == 1
        k = obstacle_const.track_depth;
    else
        k = g_tracks_VRU.cursor-1;
    end
    obsCIPP(frmidx,1) = g_tracks_VRU.matrix(k,idx).x;
    obsCIPP(frmidx,2) = g_tracks_VRU.matrix(k,idx).u;
    obsCIPP(frmidx,3) = g_tracks_VRU.matrix(k,idx).v;
    obsCIPP(frmidx,4) = g_tracks_VRU.matrix(k,idx).w;
    obsCIPP(frmidx,5) = g_tracks_VRU.matrix(k,idx).h;
    obsCIPP(frmidx,6) = g_tracks_VRU.matrix(k,idx).cls;
end

end