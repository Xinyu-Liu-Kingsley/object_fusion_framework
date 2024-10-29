
radTar = [];
%% ====================== 画图 ====================== %%
if  displayFlag
    % 显示画面
    title(['frmIdx = ',num2str(frmIDs(i))]);
    % 显示图像信息
    showImageFusion(ax1, frm,sprintf('Version.%8.2f', eobstacles(1).versionNum));
    % 融合数据和bev视角联动
    %     ax2 = axes;
    %     ax2.Position = [0.44,0.4,0.3,0.58];
    %     set(ax2, 'XLabel', [], 'YLabel', []);
    plotResultsFusion(ax2,frameDataSingle,eobstacles, sprintf('No.%d ', frmIDs(i)));
    % 显示MDCU融合结果
    if displayMDCU
       frameDataSingleMDCU = mbdProcessFrame(frmDataMDCU(frmIDs(i)));
       plotResultsFusionMDCU(ax2,frameDataSingleMDCU);
    end

    %     plotResults(ax, lanes_eo.line, lanes_xy, obstacles, eobstacles,pedstrains,radTar, sprintf('No.%d pitch - %.2f', frmIDs(i), pitches(i)))
    setTableValue(uitableFused,uitableCamera,uitableRadar,frameDataSingle,eobstacles);
    [cipvTrack,lTrack,rTrack,cameracipv,radarcipv] = saveCIPV(cipvTrack,eobstacles,frmIDs(i),lTrack,rTrack,cameracipv,radarcipv,frameDataSingle);
    fusionTrack{frmIDs(i)} = eobstacles;
    % 显示cipv
    % 显示瞬态校正值
    %     ax = subplot(2,2,3);
    %     plot(ax, pitches,'.-.')
    %     ylim([-3 3]);
    %     set(gca,'YTick',(-3:1:3));
    % %     ylabel('dynamic pitch angle $\deg$', 'Interpreter', 'latex')
    %     grid on
    %     box on
end




%% 存储CIPV
function [cipvTrack,lTrack,rTrack,cameracipv,radarcipv] = saveCIPV(cipvTrack,eobstacles,frmidx,lTrack,rTrack,cameracipv,radarcipv,frameDataSingle)
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
    cipvTrack(frmidx,4) = eobstacles(idx).velo_x;
    cipvTrack(frmidx,5) = eobstacles(idx).velo_y;
    cipvTrack(frmidx,6) = eobstacles(idx).cls;
    cipvTrack(frmidx,7) = eobstacles(idx).state;
    for row_camera=1:obstacle_const.max_obs_camera
        if frameDataSingle.CameraFrame.CameraObjectList(row_camera).ID==eobstacles(idx).fusedCamID && frameDataSingle.CameraFrame.CameraObjectList(row_camera).det_src==eobstacles(idx).det_src
            cameracipv(frmidx,1) = frameDataSingle.CameraFrame.CameraObjectList(row_camera).ID;
            cameracipv(frmidx,2) = frameDataSingle.CameraFrame.CameraObjectList(row_camera).x;
            cameracipv(frmidx,3) = frameDataSingle.CameraFrame.CameraObjectList(row_camera).y;
            cameracipv(frmidx,4) = frameDataSingle.CameraFrame.CameraObjectList(row_camera).velo_x;
            cameracipv(frmidx,5) = frameDataSingle.CameraFrame.CameraObjectList(row_camera).velo_y;
            cameracipv(frmidx,6) = frameDataSingle.CameraFrame.CameraObjectList(row_camera).cls;
        end
    end
    for row_radar=1:obstacle_const.max_obs_radar
        if frameDataSingle.RadarFrame.RadarObjectList(row_radar).ID==eobstacles(idx).fusedRadID && frameDataSingle.RadarFrame.RadarObjectList(row_radar).det_src==eobstacles(idx).det_src
            radarcipv(frmidx,1) = frameDataSingle.RadarFrame.RadarObjectList(row_radar).ID;
            radarcipv(frmidx,2) = frameDataSingle.RadarFrame.RadarObjectList(row_radar).x;
            radarcipv(frmidx,3) = frameDataSingle.RadarFrame.RadarObjectList(row_radar).y;
            radarcipv(frmidx,4) = frameDataSingle.RadarFrame.RadarObjectList(row_radar).velo_x;
            radarcipv(frmidx,5) = frameDataSingle.RadarFrame.RadarObjectList(row_radar).velo_y;
        end
    end 
%     cipvTrack(frmidx,8) = g_tracks_auto.status(idx).cls1;
%     cipvTrack(frmidx,9) = eobstacles(idx).TTC;
%     cipvTrack(frmidx,10) = g_tracks_auto.status(idx).rel_status(2);
    % lstm
%     cipvTrack(frmidx,11) = g_tracks_auto.lstmPredictor(idx).x;
%     cipvTrack(frmidx,12) = g_tracks_auto.lstmPredictor(idx).vx;
%     %EKF
%     cipvTrack(frmidx,13) = g_tracks_auto.ekfOutput(idx).x;
%     cipvTrack(frmidx,14) = g_tracks_auto.ekfOutput(idx).vx;

% else
%     cipvTrack(frmidx,9) = obstacle_const.defTTC;
end
%=================左右车道最近轨迹========================
global g_tracks_lane g_cfg_lane
% minXL = 200; idxL = 255;
% minXR = 200; idxR = 255;
% for i = 1:m
%     if eobstacles(i).x == 0
%         continue;
%     end
%     if  eobstacles(i).x<minXL || eobstacles(i).x<minXR
%         % 左右车道线界限
%         if g_tracks_lane.l_line.state >= track_status.stable
%             yl = polyval(g_tracks_lane.l_line.C(4:-1:1), eobstacles(i).x);
%         else
%             yl = g_cfg_lane.def_wid_half;
%         end
%         if g_tracks_lane.r_line.state >= track_status.stable
%             yr = polyval(g_tracks_lane.r_line.C(4:-1:1), eobstacles(i).x);
%         else
%             yr = -g_cfg_lane.def_wid_half;
%         end
%         % 左侧
%         if eobstacles(i).y>yl && eobstacles(i).y<(yl+lane_const.cfg.def_wid_half*2)&&...
%                 eobstacles(i).x<minXL
%             minXL = eobstacles(i).x;
%             idxL = i;
%         end
%         % 右侧
%         if eobstacles(i).y<yr && eobstacles(i).y>(yr-lane_const.cfg.def_wid_half*2)&&...
%                 eobstacles(i).x<minXR
%             minXR = eobstacles(i).x;
%             idxR = i;
%         end
%     end
% end
% if idxL~=255
%     lTrack(frmidx,1) = idxL ; % [id x y vx xy state]
%     lTrack(frmidx,2) = eobstacles(idxL).x;
%     lTrack(frmidx,3) = eobstacles(idxL).y;
%     lTrack(frmidx,4) = eobstacles(idxL).rel_vx;
%     lTrack(frmidx,5) = eobstacles(idxL).rel_vy;
%     lTrack(frmidx,6) = eobstacles(idxL).state;
% end
% if idxR~=255
%     rTrack(frmidx,1) = idxR ; % [id x y vx xy state]
%     rTrack(frmidx,2) = eobstacles(idxR).x;
%     rTrack(frmidx,3) = eobstacles(idxR).y;
%     rTrack(frmidx,4) = eobstacles(idxR).rel_vx;
%     rTrack(frmidx,5) = eobstacles(idxR).rel_vy;
%     rTrack(frmidx,6) = eobstacles(idxR).state;
% end

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