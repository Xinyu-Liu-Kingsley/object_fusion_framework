function setTableValue(uitableFused,uitableCamera,uitableRadar,frameData,eobstacles)
set(uitableFused, 'Data', [])
set(uitableCamera, 'Data', [])
set(uitableRadar, 'Data', [])
radar_matrix = zeros(obstacle_const.max_obs_radar,6);
camera_matrix = zeros(obstacle_const.max_obs_camera,6);
fuse_matrix = cell(obstacle_const.cfg.EOut_vehs_num,19);
cameraObjects = frameData.CameraFrame.CameraObjectList;
radarObjects = frameData.RadarFrame.RadarObjectList;
% global g_tracks_auto;
cnt = 0;
for j=1:obstacle_const.max_obs_camera
    if cameraObjects(j).flag
        cnt = cnt + 1;
        camera_matrix(cnt,1) = cameraObjects(j).ID;
        camera_matrix(cnt,2) = cameraObjects(j).x;
        camera_matrix(cnt,3) = cameraObjects(j).y;
        camera_matrix(cnt,4) = cameraObjects(j).cls;
        camera_matrix(cnt,5) = cameraObjects(j).velo_x;
        camera_matrix(cnt,6) = cameraObjects(j).velo_y;
    end
end
set(uitableCamera, 'Data', camera_matrix)
cnt1 = 0;
for j=1:obstacle_const.max_obs_radar
    if radarObjects(j).flag
        cnt1 = cnt1 + 1;
        radar_matrix(cnt1,1) = radarObjects(j).ID;
        radar_matrix(cnt1,2) = radarObjects(j).x;
        radar_matrix(cnt1,3) = radarObjects(j).y;
        radar_matrix(cnt1,4) = radarObjects(j).cls;
        radar_matrix(cnt1,5) = radarObjects(j).velo_x;
        radar_matrix(cnt1,6) = radarObjects(j).velo_y;
    end
end
set(uitableRadar, 'Data', radar_matrix)


ecnt = 0;
% column_name_fused = {...
%     'ID_fused','CamID','RadID','CIPV','CIPP','x','y','cls','motionStatus', ...
%     'motionCategory','velo_x','velo_y','width','obj_lane','img_lane'};
% for j=1:obstacle_const.cfg.EOut_vehs_num
%     if ecnt < obstacle_const.max_obs && eobstacles(j).state >= track_status.initial
%         ecnt = ecnt+1;
%         fuse_matrix(ecnt,1) = eobstacles(j).ID;
%         fuse_matrix(ecnt,2) = eobstacles(j).fusedCamID;
%         fuse_matrix(ecnt,3) = eobstacles(j).fusedRadID;
%         fuse_matrix(ecnt,4) = eobstacles(j).CIPV;
%         fuse_matrix(ecnt,5) = eobstacles(j).CIPP;
%         fuse_matrix(ecnt,6) = eobstacles(j).x;
%         fuse_matrix(ecnt,7) = eobstacles(j).y;
%         fuse_matrix(ecnt,8) = eobstacles(j).cls;
%         fuse_matrix(ecnt,9) = eobstacles(j).motionStatus;
%         fuse_matrix(ecnt,10) = eobstacles(j).motionCategory;
%         fuse_matrix(ecnt,11) = eobstacles(j).velo_x;
%         fuse_matrix(ecnt,12) = eobstacles(j).velo_y;
%         fuse_matrix(ecnt,13) = eobstacles(j).obj_width;
%         fuse_matrix(ecnt,14) = eobstacles(j).obj_lane;
%         fuse_matrix(ecnt,15) = eobstacles(j).img_lane;
%         fuse_matrix(ecnt,16) = eobstacles(j).moveType;
%         fuse_matrix(ecnt,17) = eobstacles(j).obsolute_res;
%         
%     end
% end
for j=1:obstacle_const.cfg.EOut_vehs_num
    if ecnt < obstacle_const.max_obs && eobstacles(j).state >= track_status.initial
        ecnt = ecnt+1;
        fuse_matrix{ecnt,1} = eobstacles(j).ID;
        fuse_matrix{ecnt,2} = eobstacles(j).fusedCamID;
        fuse_matrix{ecnt,3} = eobstacles(j).fusedRadID;
        fuse_matrix{ecnt,4} = eobstacles(j).CIPV;
        fuse_matrix{ecnt,5} = eobstacles(j).CIPP;
        fuse_matrix{ecnt,6} = eobstacles(j).x;
        fuse_matrix{ecnt,7} = eobstacles(j).y;
        fuse_matrix{ecnt,8} = char(eobstacles(j).cls);
        fuse_matrix{ecnt,9} = char(eobstacles(j).motionStatus);
        fuse_matrix{ecnt,10} = char(eobstacles(j).motionCategory);
        fuse_matrix{ecnt,11} = eobstacles(j).velo_x;
        fuse_matrix{ecnt,12} = eobstacles(j).velo_y;
        fuse_matrix{ecnt,13} = eobstacles(j).obj_width;
        fuse_matrix{ecnt,14} = char(eobstacles(j).obj_lane);
        fuse_matrix{ecnt,15} = char(eobstacles(j).img_lane);
        moveType = obstacle_motionStatus.unknown;
        motionCategory = obstacle_motionCategory.UNDEFINED;
          % 目标运动状态 0：运动；1：绝对静止
          % 目标运动属性 0：未知；1：靠近；2：远离；3：相对雷达静止；
        if eobstacles(j).moveType == 0
            moveType = obstacle_motionStatus.moving;
        else
            moveType = obstacle_motionStatus.stationary;
        end
        if eobstacles(j).obsolute_res == 1
            motionCategory = obstacle_motionCategory.MOVING_IN;
        elseif eobstacles(j).obsolute_res == 2
            motionCategory = obstacle_motionCategory.MOVING_OUT;
        elseif eobstacles(j).obsolute_res == 3
            motionCategory = obstacle_motionCategory.MOVING;
        end
        fuse_matrix{ecnt,16} = char(moveType);
        fuse_matrix{ecnt,17} = char(motionCategory);
        fuse_matrix{ecnt,18} = eobstacles(j).TTC;
        fuse_matrix{ecnt,19} = char(eobstacles(j).det_src);
%         fuse_matrix{ecnt,16} = eobstacles(j).moveType;
%         fuse_matrix{ecnt,17} = eobstacles(j).obsolute_res;
    end
end
set(uitableFused, 'Data', fuse_matrix(1:ecnt,:))
% set(uitableFused, 'Data',{'we',2})
end
