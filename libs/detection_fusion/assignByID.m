function matchID = assignByID(frameData,tracks,params)
% 根据历史摄像头和雷达的匹配信息，确定雷达和摄像头的匹配结果，避免出现阈值不合适造成的雷达和摄像头造成的误匹配
% frameData: 单帧雷达和摄像头的数据
% tracks: 历史轨迹信息，记录雷达和摄像头的关联情况
matchID = zeros(obstacle_const.max_obs_camera, obstacle_const.max_obs_radar_front);
CameraObjectList = frameData.CameraFrame.CameraObjectList;
RadarObjectList =  frameData.RadarFrame.RadarObjectList(1:obstacle_const.max_obs_radar_front);
flag_Tused = zeros(obstacle_const.track_width,1,"uint8");   % 记录航迹是否check完
flag_Rused = zeros(obstacle_const.max_obs_radar,1,"uint8"); % 记录Radar是否check完
for j = 1:1:length(CameraObjectList)
    if CameraObjectList(j).flag == uint8(1)
        camID_current = CameraObjectList(j).ID;
%         x_current_cam =  CameraObjectList(j).x;
%         y_current_cam = CameraObjectList(j).y;
%         vx_current_cam = CameraObjectList(j).velo_x;
%         vy_current_cam = CameraObjectList(j).velo_y;
        for i=1:obstacle_const.track_width
            if tracks.status(i).used && flag_Tused(i) == uint8(0)
                if tracks.status(i).flag
                    obs_XY = tracks.matrix(tracks.cursor, i);
                else
                    obs_XY = tracks.matrix(prev_cursor(tracks.cursor, obstacle_const.track_depth), i);
                end
                if obs_XY.obj_det_prop == obs_det_prop.fused  % 只对上一帧有融合状态的目标进行操作
                    camID_pre = obs_XY.fusedCamID;            % 找到上一帧摄像头的CamID
                    if camID_current == camID_pre
                        flag_Tused(i,1) = uint8(1);
%                         x_pre = obs_XY.x;
%                         y_pre = obs_XY.y;
%                         disCost = abs(x_pre-x_current_cam)+abs(y_pre-y_current_cam);
                        radID_pre = obs_XY.fusedRadID;        % 找到上一帧雷达的RadarID
                        for m = 1:obstacle_const.max_obs_radar_front
                            if flag_Rused(m,1) == uint8(0) && RadarObjectList(m).ID == radID_pre &&RadarObjectList(m).det_src == det_src.front 
                                % 判断关联误差是否增大
                                if tracks.status(i).CNT >= obstacle_const.cfg.thres_IDassign && ...
                                    tracks.status(i).CNTCam>=obstacle_const.cfg.thres_IDassign ...
                                    && tracks.status(i).CNTRad>=obstacle_const.cfg.thres_IDassign...
                                    && (tracks.status(i).start_Range <=100 || ...
                                    (tracks.status(i).start_Range >100 && obs_XY.y < 100)|| ...
                                    (tracks.status(i).start_Range >100 && tracks.status(i).matchErrorFlag <1))
                                    % 航迹帧数大于20,稳定航迹用ID assign
                                    % %航迹起始大于100，需要更新传感器ID
                                    matchID(j,m) = 1;
                                    flag_Rused(m,1) = uint8(1);
%                                     x_current_radar = RadarObjectList(m).x;
%                                     y_current_radar = RadarObjectList(m).y;
%                                     vx_current_radar = RadarObjectList(m).velo_x;
%                                     vy_current_radar = RadarObjectList(m).velo_y;
%                                     disCost_x_camRad = abs(x_current_radar-x_current_cam);
%                                     disCost_camRad = abs(x_current_radar-x_current_cam)+abs(y_current_cam-y_current_radar);

%                                     velCost_camRad = abs(vx_current_radar-vx_current_cam) +abs(vy_current_cam-vy_current_radar);
%                                     ay = interp1(params.y,params.a_y,y_current_cam,'linear',params.a_y(end));
%                                     if disCost_camRad < ay && velCost_camRad < ay && disCost<20 &&  disCost_x_camRad<=2.2 % temp 阈值需要调整(关联阈值放宽，防止雷达和摄像头关联不上)
%                                         matchID(j,m) = 1;
%                                         flag_Rused(m,1) = uint8(1);
%                                     end
%                                     matchID(j,m) = 1;
%                                     flag_Rused(m,1) = uint8(1);
%                                 else
%                                     x_current_radar = RadarObjectList(m).x;
%                                     y_current_radar = RadarObjectList(m).y;
%                                     vx_current_radar = RadarObjectList(m).velo_x;
%                                     vy_current_radar = RadarObjectList(m).velo_y;
%                                     disCost_x_camRad = abs(x_current_radar-x_current_cam);
%                                     disCost_camRad = abs(x_current_radar-x_current_cam)+abs(y_current_cam-y_current_radar);
%                                     velCost_camRad = abs(vx_current_radar-vx_current_cam) +abs(vy_current_cam-vy_current_radar);
%                                     ay = interp1(params.y,params.a_y,y_current_cam,'linear',params.a_y(end));
%                                     if disCost_camRad < ay && velCost_camRad < ay && disCost<20 &&  disCost_x_camRad<=2 % temp 阈值需要调整(关联阈值放宽，防止雷达和摄像头关联不上)
%                                         matchID(j,m) = 1;
%                                         flag_Rused(m,1) = uint8(1);
%                                     end
                                    break
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end




