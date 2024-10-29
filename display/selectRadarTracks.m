% Description: find main targets of radar
% Author : hanhua                    
% log:
% 
%****************************************************************% 
function   [cRadar,lRadar,rRadar,radarTracksMat] = selectRadarTracks(cRadar,lRadar,rRadar,radarTracksMat,frmidx,obs,obsL,obsR,T)
% 对雷达轨迹进行筛选
% 原则：本车道内最近；距离摄像头输出距离最近；ID不跳变；RCS限制；
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%frmN = 1000;
tracksN = 100;
minRCS = 3;

global g_tracks_lane g_cfg_lane

% index
iNearest = uint8(255);  iNearCamera =  uint8(255);
iNearestL = uint8(255); iNearCameraL =  uint8(255);
iNearestR = uint8(255); iNearCameraR =  uint8(255);

% parameters
range = [0 20 50 80 100 150 180 200];
yLaneOffset = [0 0 0 0.3 0.5 0.5 0.5 0.5];
X = [4 6 10 16 20 24 28 32];
Y = [1 1 1.4 1.8 2.4 3 3.2 3.6];

minX = 200 ;  minXCIPV = 200 ;  minYCIPV = 3.2 ; minRCS = 3 ; 
minXL = 200 ; minXLobs = 200 ;  minYLobs = 3.2 ; minRCSL = 3;
minXR = 200 ; minXRobs = 200 ;  minYRobs = 3.2 ; minRCSR = 3; 

% 基于camera输出确定阈值
if any(obs(:,3))
    minXCIPV = interp1(range,X,obs(2),"linear",30)*1.5;
    minYCIPV = interp1(range,Y,obs(2),"linear",3.6)*1.2;
end
if any(obsL(:,3))
    minXLobs = interp1(range,X,obsL(2),"linear",30);
    minYLobs = interp1(range,Y,obsL(2),"linear",3.6);
end
if any(obsR(:,3))
    minXRobs = interp1(range,X,obsR(2),"linear",30);
    minYRobs = interp1(range,Y,obsR(2),"linear",3.6);
end
%

iId = cRadar.id;
iIdL = lRadar.id;
iIdR = rRadar.id;


% 查找对应id,iNearest iNearCamera
for i =1:tracksN
    if radarTracksMat(i).measures(frmidx,2)<1e-3
        continue;
    end
    yOffset = interp1(range,yLaneOffset,radarTracksMat(i).measures(frmidx,2),"linear",3.6);
    % 左右车道线界限
    if g_tracks_lane.l_line.state >= track_status.stable
        yl = polyval(g_tracks_lane.l_line.C(4:-1:1), radarTracksMat(i).measures(frmidx,2))+yOffset;
    else
        yl = g_cfg_lane.def_wid_half+yOffset;
    end
    if g_tracks_lane.r_line.state >= track_status.stable
        yr = polyval(g_tracks_lane.r_line.C(4:-1:1), radarTracksMat(i).measures(frmidx,2))-yOffset;
    else
        yr = -g_cfg_lane.def_wid_half-yOffset;
    end

    % 本车道内
    if radarTracksMat(i).measures(frmidx,3)<=yl && radarTracksMat(i).measures(frmidx,3)>=yr &&  radarTracksMat(i).measures(frmidx,6)>minRCS
        if radarTracksMat(i).measures(frmidx,2)<minX
            minX = radarTracksMat(i).measures(frmidx,2);
            iNearest = radarTracksMat(i).id;
        end

        % 左侧
    elseif radarTracksMat(i).measures(frmidx,3)>yl && radarTracksMat(i).measures(frmidx,3)<(yl+lane_const.cfg.def_wid_half*2)&& radarTracksMat(i).measures(frmidx,6)>minRCS
        if radarTracksMat(i).measures(frmidx,2)<minXL
            minXL = radarTracksMat(i).measures(frmidx,2);
            iNearestL = radarTracksMat(i).id;
        end

        % 右侧
    elseif  radarTracksMat(i).measures(frmidx,3)<yr && radarTracksMat(i).measures(frmidx,3)>(yr-lane_const.cfg.def_wid_half*2)&& radarTracksMat(i).measures(frmidx,6)>minRCS
        if radarTracksMat(i).measures(frmidx,2)<minXR
            minXR = radarTracksMat(i).measures(frmidx,2);
            iNearestR = radarTracksMat(i).id;
        end
    end
    % 最近
    if  (any(obs(1:3)) && abs(obs(2)-radarTracksMat(i).measures(frmidx,2))<minXCIPV && abs(obs(3)-radarTracksMat(i).measures(frmidx,3))<minYCIPV)
        minXCIPV = abs(obs(2)-radarTracksMat(i).measures(frmidx,2));
        minYCIPV = abs(obs(3)-radarTracksMat(i).measures(frmidx,3));
        iNearCamera = radarTracksMat(i).id;
    end
    if  (any(obsL(1:3)) && abs(obsL(2)-radarTracksMat(i).measures(frmidx,2))<minXLobs && abs(obsL(3)-radarTracksMat(i).measures(frmidx,3))<minYLobs)&& (i~=iNearCamera)
        minXLobs = abs(obsL(2)-radarTracksMat(i).measures(frmidx,2));
        minYLobs = abs(obsL(3)-radarTracksMat(i).measures(frmidx,3));
        iNearCameraL = radarTracksMat(i).id;
    end
    if  (any(obsR(1:3)) && abs(obsR(2)-radarTracksMat(i).measures(frmidx,2))<minXRobs && abs(obsR(3)-radarTracksMat(i).measures(frmidx,3))<minYRobs)&& (i~=iNearCamera)
        minXRobs = abs(obsR(2)-radarTracksMat(i).measures(frmidx,2));
        minYRobs = abs(obsR(3)-radarTracksMat(i).measures(frmidx,3));
        iNearCameraR = radarTracksMat(i).id;
    end
end
% 判断输出雷达ID
idx = 255; % 初始
% 当前车道，优先级原id
if iId~=0 && radarTracksMat(iId).measures(frmidx,2)>1e-3
    if any(obs(1:3)) && abs(radarTracksMat(iId).measures(frmidx,2)-obs(2))<=minXCIPV*1.2 ||...
            iId == iNearest %
        idx = iId;
    end
elseif iNearCamera~=255
    idx = iNearCamera;

elseif iNearest~=255
    idx = iNearest;
end

% 左
idxL = 255;
if iIdL~=0 && radarTracksMat(iIdL).measures(frmidx,2)>1e-3 && iIdL~=idx
    if any(obsL(1:3)) && abs(radarTracksMat(iIdL).measures(frmidx,2)-obsR(2))<minXLobs*1.2 ||...
            iIdL ==iNearestL  % 
        idxL = iIdL;
    end
elseif  iNearCameraL~=255
    idxL = iNearCameraL;
elseif iNearestL ~=255
    idxL = iNearestL;
end
% 右
idxR = 255;
if iIdR~=0 && radarTracksMat(iIdR).measures(frmidx,2)>1e-3 && idxR~=idx
    if any(obsR(1:3)) && abs(radarTracksMat(iIdR).measures(frmidx,2)-obsR(2))<minXRobs*1.2 ||...
            iIdR ==iNearestR %|| iNearCameraR==255
        idxR = iIdR;
    end
elseif  iNearCameraR~=255
    idxR = iNearCameraR;
elseif iNearestR ~=255
    idxR = iNearestR;
end
%=====取值=========
if idx~=255
    if ~radarTracksMat(idx).interpFlag
        [radarTracksMat] = interpRadarTracks(radarTracksMat,idx,T);
    end
    %     if idx==mRadarId.id
    %         mRadarId.idx(frmidx)=idx;
    %     else
    %
    %     end
    cRadar.id = uint8(idx);
    % [time id x,y,vx,vy,RCS]
    cRadar.data(frmidx,1) = T(frmidx);
    cRadar.data(frmidx,2) = idx; %
    cRadar.data(frmidx,3) = radarTracksMat(idx).measures(frmidx,2);
    cRadar.data(frmidx,4) = radarTracksMat(idx).measures(frmidx,3);
    cRadar.data(frmidx,5) = radarTracksMat(idx).measures(frmidx,4);
    cRadar.data(frmidx,6) = radarTracksMat(idx).measures(frmidx,5);
    cRadar.data(frmidx,7) = radarTracksMat(idx).measures(frmidx,6);
else
    cRadar.id = uint8(0);
    cRadar.data(frmidx,1) = T(frmidx);
end
%----左-----
if idxL~=255
    if ~radarTracksMat(idxL).interpFlag
        [radarTracksMat] = interpRadarTracks(radarTracksMat,idxL,T);
    end
    lRadar.id = uint8(idxL);
    % [time id x,y,vx,vy,RCS]
    lRadar.data(frmidx,1) = T(frmidx);
    lRadar.data(frmidx,2) = (idxL); %
    lRadar.data(frmidx,3) = radarTracksMat(idxL).measures(frmidx,2);
    lRadar.data(frmidx,4) = radarTracksMat(idxL).measures(frmidx,3);
    lRadar.data(frmidx,5) = radarTracksMat(idxL).measures(frmidx,4);
    lRadar.data(frmidx,6) = radarTracksMat(idxL).measures(frmidx,5);
    lRadar.data(frmidx,7) = radarTracksMat(idxL).measures(frmidx,6);
else
    lRadar.id = uint8(0);
    lRadar.data(frmidx,1) =  T(frmidx);
end

%-----右-------
if idxR~=255
    if ~radarTracksMat(idxR).interpFlag
        [radarTracksMat] = interpRadarTracks(radarTracksMat,idxR,T);
    end
    rRadar.id = uint8(idxR);
    % [time id x,y,vx,vy,RCS]
    rRadar.data(frmidx,1) = T(frmidx);
    rRadar.data(frmidx,2) = idxR; %
    rRadar.data(frmidx,3) = radarTracksMat(idxR).measures(frmidx,2);
    rRadar.data(frmidx,4) = radarTracksMat(idxR).measures(frmidx,3);
    rRadar.data(frmidx,5) = radarTracksMat(idxR).measures(frmidx,4);
    rRadar.data(frmidx,6) = radarTracksMat(idxR).measures(frmidx,5);
    rRadar.data(frmidx,7) = radarTracksMat(idxR).measures(frmidx,6);
else
    rRadar.id = uint8(0);
    rRadar.data(frmidx,1) =  T(frmidx);
end


end


function [radarTracksMat] = interpRadarTracks(radarTracksMat,idx,T)
% [time x y vx vy RCS]
frmN = 1000 ;
% 分段插值
startFlag = false;
endFlag = false ;
if radarTracksMat(idx).cnt>20
    for k = 1:frmN
        if radarTracksMat(idx).measures(k,1)>0 && ~startFlag
            idxS = k;
            startFlag = true;
            idxE = idxS;
        end
        if startFlag && radarTracksMat(idx).measures(k,1)==0
            if radarTracksMat(idx).measures(k-1,1)>0
                idxE = k-1;
            end
            % 连续为0或阶跃是跳变10m
            if k-idxE>10 || ...
                    (abs(radarTracksMat(idx).measures(k,2)-radarTracksMat(idx).measures(idxE,2))>10 && radarTracksMat(idx).measures(k,2)>1e-3)
                endFlag = true;
            end
        end
        if endFlag
            startFlag = false ;
            endFlag = false ;
            if idxE-idxS>1
                idxSE = idxS:idxE;
                idxIn = radarTracksMat(idx).measures(idxSE)>0;
                % 对选出的片段进行插值；
                radarTracksMat(idx).measures(idxSE,2) = interp1(T(idxSE(idxIn)),radarTracksMat(idx).measures(idxSE(idxIn),2),T(idxSE),'linear'); % x
                radarTracksMat(idx).measures(idxSE,3) = interp1(T(idxSE(idxIn)),radarTracksMat(idx).measures(idxSE(idxIn),3),T(idxSE),'linear'); % y
                radarTracksMat(idx).measures(idxSE,4) = interp1(T(idxSE(idxIn)),radarTracksMat(idx).measures(idxSE(idxIn),4),T(idxSE),'linear'); % vx
                radarTracksMat(idx).measures(idxSE,5) = interp1(T(idxSE(idxIn)),radarTracksMat(idx).measures(idxSE(idxIn),5),T(idxSE),'linear'); % vy
                radarTracksMat(idx).measures(idxSE,6) = interp1(T(idxSE(idxIn)),radarTracksMat(idx).measures(idxSE(idxIn),6),T(idxSE),'linear'); % RCS
            else
                fprintf('起始帧：%d 结束帧：%d ；不做插值处理\n',idxS,idxE);
            end
        end
    end
    radarTracksMat(idx).interpFlag = uint8(1);
end

end