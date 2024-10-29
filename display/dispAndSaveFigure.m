% Description: history display & save
% Author : hanhua                    
% log:
% 20220913：新增filter lane参数画图--hanhua; 
% 20220914: 新增估计自车道信息保存及画图--hanhua; 
%****************************************************************% 
%% 存储radar cipv
% [cRadar] = hostIdRadar(cRadar,frmID1);
% [lRadar] = hostIdRadar(lRadar,frmID1);
% [rRadar] = hostIdRadar(rRadar,frmID1);
% if saveCipvRadar
%     cipvRadar = cRadar.data;
%     lobsRadar = lRadar.data;
%     robsRadar = rRadar.data;
%     idx = (cipvRadar(:,1)==0);
%     cipvRadar(idx,:)=[];lobsRadar(idx,:)=[]; robsRadar(idx,:)=[];
%     save(fullfile(dat_path, [name, '_radarCIPV.mat']),'cipvRadar','lobsRadar','robsRadar');
%     % 保存所有雷达轨迹
%     save(fullfile(dat_path, [name, '_radarTracksMat.mat']),'radarTracksMat');
% end
%==========================================================================
%% 显示数据回放过程中cipv的id; state;[x,y,vx,vy]
h1 = figure('color', 'w', 'position', [10, 10, 1275, 645], 'name', [name,'CIPV'], 'visible', 'on');
% subplot(2,2,1)
% plot(frmIDs,cipvTrack(frmIDs,2),'.-.',LineWidth=1,Color='r');
% hold on
% plot(frmIDs,cameracipv(frmIDs,2),'.-.',LineWidth=1,Color='g');
% plot(frmIDs,radarcipv(frmIDs,2),'.-.',LineWidth=1,Color='b');
% xlabel('frmIDs');ylabel('x');grid on;
% legend('cipv','camera','radar');
% subplot(2,2,2)
% plot(frmIDs,cipvTrack(frmIDs,3),'.-.',LineWidth=1,Color='r');
% hold on
% plot(frmIDs,cameracipv(frmIDs,3),'.-.',LineWidth=1,Color='g');
% plot(frmIDs,radarcipv(frmIDs,3),'.-.',LineWidth=1,Color='b');
% xlabel('frmIDs');ylabel('y');grid on;
% subplot(2,2,3)
% plot(frmIDs,cipvTrack(frmIDs,4),'.-.',LineWidth=1,Color='r');
% hold on
% plot(frmIDs,cameracipv(frmIDs,4),'.-.',LineWidth=1,Color='g');
% plot(frmIDs,radarcipv(frmIDs,4),'.-.',LineWidth=1,Color='b');
% xlabel('frmIDs');ylabel('vx');grid on;
% subplot(2,2,4)
% plot(frmIDs,cipvTrack(frmIDs,5),'.-.',LineWidth=1,Color='r');
% hold on
% plot(frmIDs,cameracipv(frmIDs,5),'.-.',LineWidth=1,Color='g');
% plot(frmIDs,radarcipv(frmIDs,5),'.-.',LineWidth=1,Color='b');
% xlabel('frmIDs');ylabel('vy');grid on;

figure;

% 第一组绘图
subplot(2, 2, 1); % 创建子图 2x2 的第一块
plot(frmIDs, cipvTrack(frmIDs, 2), '.-.', 'LineWidth', 1, 'Color', 'r');
hold on;
plot(frmIDs, cameracipv(frmIDs, 2), '.-.', 'LineWidth', 1, 'Color', 'g');
plot(frmIDs, radarcipv(frmIDs, 2), '.-.', 'LineWidth', 1, 'Color', 'b');
xlabel('frmIDs'); ylabel('x'); grid on;
legend('cipv', 'camera', 'radar');

% 第二组绘图
subplot(2, 2, 2); % 创建子图 2x2 的第二块
plot(frmIDs, cipvTrack(frmIDs, 3), '.-.', 'LineWidth', 1, 'Color', 'r');
hold on;
plot(frmIDs, cameracipv(frmIDs, 3), '.-.', 'LineWidth', 1, 'Color', 'g');
plot(frmIDs, radarcipv(frmIDs, 3), '.-.', 'LineWidth', 1, 'Color', 'b');
xlabel('frmIDs'); ylabel('y'); grid on;

% 第三组绘图
subplot(2, 2, 3); % 创建子图 2x2 的第三块
plot(frmIDs, cipvTrack(frmIDs, 4), '.-.', 'LineWidth', 1, 'Color', 'r');
hold on;
plot(frmIDs, cameracipv(frmIDs, 4), '.-.', 'LineWidth', 1, 'Color', 'g');
plot(frmIDs, radarcipv(frmIDs, 4), '.-.', 'LineWidth', 1, 'Color', 'b');
xlabel('frmIDs'); ylabel('vx'); grid on;

% 第四组绘图
subplot(2, 2, 4); % 创建子图 2x2 的第四块
plot(frmIDs, cipvTrack(frmIDs, 5), '.-.', 'LineWidth', 1, 'Color', 'r');
hold on;
plot(frmIDs, cameracipv(frmIDs, 5), '.-.', 'LineWidth', 1, 'Color', 'g');
plot(frmIDs, radarcipv(frmIDs, 5), '.-.', 'LineWidth', 1, 'Color', 'b');
xlabel('frmIDs'); ylabel('vy'); grid on;

legend('cipv', 'camera', 'radar'); % 可选，在所有子图中共用一个图例



frmMatrix =  [cipvTrack(frmIDs,1:6),cameracipv(frmIDs,1:6),radarcipv(frmIDs,1:5)];
header = {'fused_ID', 'fused_x', 'fused_y','fused_velo_x','fused_velo_y','fused_cls'...
    'camera_ID', 'camera_x', 'camera_y','camera_velo_x','camera_velo_y','camera_cls',...
    'radar_ID', 'radar_x', 'radar_y','radar_velo_x','radar_velo_y'};
data_with_header = [header; num2cell(frmMatrix)];
camera_name = strcat(frameData_name, '.csv');
% radar_name = strcat(frameData_name, '_radar', '.csv');
file_path = 'G:\01_Personal\MDCU_V204_20240105\dataset\2024-09-25\camera_test'+camera_name;

% 将矩阵数据保存为 CSV 文件
writecell(data_with_header,file_path);
% subplot(2,6,1)
% plot(frmIDs,cipvTrack(frmIDs,1),'.-.');
% xlabel('frmIDs');ylabel('id');grid on;
% hold on;
% plot(frmIDs,lTrack(frmIDs,1),'.-');
% plot(frmIDs,rTrack(frmIDs,1),'.-');
% legend('cipv','left','right');
% subplot(2,6,2)
% plot(frmIDs,cipvTrack(frmIDs,6),'.-.');
% xlabel('frmIDs');ylabel('state');grid on;
% hold on;
% plot(frmIDs,lTrack(frmIDs,6),'.-.');
% plot(frmIDs,rTrack(frmIDs,6),'.-.');
% legend('cipv','left','right');
% subplot(2,6,3)
% plot(frmIDs,cipvTrack(frmIDs,2),'.-.');
% xlabel('frmIDs');ylabel('x');grid on;
% subplot(2,6,4)
% plot(frmIDs,cipvTrack(frmIDs,3),'.-.');
% xlabel('frmIDs');ylabel('y');grid on;
% subplot(2,6,5)
% plot(frmIDs,cipvTrack(frmIDs,4),'.-.');
% xlabel('frmIDs');ylabel('vx');grid on;
% hold on;
% subplot(2,6,6)
% plot(frmIDs,cipvTrack(frmIDs,5),'.-.');
% xlabel('frmIDs');ylabel('vy');grid on;
% subplot(2,6,7)
% plot(frmIDs,lTrack(frmIDs,2),'.-.');
% xlabel('frmIDs');ylabel('x');grid on;
% title('leftLane x');
% subplot(2,6,8)
% plot(frmIDs,lTrack(frmIDs,3),'.-.');
% xlabel('frmIDs');ylabel('y');grid on;
% title('leftLane y');
% subplot(2,6,9)
% plot(frmIDs,lTrack(frmIDs,4),'.-.');
% xlabel('frmIDs');ylabel('vx');grid on;
% title('leftLane vx')
% 
% subplot(2,6,10)
% plot(frmIDs,rTrack(frmIDs,2),'.-.');
% xlabel('frmIDs');ylabel('x');grid on;
% title('rightLane x');
% subplot(2,6,11)
% plot(frmIDs,rTrack(frmIDs,3),'.-.');
% xlabel('frmIDs');ylabel('y');grid on;
% title('rightLane y');
% subplot(2,6,12)
% plot(frmIDs,rTrack(frmIDs,4),'.-.');
% xlabel('frmIDs');ylabel('vx');grid on;
% title('rightLane vx');

% if radarFlag  % %[time id x,y,vx,vy,RCS]
%     idxr = cRadar.data(frmIDs,3)>0;
%     % 同时有值R&C
%     idxc = cipvTrack(frmIDs,2)>0;
%     idx = idxr&idxc;
% 
%     idxL = lRadar.data(frmIDs,3)>0;
%     idxR = rRadar.data(frmIDs,3)>0;
% 
%     subplot(2,6,3)
%     hold on;
%     plot(frmIDs(idxr),cRadar.data(frmIDs(idxr),3),'.');
%     legend('cipv','cRadar.data');
%     subplot(2,6,4)
%     hold on;
%     plot(frmIDs(idx),cRadar.data(frmIDs(idx),4),'.');
%     legend('cipv','cRadar.data');CRightMatrix
%     subplot(2,6,5)
%     hold on;
%     plot(frmIDs(idx),cRadar.data(frmIDs(idx),5),'.');
%     legend('cipv','cRadar.data');
%     subplot(2,6,6)
%     hold on;
%     plot(frmIDs(idx),cRadar.data(frmIDs(idx),6),'.');
%     legend('cipv','cRadar.data');
% 
% 
%     subplot(2,6,7)
%     hold on;
%     plot(frmIDs(idxL),lRadar.data(frmIDs(idxL),3),'.');
%     legend('left','leftRadar');
%     subplot(2,6,8)
%     hold on;
%     plot(frmIDs(idxL),lRadar.data(frmIDs(idxL),4),'.');
%     legend('left','leftRadar');
%     subplot(2,6,9)
%     hold on;
%     plot(frmIDs(idxL),lRadar.data(frmIDs(idxL),5),'.');
%     legend('left','leftRadar');
% 
% 
%     subplot(2,6,10)
%     hold on;
%     plot(frmIDs(idxR),rRadar.data(frmIDs(idxR),3),'.');
%     legend('right','rightRadar');
%     subplot(2,6,11)
%     hold on;
%     plot(frmIDs(idxR),rRadar.data(frmIDs(idxR),4),'.');
%     legend('right','rightRadar');
%     subplot(2,6,12)
%     hold on;
%     plot(frmIDs(idxR),rRadar.data(frmIDs(idxR),5),'.');
%     legend('right','rightRadar');
%--------------------------------------------------------------------------
    %  error
%     h2 = figure('color','w','Name',[name,'滤波差值']);
%     subplot(3,1,1)
%     error = cipvTrack(frmIDs(idx),2)-cRadar.data(frmIDs(idx),3);
%     plot(frmIDs(idx),error,'o')
%     hold on;
%     plot(frmIDs(idxr),cRadar.data(frmIDs(idxr),3)*0.1,'.');
%     plot(frmIDs,zeros(length(frmIDs),1),'k-');
%     plot(frmIDs(idx),abs(error),'.')
%     set(gca,'YTick',(-12:2:12));
%     legend('差值','0.1*radar测距','绝对误差')
%     xlabel('frmIDs');ylabel('error'); grid on;
%     title('camerTrack-Radar');
% 
%     subplot(3,1,2)
%     errorL = lTrack(frmIDs(idxL),2)-lRadar.data(frmIDs(idxL),3);
%     plot(frmIDs(idxL),errorL,'o')
%     ylim([-12 12]);
%     xlabel('frmIDs');ylabel('error'); grid on;
%     title('left camerTrack-Radar');
% 
%     subplot(3,1,3)
%     errorR = rTrack(frmIDs(idxR),2)-rRadar.data(frmIDs(idxR),3);
%     plot(frmIDs(idxR),errorR,'o')
%     ylim([-12 12]);
%     xlabel('frmIDs');ylabel('error'); grid on;
%     title('right camerTrack-Radar');
% 
% 
% end
% if saveVideo
%     saveas(h1,[name,'.fig']);
%     if radarFlag
%         saveas(h2,[name,'_error.fig']);
%     end
% end
% %====================================================================
% %% 显示数据回放过程中% [x,y,x1,x2,u,v,w,h,r,cls];
% 
% h3 = figure('color', 'w', 'position', [10, 10, 1200, 800], 'name', [name,'_obsCIPV'], 'visible', 'on');
% subplot(2,3,1)
% hold on;
% plot(frmIDs,obsCIPV(frmIDs,1),'o-.'); % x
% plot(frmIDs,obsCIPV(frmIDs,3),'.-.'); % x1
% plot(frmIDs,obsCIPV(frmIDs,4),'.-.'); % x2
% 
% % 修正后的
% plot(frmIDs,obsCIPV(frmIDs,11),'.-.'); % x3
% %   plot(frmIDs,obsCIPV(frmIDs,13),'.-.'); % xW
% 
% legend('加权','投影','宽度','车道宽')
% xlabel('frmIDs');ylabel('x');grid on;
% title('测距')
% 
% subplot(2,3,2)
% hold on;
% plot(frmIDs,obsCIPV(frmIDs,2),'.-.');
% 
% %     % 修正后
% %     plot(frmIDs,obsCIPV(frmIDs,12),'.-.'); % yP
% %     legend('投影','修正投影');
% 
% xlabel('frmIDs');ylabel('y');grid on;
% 
% subplot(2,3,3)
% hold on;
% plot(frmIDs,obsCIPV(frmIDs,5),'.-.'); % u
% plot(frmIDs,obsCIPV(frmIDs,6),'.-.'); % v
% plot(frmIDs,obsCIPV(frmIDs,6)+obsCIPV(frmIDs,8),'.-.'); % bottom
% legend('u','v','bottom(v+h)');
% xlabel('frmIDs');ylabel('pixel');grid on;
% 
% 
% subplot(2,3,4)
% hold on;
% plot(frmIDs,obsCIPV(frmIDs,7),'.-.'); % w
% plot(frmIDs,obsCIPV(frmIDs,8),'.-.'); % h
% legend('w','h')
% xlabel('frmIDs');ylabel('pixel');grid on;
% 
% 
% subplot(2,3,5)
% plot(frmIDs,obsCIPV(frmIDs,9),'.-.'); % r
% xlabel('frmIDs');ylabel('r');grid on;
% 
% subplot(2,3,6)
% plot(frmIDs,obsCIPV(frmIDs,10),'.-.'); % cls
% hold on;
% plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % direction
% legend('cls','direction');
% xlabel('frmIDs');ylabel('cls & direction');grid on;
% %==========================================================================
% if radarFlag  % [time id x,y,vx,vy,RCS]
%     h4 = figure('Color','w','name',[name,'测距']) ;
%     subplot(1,3,1)
%     idx = cRadar.data(frmIDs,3)>0;
%     %subplot(1,2,1)
%     ltype = '.';
%     lsize = 9;
%     hold on;
%     plot(frmIDs,obsCIPV(frmIDs,1),'o'); % x
%     plot(frmIDs,obsCIPV(frmIDs,3),ltype,'markersize',lsize); % x1
%     plot(frmIDs,obsCIPV(frmIDs,4),ltype,'markersize',lsize); % x2
%     plot(frmIDs(idx),cRadar.data(frmIDs(idx),3),ltype,'markersize',lsize); % rada
%     plot(frmIDs,obsCIPV(frmIDs,11),ltype,'markersize',lsize); % x3
% 
%     %   plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % xW
% 
%     legend('加权','投影','宽度','radar','车道宽')
%     xlabel('frmIDs');ylabel('x');grid on;
%     title('测距')
% 
%     % 左侧车道
%     subplot(1,3,2)  % lRadar.data
%     idx = lRadar.data(frmIDs,3)>0;
%     %subplot(1,2,1)
%     hold on;
%     plot(frmIDs,obsL(frmIDs,1),'o'); % x
%     plot(frmIDs,obsL(frmIDs,3),ltype,'markersize',lsize); % x1
%     plot(frmIDs,obsL(frmIDs,4),ltype,'markersize',lsize); % x2
%     plot(frmIDs(idx),lRadar.data(frmIDs(idx),3),ltype,'markersize',lsize); % radar
%     plot(frmIDs,obsL(frmIDs,11),ltype,'markersize',lsize); % x3
%     % plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % xW
% 
%     legend('加权','投影','宽度','radar','车道宽')
%     xlabel('frmIDs');ylabel('x');grid on;
%     title('左侧测距')
% 
%     % 右侧车道
%     subplot(1,3,3)  % lRadar.data
%     idx = rRadar.data(frmIDs,3)>0;
%     %subplot(1,2,1)
%     hold on;
%     plot(frmIDs,obsR(frmIDs,1),'o'); % x
%     plot(frmIDs,obsR(frmIDs,3),ltype,'markersize',lsize); % x1
%     plot(frmIDs,obsR(frmIDs,4),ltype,'markersize',lsize); % x2
%     plot(frmIDs(idx),rRadar.data(frmIDs(idx),3),ltype,'markersize',lsize); % radar
% 
%     plot(frmIDs,obsR(frmIDs,11),ltype,'markersize',lsize); % x3
%     % plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % xW
% 
%     legend('加权','投影','宽度','radar','车道宽')
%     xlabel('frmIDs');ylabel('x');grid on;
%     title('右侧测距')
% 
%     %     subplot(1,2,2)
%     %     hold on;
%     %     plot(frmIDs(idx),obsCIPV(frmIDs(idx),1)-cRadar.data(frmIDs(idx),3),'.-.');  % x-radar
%     %     plot(frmIDs(idx),obsCIPV(frmIDs(idx),3)-cRadar.data(frmIDs(idx),3),'.-.'); % x1-radar
%     %     plot(frmIDs(idx),obsCIPV(frmIDs(idx),4)-cRadar.data(frmIDs(idx),3),'.-.'); % x2-radar
%     %     legend('加权-radar','投影-radar','宽度-radar');
%     %     xlabel('frmIDs');ylabel('error');grid on;
%     %     ylim([-12 12]);
%     %     title('差值')
% 
% end
% 
% if saveVideo && radarFlag
%     saveas(h4,[name,'_singleFrm.fig']);
% end
%==========================================================================
%% 反推车宽结果
% h5 = figure('Color','w','name',[name,'车宽计算']) ;
% title('车宽')
% plot(frmIDs,obsCIPV(frmIDs,12),'.--'); % obsW1
% hold on;
% plot(frmIDs,cipvTrack(frmIDs,7),'.--'); % cls
% plot(frmIDs,cipvTrack(frmIDs,8),'.--'); % cls
% legend('基于投影w','output-cls','估计cls1');
%==========================================================================
%% 模糊测距选择结果
% h6=figure('Color','w','name',[name,'fuzzy']) ;
% ltype = '.-.';
% lsize = 9;
% subplot(121);hold on;
% plot(frmIDs,obsCIPV(frmIDs,3),ltype,'markersize',lsize); % x1
% plot(frmIDs,obsCIPV(frmIDs,4),ltype,'markersize',lsize); % x2
% plot(frmIDs,obsCIPV(frmIDs,11),ltype,'markersize',lsize); % x3
% plot(frmIDs,cRadar.data(frmIDs,3),ltype,'markersize',lsize); % rad
% plot(Fx,'o') % fuzzy
% plot(frmIDs,cipvTrack(frmIDs,2),'.-');
% plot(frmIDs,0.5*obsCIPV(frmIDs,3)+0.5*obsCIPV(frmIDs,4),'+','markersize',2);
% legend('投影','宽度','车道宽','radar','fuzzySelect','cameraTrack','0.5x1+0.5x2')
% grid on;
% subplot(122); hold on;
% plot(frmIDs,cipvTrack(frmIDs,4),ltype);
% plot(frmIDs,cipvTrack(frmIDs,9),ltype);
% plot(frmIDs,cipvTrack(frmIDs,10),ltype);
% plot(frmIDs,-carSignals(frmIDs,1)/3.6,ltype);
% plot(frmIDs,3.5*ones(length(frmIDs),1),'-');
% plot(frmIDs,2.7*ones(length(frmIDs),1),'-');
% legend('rel vx','TTC','rel status','-1*egoCar speed','level 1 waring','level 2 waring');grid on;
%==========================================================================
%% 瞬态补偿值
% h7= figure('Color','w','name',[name,'瞬态补偿']) ;
% title('瞬态补偿')
% plot(frmIDs(1:length(pitches)),pitches,'.-.')
% grid on;
%==========================================================================
%% 车道线方程参数，单帧&滤波后
% h8 = figure('Color','w','name',[name,'车道线方程']) ;
% subplot(2,2,1); hold on;
% plot(frmIDs,CLeftMatrix(frmIDs,1),ltype,'markersize',lsize); % C0
% plot(frmIDs,CLeftMatrix(frmIDs,5),ltype,'markersize',lsize-4); % C0
% xlabel('frmIDs');ylabel('C0'); grid on;
% title('Left Line C0');
% subplot(2,2,2); hold on;
% plot(frmIDs,CLeftMatrix(frmIDs,2),ltype,'markersize',lsize); % C1
% plot(frmIDs,CLeftMatrix(frmIDs,6),ltype,'markersize',lsize-4); % C1
% xlabel('frmIDs');ylabel('C1'); grid on;
% title('Left Line C1');
% subplot(2,2,3); hold on;
% plot(frmIDs,CRightMatrix(frmIDs,1),ltype,'markersize',lsize); % C0
% plot(frmIDs,CRightMatrix(frmIDs,5),ltype,'markersize',lsize-4); % C0
% xlabel('frmIDs');ylabel('C0'); grid on;
% title('Right Line C0');
% subplot(2,2,4); hold on;
% plot(frmIDs,CRightMatrix(frmIDs,2),ltype,'markersize',lsize); % C1
% plot(frmIDs,CRightMatrix(frmIDs,6),ltype,'markersize',lsize-4); % C1
% xlabel('frmIDs');ylabel('C1'); grid on;
% title('Right Line C1');
% %% 为方便观测LDW，将车道线延长至轮子位置
% h9 = figure('Color','w','name',[name,'延长至车轮']);
% subplot(121); hold on;
% plot(frmIDs,CLeftMatrix(frmIDs,11),'.-','markersize',5); % wheel_y
% plot(frmIDs,g_ego_params.width/2*ones(length(frmIDs),1),'r--')
% xlabel('frmIDs');ylabel('lane to wheel y'); grid on;title('left lane to wheel')
% subplot(122); hold on;
% plot(frmIDs,CRightMatrix(frmIDs,11),'.-','markersize',5); % wheel_y
% plot(frmIDs,-g_ego_params.width/2*ones(length(frmIDs),1),'r--')
% xlabel('frmIDs');ylabel('lane to wheel y'); grid on;title('right lane to wheel')
%==========================================================================
%% 估计出的车道信息，宽度 曲率 半径等
% h10 = figure('Color','w','name',[name,'车道信息']) ;
% ms = 4;
% subplot(2,2,1);
% plot(frmIDs,laneParams(frmIDs,1),'.--','markersize',ms);
% xlabel('frmIDs');ylabel('m'); grid on;
% title('host lane width');
% subplot(2,2,2);
% plot(frmIDs,laneParams(frmIDs,2),'.--','markersize',ms);
% xlabel('frmIDs');ylabel('1/m'); grid on;
% title('radius');
% subplot(2,2,3);hold on;
% plot(frmIDs,laneParams(frmIDs,3),'.--','markersize',ms);
% xlabel('frmIDs');ylabel('degree/s'); grid on;
% title('yawrate');
% plot(frmIDs,rad2deg(carSignals(frmIDs,2)),'.','markersize',ms);
% legend('基于车道线估计','自车输出'); 
% subplot(2,2,4);
% plot(frmIDs,laneParams(frmIDs,4),'.--','markersize',ms);
% xlabel('frmIDs');ylabel('m/s'); grid on;
% title('lateral speed');
% %==========================================================================
% %% 行人CIPV
% h11 = figure('Color','w','name',[name,'行人']) ;
% plot(frmIDs,cippTrack(frmIDs,2),'.-'); hold on;
% plot(frmIDs,obsCIPP(frmIDs,1),'.-');
% legend('pedTrack','投影');
% if radarFlag
% end
% h12 = figure('Color','w','name',[name,'ped bbox']) ;
% subplot(121); hold on;
% plot(frmIDs,obsCIPP(frmIDs,2),'.-'); 
% plot(frmIDs,obsCIPP(frmIDs,3),'.-'); 
% plot(frmIDs,obsCIPP(frmIDs,3)+obsCIPP(frmIDs,5),'.-'); 
% legend('u','r','bottom')
% subplot(122);hold on;
% plot(frmIDs,obsCIPP(frmIDs,4),'.-');
% plot(frmIDs,obsCIPP(frmIDs,5),'.-'); 
% legend('w','h'); 
%==========================================================================
%% 是否保存；
if saveVideo
      saveas(h1,[name,'_cipv.fig']);
%     saveas(h3,[name,'_cipv.fig']);
%     saveas(h5,[name,'_width.fig']);
%     saveas(h6,[name,'_fuzzy.fig']);
%     saveas(h7,[name,'_pitch.fig']);
%     saveas(h8,[name,'_lineC.fig']);
%     saveas(h10,[name,'_lineTackingC.fig']);
end


