function dispObsCIPV(obsCIPV,obsL,obsR,frmIDs,radarFlag,cipvRadar,lobsRadar,robsRadar,saveVideo,name)
% 显示数据回放过程中% [x,y,x1,x2,u,v,w,h,r,cls];

    h1 = figure('color', 'w', 'position', [10, 10, 1200, 800], 'name', 'obsCIPV', 'visible', 'on');
    subplot(2,3,1)
    hold on;
    plot(frmIDs,obsCIPV(frmIDs,1),'o-.'); % x
    plot(frmIDs,obsCIPV(frmIDs,3),'.-.'); % x1
    plot(frmIDs,obsCIPV(frmIDs,4),'.-.'); % x2
    
    % 修正后的
    plot(frmIDs,obsCIPV(frmIDs,11),'.-.'); % x3
%     plot(frmIDs,obsCIPV(frmIDs,13),'.-.'); % xW

    legend('加权','投影','宽度','车道宽')
    xlabel('frmIDs');ylabel('x');grid on;
    title('测距')

    subplot(2,3,2)
    hold on;
    plot(frmIDs,obsCIPV(frmIDs,2),'.-.');

%     % 修正后
%     plot(frmIDs,obsCIPV(frmIDs,12),'.-.'); % yP
%     legend('投影','修正投影'); 

    xlabel('frmIDs');ylabel('y');grid on;

    subplot(2,3,3)
    hold on;
    plot(frmIDs,obsCIPV(frmIDs,5),'.-.'); % u
    plot(frmIDs,obsCIPV(frmIDs,6),'.-.'); % v
    legend('u','v')
    xlabel('frmIDs');ylabel('pixel');grid on;
    

    subplot(2,3,4)
    hold on;
    plot(frmIDs,obsCIPV(frmIDs,7),'.-.'); % w
    plot(frmIDs,obsCIPV(frmIDs,8),'.-.'); % h
    legend('w','h')
    xlabel('frmIDs');ylabel('pixel');grid on;
    

    subplot(2,3,5)
    plot(frmIDs,obsCIPV(frmIDs,9),'.-.'); % r
    xlabel('frmIDs');ylabel('r');grid on;

    subplot(2,3,6)
    plot(frmIDs,obsCIPV(frmIDs,10),'.-.'); % cls
    xlabel('frmIDs');ylabel('cls');grid on;

if radarFlag  % [time id x,y,vx,vy,RCS]
    h2 = figure('Color','w','name','测距') ;
    subplot(1,3,1)
    idx = cipvRadar(frmIDs,3)>0;
    %subplot(1,2,1)
    ltype = '.';
    lsize = 9;
    hold on;
    plot(frmIDs,obsCIPV(frmIDs,1),'o'); % x
    plot(frmIDs,obsCIPV(frmIDs,3),ltype,'markersize',lsize); % x1
    plot(frmIDs,obsCIPV(frmIDs,4),ltype,'markersize',lsize); % x2
    plot(frmIDs(idx),cipvRadar(frmIDs(idx),3),ltype,'markersize',lsize); % rada
    plot(frmIDs,obsCIPV(frmIDs,11),ltype,'markersize',lsize); % x3

%   plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % xW

    legend('加权','投影','宽度','radar','车道宽')
    xlabel('frmIDs');ylabel('x');grid on;
    title('测距')

    % 左侧车道
    subplot(1,3,2)  % lobsRadar
    idx = lobsRadar(frmIDs,3)>0;
    %subplot(1,2,1)
    hold on;
    plot(frmIDs,obsL(frmIDs,1),'o'); % x
    plot(frmIDs,obsL(frmIDs,3),ltype,'markersize',lsize); % x1
    plot(frmIDs,obsL(frmIDs,4),ltype,'markersize',lsize); % x2
    plot(frmIDs(idx),lobsRadar(frmIDs(idx),3),ltype,'markersize',lsize); % radar
    plot(frmIDs,obsL(frmIDs,11),ltype,'markersize',lsize); % x3
    % plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % xW

    legend('加权','投影','宽度','radar','车道宽')
    xlabel('frmIDs');ylabel('x');grid on;
    title('左侧测距')

    % 右侧车道
    subplot(1,3,3)  % lobsRadar
    idx = robsRadar(frmIDs,3)>0;
    %subplot(1,2,1)
    hold on;
    plot(frmIDs,obsR(frmIDs,1),'o'); % x
    plot(frmIDs,obsR(frmIDs,3),ltype,'markersize',lsize); % x1
    plot(frmIDs,obsR(frmIDs,4),ltype,'markersize',lsize); % x2
    plot(frmIDs(idx),robsRadar(frmIDs(idx),3),ltype,'markersize',lsize); % radar

    plot(frmIDs,obsR(frmIDs,11),ltype,'markersize',lsize); % x3
    % plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % xW

    legend('加权','投影','宽度','radar','车道宽')
    xlabel('frmIDs');ylabel('x');grid on;
    title('右侧测距')

    %     subplot(1,2,2)
    %     hold on;
    %     plot(frmIDs(idx),obsCIPV(frmIDs(idx),1)-cipvRadar(frmIDs(idx),3),'.-.');  % x-radar
    %     plot(frmIDs(idx),obsCIPV(frmIDs(idx),3)-cipvRadar(frmIDs(idx),3),'.-.'); % x1-radar
    %     plot(frmIDs(idx),obsCIPV(frmIDs(idx),4)-cipvRadar(frmIDs(idx),3),'.-.'); % x2-radar
    %     legend('加权-radar','投影-radar','宽度-radar');
    %     xlabel('frmIDs');ylabel('error');grid on;
    %     ylim([-12 12]);
    %     title('差值') 

end

h3 = figure('Color','w','name','车宽计算') ;
title('车宽')
plot(frmIDs,obsCIPV(frmIDs,12),'.--'); % obsW1
hold on;
plot(frmIDs,obsCIPV(frmIDs,13),'.--'); % obsW2
legend('基于投影','朝向');

if saveVideo && radarFlag
    saveas(h2,[name,'_singleFrm.fig']);
    saveas(h3,[name,'_width.fig']);
end

end