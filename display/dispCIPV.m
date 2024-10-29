function dispCIPV(cipvTrack,lTrack,rTrack,frmIDs,radarFlag,cipvRadar,lobsRadar,robsRadar,saveVideo,name)
% 显示数据回放过程中cipv的id; state;[x,y,vx,vy]

    h1 = figure('color', 'w', 'position', [10, 10, 1200, 800], 'name', 'CIPV', 'visible', 'on');
    subplot(2,6,1)
    plot(frmIDs,cipvTrack(frmIDs,1),'.-.');
    xlabel('frmIDs');ylabel('id');grid on;
    hold on;
    plot(frmIDs,lTrack(frmIDs,1),'.-');
    plot(frmIDs,rTrack(frmIDs,1),'.-');
    legend('cipv','left','right');
    subplot(2,6,2)
    plot(frmIDs,cipvTrack(frmIDs,6),'.-.');
    xlabel('frmIDs');ylabel('state');grid on;
    hold on;
    plot(frmIDs,lTrack(frmIDs,6),'.-.');
    plot(frmIDs,rTrack(frmIDs,6),'.-.');
    legend('cipv','left','right');
    subplot(2,6,3)
    plot(frmIDs,cipvTrack(frmIDs,2),'.-.');
    xlabel('frmIDs');ylabel('x');grid on;
    subplot(2,6,4)
    plot(frmIDs,cipvTrack(frmIDs,3),'.-.');
    xlabel('frmIDs');ylabel('y');grid on;
    subplot(2,6,5)
    plot(frmIDs,cipvTrack(frmIDs,4),'.-.');
    xlabel('frmIDs');ylabel('vx');grid on;
    hold on;
    subplot(2,6,6)
    plot(frmIDs,cipvTrack(frmIDs,5),'.-.');
    xlabel('frmIDs');ylabel('vy');grid on;


    subplot(2,6,7)
    plot(frmIDs,lTrack(frmIDs,2),'.-.');
    xlabel('frmIDs');ylabel('x');grid on;
    title('leftLane x');
    subplot(2,6,8)
    plot(frmIDs,lTrack(frmIDs,3),'.-.');
    xlabel('frmIDs');ylabel('y');grid on;
    title('leftLane y');
    subplot(2,6,9)
    plot(frmIDs,lTrack(frmIDs,4),'.-.');
    xlabel('frmIDs');ylabel('vx');grid on;
    title('leftLane vx')

    subplot(2,6,10)
    plot(frmIDs,rTrack(frmIDs,2),'.-.');
    xlabel('frmIDs');ylabel('x');grid on;
    title('rightLane x');
    subplot(2,6,11)
    plot(frmIDs,rTrack(frmIDs,3),'.-.');
    xlabel('frmIDs');ylabel('y');grid on;
    title('rightLane y');
    subplot(2,6,12)
    plot(frmIDs,rTrack(frmIDs,4),'.-.');
    xlabel('frmIDs');ylabel('vx');grid on;
    title('rightLane vx');

    if radarFlag  % %[time id x,y,vx,vy,RCS]
        idxr = cipvRadar(frmIDs,3)>0;
        % 同时有值R&C
        idxc = cipvTrack(frmIDs,2)>0;
        idx = idxr&idxc;

        idxL = lobsRadar(frmIDs,3)>0;
        idxR = robsRadar(frmIDs,3)>0;

        subplot(2,6,3)
        hold on;
        plot(frmIDs(idxr),cipvRadar(frmIDs(idxr),3),'.');
        legend('cipv','cipvRadar');
        subplot(2,6,4)
        hold on;
        plot(frmIDs(idx),cipvRadar(frmIDs(idx),4),'.');
        legend('cipv','cipvRadar');
        subplot(2,6,5) 
        hold on;
        plot(frmIDs(idx),cipvRadar(frmIDs(idx),5),'.');
        legend('cipv','cipvRadar');
        subplot(2,6,6)
        hold on;
        plot(frmIDs(idx),cipvRadar(frmIDs(idx),6),'.');
        legend('cipv','cipvRadar');


        subplot(2,6,7)
        hold on;
        plot(frmIDs(idxL),lobsRadar(frmIDs(idxL),3),'.');
        legend('left','leftRadar');
        subplot(2,6,8)
        hold on;
        plot(frmIDs(idxL),lobsRadar(frmIDs(idxL),4),'.');
        legend('left','leftRadar');
        subplot(2,6,9)
        hold on;
        plot(frmIDs(idxL),lobsRadar(frmIDs(idxL),5),'.');
        legend('left','leftRadar');


        subplot(2,6,10)
        hold on;
        plot(frmIDs(idxR),robsRadar(frmIDs(idxR),3),'.');
        legend('right','rightRadar');
        subplot(2,6,11)
        hold on;
        plot(frmIDs(idxR),robsRadar(frmIDs(idxR),4),'.');
        legend('right','rightRadar');
        subplot(2,6,12)
        hold on;
        plot(frmIDs(idxR),robsRadar(frmIDs(idxR),5),'.');
        legend('right','rightRadar');

        %  error
        h2 = figure('color','w','Name','滤波差值');
        subplot(3,1,1)
        error = cipvTrack(frmIDs(idx),2)-cipvRadar(frmIDs(idx),3);
        plot(frmIDs(idx),error,'o')
        hold on;
        plot(frmIDs(idxr),cipvRadar(frmIDs(idxr),3)*0.1,'.');     
        plot(frmIDs,zeros(length(frmIDs),1),'k-');
        plot(frmIDs(idx),abs(error),'.')
        set(gca,'YTick',(-12:2:12));
        legend('差值','0.1*radar测距','绝对误差')
        xlabel('frmIDs');ylabel('error'); grid on;
        title('camerTrack-Radar');

        subplot(3,1,2)
        errorL = lTrack(frmIDs(idxL),2)-lobsRadar(frmIDs(idxL),3);
        plot(frmIDs(idxL),errorL,'o')
        ylim([-12 12]);
        xlabel('frmIDs');ylabel('error'); grid on;
        title('left camerTrack-Radar');

        subplot(3,1,3)
        errorR = rTrack(frmIDs(idxR),2)-robsRadar(frmIDs(idxR),3);
        plot(frmIDs(idxR),errorR,'o')
        ylim([-12 12]);
        xlabel('frmIDs');ylabel('error'); grid on;
        title('right camerTrack-Radar');


    end
    if saveVideo
        saveas(h1,[name,'.fig']);
        if radarFlag
            saveas(h2,[name,'_error.fig']);
        end
    end
end



