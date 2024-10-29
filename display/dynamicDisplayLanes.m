% 为验证车道线方程与图像上车道线检测关键点重合度而显示
clf
image(frm,'AlphaData', 0.8);
hold on;
% if ~isempty(lanes_uv_select)
%     for ii=1:numel(lanes_uv_select)
%         len = lanes_uv_select(ii).len;
%         if len > 0
%             scatter(lanes_uv_select(ii).u(1:len), lanes_uv_select(ii).v(1:len), 10, 'filled','color',[0.4660 0.6740 0.1880])
%         end
%     end
% end
% [K_Rw2c,~] = lookup_R();
% lanes_me投影回图像上面
%%
% lanes_c2uv = repmat(lane_const.UV, 1, lane_const.max_lns);
% for j = 1:lane_const.max_lns
% 
%     if lanes_me(j).flag ~= uint8(0)
%         len = lanes_xy_select(j).len;
%         x = lanes_xy(j).x(1:len);
%         y = polyval(flip(lanes_me(j).C), x);
%         uv = xy2uv([x -y], K_Rw2c, g_cam_params.T);
%         n = len;
%         lanes_c2uv(j).len = n;
%         lanes_c2uv(j).u(1:n) = uv(:,1);
%         lanes_c2uv(j).v(1:n) = uv(:,2);
% 
%         plot(uv(:,1),uv(:,2),'o','markersize',5,'lineWidth',1,'Color',[0,0.8,1]);
%     end
% end
% title(sprintf('No.%d', frmIDs(i)),'Interpreter', 'latex');

%%
% 基于lane_uv去做比较；
[~, Rc2w_m_invK] = lookup_R();
[K_Rw2c,~] = lookup_R();
lane_uv_compare = repmat(lane_const.UV, 1, lane_const.max_lns);
lanes_c2uv = repmat(lane_const.UV, 1, lane_const.max_lns);
for kk = 1:lane_const.max_lns
    if lanes_me(kk).flag ~= uint8(0)
        len = lanes_uv(kk).len;
        % 投影
        Pw = uv2xy([lanes_uv(kk).u(1:len), lanes_uv(kk).v(1:len)], Rc2w_m_invK, g_cam_params.H, g_cam_params.T);
        % 选取车道线预设范围内的投影点
        idx = Pw(:,1)>=g_cfg_lane.x_coverage(1) & Pw(:,1)<=g_cfg_lane.x_coverage(2);
        len1 = sum(idx);
        lane_uv_compare(kk).len = len1;
        lane_uv_compare(kk).u(1:len1) = lanes_uv(kk).u(idx);
        lane_uv_compare(kk).v(1:len1) = lanes_uv(kk).v(idx);
        lane_uv_compare(kk).index = lanes_me(kk).index;

        x = Pw(idx,1);
        y = polyval(flip(lanes_me(kk).C), x);
        uv = xy2uv([x -y], K_Rw2c, g_cam_params.T);
        n = len1;
        lanes_c2uv(kk).len = n;
        lanes_c2uv(kk).u(1:n) = uv(:,1);
        lanes_c2uv(kk).v(1:n) = uv(:,2);
        lanes_c2uv(kk).index = lanes_me(kk).index;

        plot(uv(:,1),uv(:,2),'o','markersize',5,'lineWidth',1,'Color',[0,0.8,1]);

        scatter(lane_uv_compare(kk).u(1:len1), lane_uv_compare(kk).v(1:len1), 10, 'filled','color',[0.4660 0.6740 0.1880])

    end
end

title(sprintf('No.%d', frmIDs(i)),'Interpreter', 'latex');


%%
% uv_var = 
for jj = 1:lane_const.max_lns
    if lane_uv_compare(jj).index == 1       % left
        len = lane_uv_compare(jj).len;
        uv_var_l = abs(lane_uv_compare(jj).u(1:len) - lanes_c2uv(jj).u(1:len));
        lanes_uv_var(frmIDs(i),1) = mean(uv_var_l);
        lanes_uv_var(frmIDs(i),2) = var(uv_var_l);
    elseif lane_uv_compare(jj).index == 2   % right
        len = lane_uv_compare(jj).len;
        uv_var_r = abs(lane_uv_compare(jj).u(1:len) - lanes_c2uv(jj).u(1:len));
        lanes_uv_var(frmIDs(i),3) = mean(uv_var_r);
        lanes_uv_var(frmIDs(i),4) = var(uv_var_r);
    end
end
%%
%   lanes_uv_var_new = lanes_uv_var;
%   save('lanes_uv_var458.mat','lanes_uv_var_new');