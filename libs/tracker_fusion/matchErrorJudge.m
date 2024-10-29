function[matchErrorFlag] = matchErrorJudge(tracks,jdx)
% 判断关联误差有没有持续增大
% x_error_history = tracks(jdx).x_error_history;
y_error_history = tracks.status(jdx).y_error_history;
matchErrorFlag = uint8(0);
if tracks.status(jdx).CNTRad>=7
    % 计算连续6个数之间的差值
%     differences = diff(y_error_history);

    % 使用polyfit拟合差值的趋势
    p = polyfit(1:length(y_error_history), y_error_history, 1);

    % 判断拟合的斜率是否大于0
    if p(1) > 0.2 && mean(y_error_history) >=5
        matchErrorFlag = uint8(1); % 如果斜率大于0，则表示存在逐渐增大的趋势
    else
        matchErrorFlag = uint8(0); % 否则表示趋势不增大
    end

end
end

