function eVal = win_filtering(win, values)    %#codegen
% Description: 窗口滤波，且剔除最大、最小值
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
% win       - 滤波窗口，M x 1
% cursor    - 当前游标位置，标量
% values    - M x N矩阵，M跟踪深度，N变量数量
% eVal      - 滤波结果，1 x N

[M, N] = size(values);    % 变量数量

assert(isa(win, 'double'))
assert(isa(values, 'double'))

assert(M == length(win)+2)

flag = uint8(0);

if flag == uint8(1)
    % 直接取平均值
    eVal = mean(values, 1);
elseif flag == uint8(2)
    % 剔除最大、最小值之后取平均值
    eVal = (sum(values, 1) - max(values, [], 1) - min(values, [], 1)) / (m - 2);
else
%     I = [cursor:-1:1, M:-1:cursor+1];       % 历史轨迹序号
%     I = 1:M;       % 历史轨迹序号
    [~, Imax] = max(values, [], 1);    % 最大值
    [~, Imin] = min(values, [], 1);    % 最小值
    coef = zeros(M, N);                     % 滤波矩阵
    for i=1:N
        if Imax(i) == Imin(i) 	% 最大值位置与最小值位置相同，则表示序列中所有值都相同
            coef(1:M-2, i) = win;
            continue
        else                    % 对于非最大值/最小值的位置填充滤波器系数
            idx = 0;            % 计数器，最大为M - 2
            for j=1:M
%                 assert(I(j)~=Imax(i) || I(j)~=Imin(i))  % 确认最大值位置与最小值位置不相同
                assert(idx <= M-2)
                if j ~= Imax(i) && j ~= Imin(i)
                    idx = idx + 1;
                    coef(j, i) = win(idx);
                end
            end
        end
    end
    eVal = sum(values .* coef, 1);
end

end

