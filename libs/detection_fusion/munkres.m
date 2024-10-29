function [M,cost] = munkres(costMat)
%%#codegen
% MUNKRES   Munkres (Hungarian) Algorithm for Linear Assignment Problem.
%
% [ASSIGN,COST] = munkres(COSTMAT) returns the optimal column indices,
% ASSIGN assigned to each row and the minimum COST based on the assignment
% problem represented by the COSTMAT, where the (i,j)th element represents the cost to assign the jth
% job to the ith worker.
%
% Partial assignment: This code can identify a partial assignment is a full
% assignment is not feasible. For a partial assignment, there are some
% zero elements in the returning assignment vector, which indicate
% un-assigned tasks. The cost returned only contains the cost of partially
% assigned tasks.

% This is vectorized implementation of the algorithm. It is the fastest
% among all Matlab implementations of the algorithm.

% Examples
% Example 1: a 5 x 5 example
%{
[assignment,cost] = munkres(magic(5));
disp(assignment); % 3 2 1 5 4
disp(cost); %15
%}
% Example 2: 400 x 400 random data
%{
n=400;
A=rand(n);
tic
[a,b]=munkres(A);
toc                 % about 2 seconds
%}
% Example 3: rectangular assignment with inf costs
%{
A=rand(10,7);
A(A>0.7)=Inf;
[a,b]=munkres(A);
%}
% Example 4: an example of partial assignment
%{
A = [1 3 Inf; Inf Inf 5; Inf Inf 0.5];
[a,b]=munkres(A)
%}
% a = [1 0 3]
% b = 1.5
% Reference:
% "Munkres' Assignment Algorithm, Modified for Rectangular Matrices",
% http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html

eps = 1e-1; % 精度1e-5
assignment  = zeros(1, size(costMat, 1));
cost        = zeros(1, 1, class(costMat));
M           = zeros(size(costMat), class(costMat));
validMat    = (abs(costMat - costMat)< eps) & (costMat < Inf); % 筛选有限数据，精度不能太高
bigM        = 10^(ceil(log10(sum(costMat(validMat))))+1);
costMat(~validMat) = bigM;

validCol = any(validMat, 1);
validRow = any(validMat, 2);

nRows = sum(validRow);
nCols = sum(validCol);
n = max(nRows,nCols);
if ~n
    return
end

maxv=10 * max(costMat(validMat));

dMat = zeros(n) + maxv;
dMat(1:nRows,1:nCols) = costMat(validRow,validCol);

%*************************************************
% Munkres' Assignment Algorithm starts here
%*************************************************

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   STEP 1: Subtract the row minimum from each row.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
minR = min(dMat,[],2);
% 每一行减去行最小值后，得到的矩阵，提取列最小值
minC = min(bsxfun(@minus, dMat, minR));

%**************************************************************************
%   STEP 2: Find a zero of dMat. If there are no starred zeros in its
%           column or row start the zero. Repeat for each zero
%**************************************************************************
% zP等于每一行减去每行最小值后的矩阵内，得到0所在位置，下面标星使用
zP = abs(dMat - bsxfun(@plus, minC, minR)) < eps;

starZ = zeros(n,1);
while any(zP(:)) % zP == 1标星，每行只标一个星
    [r, c] = find(zP, 1);
    starZ(r) = c;
    zP(r,:) = false;
    zP(:,c) = false;
end

while 1
    %**************************************************************************
    %   STEP 3: Cover each column with a starred zero. If all the columns are
    %           covered then the matching is maximum
    %**************************************************************************
    if all(starZ > 0)
        % 如果所有行都标星，找到最优解，结束
        break
    end
    coverColumn = false(1, n);
    coverColumn(starZ(starZ > 0)) = true; % 得到已覆盖的列，已划线
    coverRow = false(n,1);
    primeZ = zeros(n,1);
    % 找到未划线的矩阵，，每列减去列内最小值，得到0的位置，rIdx行号，cIdx列号，有多少列就找到多少个0
   
    [rIdx, cIdx] = find(abs(dMat(~coverRow,~coverColumn) - bsxfun(@plus,minR(~coverRow),minC(~coverColumn))) < eps);
   
    % 如果all(starZ > 0)条件不成立，一定存在未覆盖的列，则[rIdx, cIdx]不可能为空
    cR = find(~coverRow);               % 原dMat未覆盖row
    cC = (find(~coverColumn))';         % 原dMat未覆盖col
    initRow = cR(rIdx);                 % 原dMat索引row, 0所在位置
    initCol = cC(cIdx);                 % 原dMat索引col，0所在位置
    uZr = initRow(1);                   % 未覆盖的列中，第一个0所在行
    uZc = initCol(1);                   % 未覆盖的列中，第一个0所在列
    while 1
        %**************************************************************************
        %   STEP 4: Find a noncovered zero and prime it.  If there is no starred
        %           zero in the row containing this primed zero, Go to Step 5.
        %           Otherwise, cover this row and uncover the column containing
        %           the starred zero. Continue in this manner until there are no
        %           uncovered zeros left. Save the smallest uncovered value and
        %           Go to Step 6.
        %**************************************************************************
        cR = find(~coverRow);
        cC = (find(~coverColumn))';
        rIdx = cR(rIdx);
        cIdx = cC(cIdx);
        Step = 6;
        while ~isempty(cIdx)
            % 直到cIdx为空，说明没有需要刻记的行了，跳出循环， to step 6
            uZr = rIdx(1);
            uZc = cIdx(1);
            % 刻记uZr行
            primeZ(uZr) = uZc;
            stz = starZ(uZr);
            % ~stz 说明uZr行未曾标星，
            if ~stz
                % 若未标星，却刻记，to step 5
                Step = 5;
                break;
            end
            % 若已标星，也刻记：刻记的行划线，标星的列不划线，重新调整覆盖区域
            coverRow(uZr) = true;
            coverColumn(stz) = false;
            z = rIdx==uZr;
            rIdx(z) = [];
            cIdx(z) = [];
            cR = find(~coverRow);
            % 减去去星所在列的最小值，如果出现0，加入刻记队列，有可能成为标星
            z = abs(dMat(~coverRow,stz) - (minR(~coverRow) + minC(stz))) < eps;
            rIdx = [rIdx(:);cR(z)];
            cIdx = [cIdx(:);stz(ones(sum(z),1))];
        end
        if Step == 6
            % *************************************************************************
            % STEP 6: Add the minimum uncovered value to every element of each covered
            %         row, and subtract it from every element of each uncovered column.
            %         Return to Step 4 without altering any stars, primes, or covered lines.
            %**************************************************************************
            % 找到未未覆盖的矩阵A,最小值minval，A - minval必然有一个0，[rIdx,cIdx]即为0所在位置
            % to step 4
            [minval, rIdx, cIdx] = outerplus(dMat(~coverRow, ~coverColumn), minR(~coverRow), minC(~coverColumn), eps);
            minC(~coverColumn) = minC(~coverColumn) + minval;
            minR(coverRow) = minR(coverRow) - minval;
        else
            break
        end
    end
    %**************************************************************************
    % STEP 5:
    %  Construct a series of alternating primed and starred zeros as
    %  follows:
    %  Let Z0 represent the uncovered primed zero found in Step 4.
    %  Let Z1 denote the starred zero in the column of Z0 (if any).
    %  Let Z2 denote the primed zero in the row of Z1 (there will always
    %  be one).  Continue until the series terminates at a primed zero
    %  that has no starred zero in its column.  Unstar each starred
    %  zero of the series, star each primed zero of the series, erase
    %  all primes and uncover every line in the matrix.  Return to Step 3.
    %**************************************************************************
    % step 5 是从step 4跳出，[uZr,uZc]已经刻记但未曾标星的跳出来的
    rowZ1 = find(starZ==uZc);        % rowZ1有可能为空？
    starZ(uZr) = uZc;                % [uZr,uZc]刻记改为标星
    
    while rowZ1 > 0
        % 原来已经标星，需要取消原标星，将刻记改为标星
        starZ(rowZ1) = 0;            % 原标星取消，刻记已改为标星
        uZc = primeZ(rowZ1(1));      % 在取消的标星行必有刻记的，将刻记改为标星
        uZr = rowZ1(1);
        rowZ1 = find(starZ==uZc);    % 是否还存在标星重复的行，若有，下一个循环去掉这个标星
        starZ(uZr) = uZc;            % 刻记改为标星
    end
end

% Cost of assignment
rowIdx = find(validRow);
colIdx = find(validCol);
starZ = starZ(1:nRows);
vIdx = starZ <= nCols;
assignment(rowIdx(vIdx)) = colIdx(starZ(vIdx));
pass = assignment(assignment>0);
pass(~diag(validMat(assignment>0, pass))) = 0;
assignment(assignment>0) = pass;
cost = trace(costMat(assignment>0, assignment(assignment>0)));
one = ones(1, 1, class(costMat));
for i = 1:numel(assignment)
    if assignment(i) > 0
        M(i, assignment(i)) = one;
    end
end
% cost = sum(M .* costMat);

end

function [minval,rIdx,cIdx] = outerplus(M, x, y, eps)
ny = size(M,2);
minval = single(1e9);
for c = 1:ny
    M(:,c) = M(:,c)-(x+y(c));
    minval = min(minval,min(M(:,c)));
end
[rIdx,cIdx] = find(abs(M - minval) < eps);

end
