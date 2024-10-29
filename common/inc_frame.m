function [cnt, lst] = inc_frame(cnt, lst)    %#codegen
% Description: 有效帧 - 更新
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
coder.inline('always')
assert(isscalar(cnt))
assert(isscalar(lst))
assert(isa(cnt, 'int32'))
assert(isa(lst, 'int32'))
cnt = cnt + int32(1);
lst = max(int32(0), lst - int32(1));

end

