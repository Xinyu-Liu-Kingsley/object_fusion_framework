function [cnt, lst] = dec_frame(cnt, lst)    %#codegen
% Description: 丢帧-更新
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
coder.inline('always')
assert(isscalar(cnt))
assert(isscalar(lst))
assert(isa(cnt, 'int32'))
assert(isa(lst, 'int32'))
cnt = max(int32(0), cnt - int32(1));
lst = lst + int32(1);

end
