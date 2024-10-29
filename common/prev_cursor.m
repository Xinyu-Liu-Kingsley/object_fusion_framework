function cursor_pre = prev_cursor(cursor, len)    %#codegen
% Description: 上一帧
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
coder.inline('always')
assert(isscalar(cursor))
assert(isscalar(len))
assert(isa(cursor, 'uint8'))
assert(isa(len, 'uint8'))
if cursor == 1
    cursor_pre = uint8(len);
else
    cursor_pre = cursor-uint8(1);
    assert(cursor_pre >= 1)
end

end
