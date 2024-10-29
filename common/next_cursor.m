function cursor_nxt = next_cursor(cursor, len)    %#codegen
% Description: 下一帧
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
coder.inline('always')
assert(isscalar(cursor))
assert(isscalar(len))
assert(isa(cursor, 'uint8'))
assert(isa(len, 'uint8'))
if cursor >= len
    cursor_nxt = uint8(1);
else
    cursor_nxt = cursor+uint8(1);
    assert(cursor_nxt <= len)
end

end