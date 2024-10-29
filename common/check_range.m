function out = check_range(value, range)    %#codegen 
% Description: 检查数据是否处于有效范围之内
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
global s_check_arg_range;

coder.inline('always')

if s_check_arg_range
    assert(isa(value, 'double'))
    assert(isa(range, 'double'))
    
    assert(isscalar(value))
    assert(numel(range) == 2)
    
    assert(range(2) > range(1))
    
    out = max(range(1), min(value, range(2)));
    
    if out ~= value
        disp_info(sprintf('????? out of range : %f [%f - %f]\n', value, range(1), range(2)), info_cls.general, info_spec.warning)
    end
else
    out = value;
end

end

