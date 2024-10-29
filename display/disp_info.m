function disp_info(info, cls, spec)    %#codegen
% 显示信息

coder.inline('always')

global g_disp_info_cls;
global g_disp_info_spec;

if nargin < 3
    spec = info_spec.general;
end

if nargin < 2
    cls = info_cls.general;
end

% 如果对应的显示类别有效，且显示级别高于规定等级，则显示信息
if (bitand(g_disp_info_cls, uint16(cls), 'uint16')) && (g_disp_info_spec <= uint8(spec))
    fprintf('%s\n', info)
end

end

