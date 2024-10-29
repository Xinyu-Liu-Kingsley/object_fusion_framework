function mbdCfgDispInfo(cls, spec)    %#codegen
% Description: 配置显示信息
% cls  - 可显示信息类别，如：info_cls.general | info_cls.lane | info_cls.obstalce
% spec - 可显示信息级别，如：info_spec.warning表示显示级别大于等于warning的信息都会被显示
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 

% bitor(info_cls.general, bitor(info_cls.lane, info_cls.obstacle, 'uint16'))

global g_disp_info_cls;
global g_disp_info_spec;

g_disp_info_cls = uint16(cls);

assert(spec >= info_spec.general && spec <= info_spec.error)
g_disp_info_spec = uint8(spec);

end

