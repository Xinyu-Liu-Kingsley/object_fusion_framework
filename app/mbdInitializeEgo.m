
function mbdInitializeEgo(axle_nums, w, l, h, f_overhang, f_wheelbase, axle12_base, axle23_base, axle34_base, total_mass, curb_mass, axle1_weight, axle2_weight, axle3_weight, axle4_weight)    %#codegen
% 初始化车辆相关参数
% axle_nums	-   车轴数
% w         -   车宽
% l         -   车长
% h         -   车高
% f_overhang    -   前悬
% f_wheelbase   -   前轴轮距

assert(isa(axle_nums, 'uint8'))
assert(isscalar(axle_nums))
assert(isa(w, 'double'))
assert(isscalar(w))
assert(isa(l, 'double'))
assert(isscalar(l))
assert(isa(h, 'double'))
assert(isscalar(h))
assert(isa(f_overhang, 'double'))
assert(isscalar(f_overhang))
assert(isa(f_wheelbase, 'double'))
assert(isscalar(f_wheelbase))
assert(isa(axle12_base, 'double'))
assert(isscalar(axle12_base))
assert(isa(axle23_base, 'double'))
assert(isscalar(axle23_base))
assert(isa(axle34_base, 'double'))
assert(isscalar(axle34_base))
assert(isa(total_mass, 'double'))
assert(isscalar(total_mass))
assert(isa(curb_mass, 'double'))
assert(isscalar(curb_mass))
assert(isa(axle1_weight, 'double'))
assert(isscalar(axle1_weight))
assert(isa(axle2_weight, 'double'))
assert(isscalar(axle2_weight))
assert(isa(axle3_weight, 'double'))
assert(isscalar(axle3_weight))
assert(isa(axle4_weight, 'double'))
assert(isscalar(axle4_weight))

assert(axle_nums >= ego_const.min_axles && axle_nums <= ego_const.max_axles)

global g_ego_params;

g_ego_params.axle_nums = axle_nums;
g_ego_params.width = check_range(w, ego_const.rng_width);
g_ego_params.length = check_range(l, ego_const.rng_length);
g_ego_params.height = check_range(h, ego_const.rng_height);
g_ego_params.front_overhang = check_range(f_overhang, ego_const.rng_foverhang);
g_ego_params.front_wheelbase = check_range(f_wheelbase, ego_const.rng_wheelbase);
g_ego_params.axle12_base = check_range(axle12_base, ego_const.rng_axlebase);
if axle_nums > uint8(2)
    g_ego_params.axle23_base = check_range(axle23_base, ego_const.rng_axlebase);
end
if axle_nums > uint8(3)
    g_ego_params.axle34_base = check_range(axle34_base, ego_const.rng_axlebase);
end
g_ego_params.total_mass = check_range(total_mass, ego_const.rng_mass);
g_ego_params.curb_mass = check_range(curb_mass, ego_const.rng_mass);
g_ego_params.axle1_weight = check_range(axle1_weight, ego_const.rng_axleweight);
g_ego_params.axle2_weight = check_range(axle2_weight, ego_const.rng_axleweight);
if axle_nums > uint8(2)
    g_ego_params.axle3_weight = check_range(axle3_weight, ego_const.rng_axleweight);
end
if axle_nums > uint8(3)
    g_ego_params.axle4_weight = check_range(axle4_weight, ego_const.rng_axleweight);
end

disp_ego_params();

end

