function mbdInitizlizeObstacle(FPS, min_conf, x_coverage, y_coverage, ref_widths, thres_stable, thres_unstable, thres_lost)    %#codegen
%MBDINITIZLIZEOBSTACLE 此处显示有关此函数的摘要
%   此处显示详细说明
% cfg = struct('FPS', 30, 'min_conf', 0.1, 'x_coverage', [0, 150], 'y_coverage',
% [-16, 16], 'ref_widths', [1.80, 2.45, 2.55], 'max_lst', int32(6), 'min_cnt', int32(16));
%             'thres_stable', int32(10),...           % stable状态阈值。CNT大于此值进入stable状态
%             'thres_unstable', int32(3),...          % unstable状态阈值。LST大于此值进入unstable状态
%             'thres_lost', int32(6)...              % lost状态阈值。LST大于此值则目标丢失

assert(isa(FPS, 'double'))
assert(isa(min_conf, 'double'))
assert(isa(x_coverage, 'double'))
assert(isa(y_coverage, 'double'))
assert(isa(ref_widths, 'double'))
assert(isa(thres_stable, 'int32'))
assert(isa(thres_unstable, 'int32'))
assert(isa(thres_lost, 'int32'))

assert(isscalar(FPS))
assert(isscalar(min_conf))
assert(numel(x_coverage) == 2)
% assert(x_coverage(2) > x_coverage(1) && x_coverage(1) >= 0 && x_coverage(2) <= 200)
% assert(numel(y_coverage) == 2)
% assert(y_coverage(2) > y_coverage(1) && y_coverage(2) <= 50 && y_coverage(1) >= -50)
assert(numel(ref_widths) <= 13)
assert(ref_widths(3) >= 1.2 && ref_widths(3) <= 3.5)
assert(ref_widths(2) >= 1.2 && ref_widths(2) <= 3.5)
assert(ref_widths(1) >= 1.2 && ref_widths(1) <= 3.5)

assert(isscalar(thres_stable))
assert(isscalar(thres_unstable))
assert(isscalar(thres_lost))

assert(obstacle_const.len_1st <= thres_stable)
assert(thres_unstable < thres_stable)
assert(thres_lost < thres_stable)

global g_cfg_auto g_cfg_VRU;

% g_cfg_auto.FPS = check_range(FPS, [5, 30]);
g_cfg_auto.deltaT = check_range(1/FPS, [1/5, 1/30]);
g_cfg_auto.min_conf = check_range(min_conf, [0.01, 1]);
g_cfg_auto.x_coverage = x_coverage;
g_cfg_auto.y_coverage = y_coverage;
g_cfg_auto.ref_widths = ref_widths;

g_cfg_auto.thres_stable = int32(check_range(double(thres_stable), [1, 100]));
g_cfg_auto.thres_unstable = int32(check_range(double(thres_unstable), [1, 100]));
g_cfg_auto.thres_lost = int32(check_range(double(thres_lost), [1, 100]));

g_cfg_VRU.FPS = check_range(FPS, [5, 30]);
g_cfg_VRU.deltaT = check_range(1/FPS, [1/5, 1/30]);
g_cfg_VRU.thres_stable = int32(10);
g_cfg_VRU.thres_unstable = int32(5);
g_cfg_VRU.thres_lost = int32(8);

end

