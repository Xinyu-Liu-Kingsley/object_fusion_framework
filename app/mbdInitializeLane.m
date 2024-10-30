function mbdInitializeLane(FPS, min_conf, min_pts, x_coverage, y_coverage,...
    w_coverage, min_x_len, max_x_near, max_heading_deg, min_radius,...
    def_wid_half, max_lst, min_cnt, max_delta_u, max_delta_v, epsilon, min_clu)    %#codegen
%MBDINITIALIZELANE 此处显示有关此函数的摘要
%   此处显示详细说明

assert(isa(FPS, 'double'))
assert(isa(min_conf, 'double'))
assert(isa(min_pts, 'uint16'))
assert(isa(x_coverage, 'double'))
assert(isa(y_coverage, 'double'))
assert(isa(w_coverage, 'double'))
assert(isa(min_x_len, 'double'))
assert(isa(max_x_near, 'double'))
assert(isa(max_heading_deg, 'double'))
assert(isa(min_radius, 'double'))
assert(isa(def_wid_half, 'double'))
assert(isa(max_lst, 'int32'))
assert(isa(min_cnt, 'int32'))
assert(isa(max_delta_u, 'double'))
assert(isa(max_delta_v, 'double'))

assert(isscalar(FPS))
assert(isscalar(min_conf))
assert(isscalar(min_pts))
assert(numel(x_coverage) == 2)
assert(x_coverage(2) > x_coverage(1) && x_coverage(1) >= 0 && x_coverage(2) <= 200)
assert(numel(y_coverage) == 2)
assert(y_coverage(2) > y_coverage(1) && y_coverage(2) <= 50 && y_coverage(1) >= -50)
assert(numel(w_coverage) == 2)
assert(w_coverage(2) > w_coverage(1) && w_coverage(1) >= 2.6 && w_coverage(2) <= 4.9)
assert(isscalar(min_x_len))
assert(min_x_len > 2 && min_x_len < x_coverage(2))
assert(isscalar(max_x_near))
assert(max_x_near >= 0 && max_x_near < x_coverage(2))
assert(isscalar(max_heading_deg))
assert(max_heading_deg >= 10 && max_heading_deg < 90)
assert(isscalar(min_radius))
assert(min_radius >= 50 && min_radius < 100000)
assert(isscalar(def_wid_half))
assert(def_wid_half >= w_coverage(1) / 2 && def_wid_half <= w_coverage(2) / 2)
assert(isscalar(max_lst))
assert(max_lst > 0)
assert(isscalar(min_cnt))
assert(min_cnt > 0)
assert(isscalar(max_delta_u))
assert(max_delta_u > 0)
assert(isscalar(max_delta_v))
assert(max_delta_v > 0)
assert(isscalar(epsilon))
assert(epsilon > 0 && epsilon < 2)
assert(isscalar(min_clu))
assert(min_clu > 0 && min_clu < lane_const.max_pts)

global g_cfg_lane;

g_cfg_lane.FPS = check_range(FPS, [5, 30]);
g_cfg_lane.deltaT = check_range(1/FPS, [1/5, 1/30]);
g_cfg_lane.min_conf = check_range(min_conf, [0.01, 1]);
g_cfg_lane.min_pts = uint16(check_range(double(min_pts), [3, double(lane_const.max_pts)]));
g_cfg_lane.x_coverage = x_coverage;
g_cfg_lane.y_coverage = y_coverage;
g_cfg_lane.w_coverage = w_coverage;
g_cfg_lane.min_x_len = check_range(min_x_len, [5, 100]);
g_cfg_lane.max_x_near = max_x_near;
g_cfg_lane.max_heading = max_heading_deg;
g_cfg_lane.min_radius = min_radius;
g_cfg_lane.def_wid_half = check_range(def_wid_half, [1.25, 3]);
g_cfg_lane.max_lst = int32(check_range(double(max_lst), [1, 100]));
g_cfg_lane.min_cnt = int32(check_range(double(min_cnt), [1, double(lane_const.track_depth * 10)]));
g_cfg_lane.max_delta_u = check_range(max_delta_u, [10, 100]);
g_cfg_lane.max_delta_v = check_range(max_delta_v, [10, 100]);
g_cfg_lane.epsilon = epsilon;
g_cfg_lane.min_clu = min_clu;

end

