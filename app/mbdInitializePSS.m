function [u, v, w, h] = mbdInitializePSS(fx, fy, cx, cy, img_size, pitch_band, yaw_band)    %#codegen
% 售后服务（post sale serves）功能之标定

assert(isa(pitch_band, 'double'))
assert(isa(yaw_band, 'double'))
assert(numel(pitch_band) == 2)
assert(numel(yaw_band) == 2)
assert(pitch_band(1) < pitch_band(2))
assert(yaw_band(1) < yaw_band(2))
assert(isscalar(fx))
assert(isscalar(fy))
assert(isscalar(cx))
assert(isscalar(cy))

global g_pss_calibrate;
g_pss_calibrate = cam_const.pss_calibrate;
g_pss_calibrate.fx = fx;
g_pss_calibrate.fy = fy;
g_pss_calibrate.cx = cx;
g_pss_calibrate.cy = cy;
g_pss_calibrate.pitch_rng = pitch_band;
g_pss_calibrate.yaw_rng = yaw_band;
g_pss_calibrate.img_size = img_size;
g_pss_calibrate.calis = zeros(cam_const.cali_num_frm, 2);
g_pss_calibrate.cnt_val = uint32(0);
g_pss_calibrate.cnt_all = uint32(0);

v1 = tan(deg2rad(pitch_band(1)))*g_pss_calibrate.fy+g_pss_calibrate.cy;
v2 = tan(deg2rad(pitch_band(2)))*g_pss_calibrate.fy+g_pss_calibrate.cy;

u1 = g_pss_calibrate.cx-tan(deg2rad(yaw_band(1)))*g_pss_calibrate.fx;
u2 = g_pss_calibrate.cx-tan(deg2rad(yaw_band(2)))*g_pss_calibrate.fx;

u = u2;
v = v1;
w = u1-u2;
h = v2-v1;

end
