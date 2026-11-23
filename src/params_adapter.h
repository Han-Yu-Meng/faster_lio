#ifndef FASTER_LIO_PARAMS_ADAPTER_H
#define FASTER_LIO_PARAMS_ADAPTER_H

#include <string>
#include <vector>

namespace lightning {

// Mirrors the `fasterlio:` + `roi:` keys read by lightning::LaserMapping::
// LoadParamsFromYAML. Defaults are taken from lc_backend/config/config_airy_small.yaml.
struct FasterLIOParams {
    int max_iteration = 4;
    float esti_plane_threshold = 0.15f;
    float filter_size_scan = 0.2f;
    float filter_size_map = 0.25f;
    bool keep_first_imu_estimation = false;
    float gyr_cov = 0.1f;
    float acc_cov = 0.1f;
    float b_gyr_cov = 0.0001f;
    float b_acc_cov = 0.0001f;
    double blind = 0.3;
    double time_scale = 1.0;
    int lidar_type = 4;  // RoboSense
    int scan_line = 80;
    int point_filter_num = 3;
    std::vector<double> extrinsic_T = {0.004250, 0.004180, -0.004460};
    std::vector<double> extrinsic_R = {
        0.0033883, -0.99997669, 0.00592802,
        -0.99999315, -0.00339705, -0.00146638,
        0.00148648, -0.00592301, -0.99998135};
    float ivox_grid_resolution = 0.25f;
    int ivox_nearby_type = 18;
    bool use_aa = false;
    int skip_lidar_num = 0;
    float roi_height_max = 15.0f;
    float roi_height_min = -15.0f;
    double kf_dis_th = 0.5;
    double kf_angle_th = 10.0;  // degrees
    bool enable_icp_part = false;
    int min_pts = 500;
    float plane_icp_weight = 300.0f;
    bool imu_filter = false;
    bool proj_kfs = false;
};

}  // namespace lightning

#endif
