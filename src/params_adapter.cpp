// FINS parameter injection for the copied lightning::LaserMapping core.
// Mirrors LoadParamsFromYAML (laser_mapping.cc:35-131) without a yaml file.

#include "params_adapter.h"

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "core/lio/laser_mapping.h"

namespace lightning {

bool LaserMapping::ApplyParams(const FasterLIOParams &p) {
    fasterlio::NUM_MAX_ITERATIONS = p.max_iteration;
    fasterlio::ESTI_PLANE_THRESHOLD = p.esti_plane_threshold;

    filter_size_map_min_ = p.filter_size_map;
    keep_first_imu_estimation_ = p.keep_first_imu_estimation;

    preprocess_->Blind() = p.blind;
    preprocess_->TimeScale() = p.time_scale;
    preprocess_->NumScans() = p.scan_line;
    preprocess_->PointFilterNum() = p.point_filter_num;

    switch (p.lidar_type) {
        case 1: preprocess_->SetLidarType(LidarType::AVIA); break;
        case 2: preprocess_->SetLidarType(LidarType::VELO32); break;
        case 3: preprocess_->SetLidarType(LidarType::OUST64); break;
        case 4: preprocess_->SetLidarType(LidarType::ROBOSENSE); break;
        default:
            LOG(WARNING) << "[FasterLIO] unknown lidar_type " << p.lidar_type;
            return false;
    }
    LOG(INFO) << "[FasterLIO] lidar_type " << p.lidar_type;

    switch (p.ivox_nearby_type) {
        case 0:  ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER; break;
        case 6:  ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6; break;
        case 18: ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18; break;
        case 26: ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26; break;
        default:
            LOG(WARNING) << "[FasterLIO] unknown ivox_nearby_type, use NEARBY18";
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
    ivox_options_.resolution_ = p.ivox_grid_resolution;

    preprocess_->SetHeightROI(p.roi_height_max, p.roi_height_min);

    options_.kf_dis_th_ = p.kf_dis_th;
    options_.kf_angle_th_ = p.kf_angle_th * M_PI / 180.0;
    options_.enable_icp_part_ = p.enable_icp_part;
    options_.min_pts = p.min_pts;
    options_.plane_icp_weight_ = p.plane_icp_weight;
    options_.proj_kfs_ = p.proj_kfs;

    p_imu_->SetUseIMUFilter(p.imu_filter);

    use_aa_ = p.use_aa;
    skip_lidar_num_ = p.skip_lidar_num;
    enable_skip_lidar_ = p.skip_lidar_num > 0;

    extrinT_ = p.extrinsic_T;
    extrinR_ = p.extrinsic_R;
    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR_);

    voxel_scan_.setLeafSize(p.filter_size_scan, p.filter_size_scan, p.filter_size_scan);
    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    p_imu_->SetGyrCov(Vec3d(p.gyr_cov, p.gyr_cov, p.gyr_cov));
    p_imu_->SetAccCov(Vec3d(p.acc_cov, p.acc_cov, p.acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(p.b_gyr_cov, p.b_gyr_cov, p.b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(p.b_acc_cov, p.b_acc_cov, p.b_acc_cov));

    // Same post-parameter initialization as LaserMapping::Init() (lines 21-31).
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);
    return true;
}

}  // namespace lightning
