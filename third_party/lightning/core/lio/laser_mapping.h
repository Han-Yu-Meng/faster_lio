#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"
#include "pointcloud_preprocess.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

namespace ui {
class PangolinWindow;
}

struct FasterLIOParams;

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        bool enable_icp_part_ = true;    // 是否添加ICP部分
        double plane_icp_weight_ = 1.0;  // 点面ICP部分的权重
        double icp_weight_ = 100;        // ICP部分的权重

        int min_pts = 300;  // 配准所需的点数

        /// 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;

        bool proj_kfs_ = false;
        int max_proj_kfs_ = 5;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    /// init without ros
    bool Init(const std::string &config_yaml);
    /// Inject parameters from FINS parameter server (no yaml file needed).
    bool ApplyParams(const FasterLIOParams &p);

    bool Run();

    // callbacks of lidar and imu
    /// 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    /// 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    /// 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    /// 保存前端的地图
    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    /// 获取关键帧
    Keyframe::Ptr GetKeyframe() const { return last_kf_; }

    /// 获取激光的状态
    NavState GetState() const { return state_point_; }

    /// 获取IMU状态
    NavState GetIMUState() const {
        if (p_imu_->IsIMUInited()) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    /// ---- fixed-map localization extension ----
    /// Load a prior PCD into ivox as an immutable map. map_to_internal transforms
    /// points from map frame into the internal (IMU world) frame, normally
    /// B^-1 * T_map^odom^-1 where B is base->lidar.
    bool LoadPriorMap(const std::string &pcd_path, double voxel_size,
                      const SE3 &map_to_internal);
    /// Disable incremental mapping so ivox stays immutable after loading.
    void EnableLocalizationMode();
    /// IMU-rate snapshot source: must be the lidar-frame-end state (kf_), NOT
    /// kf_imu_ which is already extrapolated to the latest IMU. Using kf_imu_
    /// here double-integrates IMU in the high-rate callback -> drift/jitter.
    ESKF SnapshotIMUFilter() const { return kf_; }
    ESKF::ProcessNoiseType GetQ() const { return p_imu_->Q_; }
    double GetFinalRes() const { return kf_.GetFinalRes(); }
    double GetLidarResidualMean() const { return kf_.GetLidarResidualMean(); }
    int GetIterations() const { return kf_.GetIterations(); }
    int GetEffectFeatNum() const { return effect_feat_surf_; }
    bool IsFixedMapActive() const { return fixed_map_active_; }

    CloudPtr GetScanUndist() const { return scan_undistort_; }

    /// [slam_node] 用实际 lidar->base(车体) 外参覆盖 yaml 里的近似值（Startup 时由 /tf_static 提供）
    void SetLidarExtrinsic(const Mat3d& R_lidar_to_body, const Vec3d& t_lidar_to_body) {
        offset_R_lidar_fixed_ = R_lidar_to_body;
        offset_t_lidar_fixed_ = t_lidar_to_body;
        if (p_imu_) {
            p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
        }
    }

    /// [slam_node] LIO 自身的 lidar->IMU 外参（yaml），用于把 IMU 位姿换算成 base_link 位姿
    Mat3d GetLidarImuR() const { return offset_R_lidar_fixed_; }
    Vec3d GetLidarImuT() const { return offset_t_lidar_fixed_; }

    /// [slam_node] 仅用于【发布】点云的 lidar->车体 外参（不影响 LIO 内部建图/去畸变）
    void SetPublishExtrinsic(const Mat3d& R_lidar_to_body, const Vec3d& t_lidar_to_body) {
        pub_R_lidar_body_ = R_lidar_to_body;
        pub_t_lidar_body_ = t_lidar_to_body;
        use_pub_extrinsic_ = true;
    }

    /// [slam_node] 用给定刚体变换把去畸变扫描转到目标系（cloud 与 odom->base 共用同一变换，避免重影）
    CloudPtr GetCloudWithTransform(const Mat3d &R, const Vec3d &t) const {
        auto out = std::make_shared<PointCloudType>();
        out->resize(scan_undistort_->size());
        for (size_t i = 0; i < scan_undistort_->size(); ++i) {
            const auto &pi = scan_undistort_->points[i];
            Vec3d p = R * pi.getVector3fMap().cast<double>() + t;
            PointType po;
            po.x = p(0); po.y = p(1); po.z = p(2);
            po.intensity = pi.intensity; po.time = pi.time;
            out->points[i] = po;
        }
        out->width = out->points.size(); out->height = 1; out->is_dense = false;
        return out;
    }

    /// [slam_node] 当前扫描转到世界系（可叠加发布专用外参 base_link->main_lidar），与内部建图同口径
    CloudPtr GetWorldCloud() const {
        auto out = std::make_shared<PointCloudType>();
        out->resize(scan_undistort_->size());
        const Mat3d &Rd = use_pub_extrinsic_ ? pub_R_lidar_body_ : offset_R_lidar_fixed_;
        const Vec3d &td = use_pub_extrinsic_ ? pub_t_lidar_body_ : offset_t_lidar_fixed_;
        for (size_t i = 0; i < scan_undistort_->size(); ++i) {
            const auto &pi = scan_undistort_->points[i];
            Vec3d p_global(state_point_.rot_ *
                               (Rd * pi.getVector3fMap().cast<double>() + td) +
                           state_point_.pos_);
            PointType po;
            po.x = p_global(0); po.y = p_global(1); po.z = p_global(2);
            po.intensity = pi.intensity; po.time = pi.time;
            out->points[i] = po;
        }
        out->width = out->points.size(); out->height = 1; out->is_dense = false;
        return out;
    }
    CloudPtr GetProjCloud();

    /// 获取最新的点云
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() { return all_keyframes_; }

    /**
     * 计算全局地图
     * @param use_lio_pose
     * @return
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

   private:
    // sync lidar with imu
    bool SyncPackages();

    void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot_ *
                           (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    bool LoadParamsFromYAML(const std::string &yaml);

    /// 创建关键帧
    void MakeKF();

    /// 将附近的关键帧投影至cloud中
    void ProjectKFs(CloudPtr cloud, int size_limit = 1000);

   private:
    Options options_;

    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// fixed-map localization
    bool fixed_map_active_ = false;

    /// local map related
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();
    // 发布专用外参（base_link->main_lidar），仅 GetWorldCloud 使用，不影响 LIO
    Mat3d pub_R_lidar_body_ = Mat3d::Identity();
    Vec3d pub_t_lidar_body_ = Vec3d::Zero();
    bool use_pub_extrinsic_ = false;
    std::string map_file_path_;

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    /// 点面相关
    std::vector<PointVector> nearest_points_;  // nearest points of current scan
    std::vector<Vec4f> corr_pts_;              // inlier pts
    std::vector<Vec4f> corr_norm_;             // inlier plane norms
    std::vector<float> residuals_;             // point-to-plane residuals
    std::vector<char> point_selected_surf_;    // selected points
    std::vector<Vec4f> plane_coef_;            // plane coeffs

    /// 点到点相关
    std::vector<char> point_selected_icp_;  // 点到点的selected points

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    /// options
    bool keep_first_imu_estimation_ = false;  // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_surf_ = 0, frame_num_ = 0, effect_feat_icp_ = 0;

    double last_lidar_time_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    bool use_aa_ = false;  // use anderson acceleration?

    std::list<Keyframe::Ptr> proj_kfs_;  // 投影到当前帧的关键帧

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H