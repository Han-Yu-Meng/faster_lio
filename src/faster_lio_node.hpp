#ifndef FASTER_LIO_NODE_HPP
#define FASTER_LIO_NODE_HPP

#include <fins/node.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "core/lio/eskf.hpp"
#include "core/lio/laser_mapping.h"
#include "params_adapter.h"

class FasterLIO : public fins::Node {
 public:
  void define() override {
    set_name("FasterLIO");
    set_description("FasterLIO fixed-map localization node (FAST_LIO-compatible ports)");
    set_category("SLAM");

    register_input<sensor_msgs::msg::Imu>("imu", &FasterLIO::on_imu);
    register_input<livox_ros_driver2::msg::CustomMsg>("lidar", &FasterLIO::on_livox);
    register_input<sensor_msgs::msg::PointCloud2>("lidar_standard", &FasterLIO::on_lidar);
    register_input<geometry_msgs::msg::TransformStamped>("$T_{base}^{lidar}$", &FasterLIO::on_transform);
    register_input<geometry_msgs::msg::TransformStamped>("initial_alignment", &FasterLIO::on_initial_alignment);

    register_output<pcl::PointCloud<pcl::PointXYZI>::Ptr>("cloud");
    register_output<pcl::PointCloud<pcl::PointXYZI>::Ptr>("cloud_body");
    register_output<geometry_msgs::msg::TransformStamped>("$T_{map}^{odom}$");
    register_output<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose");
    register_output<std_msgs::msg::String>("localization_status");
    register_output<std_msgs::msg::Float64>("localization_confidence");
    register_output<nav_msgs::msg::Path>("path");
    register_output<nav_msgs::msg::Odometry>("odometry");
    register_output<geometry_msgs::msg::TransformStamped>("$T_{odom}^{base}$");
  }

  void initialize() override {
    auto &ps = fins::param_server();
    fixed_map_enabled_ = ps.get("FastLIO.localization.enabled", false)
                             .with_description("Track immutable prior map");
    map_path_ = ps.get("FastLIO.localization.map_path", std::string(""))
                    .with_description("prior PCD path");
    map_leaf_size_ = ps.get("FastLIO.localization.map_leaf_size", 0.15)
                         .greater_than(0.0);
    min_effective_points_ = ps.get("FastLIO.localization.min_effective_points", 30)
                                .greater_than(5);
    max_mean_residual_ = ps.get("FastLIO.localization.max_mean_residual", 1.11)
                             .greater_than(0.0);
    failures_before_lost_ = ps.get("FastLIO.localization.failures_before_lost", 5)
                                .greater_than(0);
    good_frames_to_lock_ = ps.get("FastLIO.localization.good_frames_to_lock", 3)
                               .greater_than(0);
    max_alignment_age_ = ps.get("FastLIO.localization.max_alignment_age", 60.0)
                             .greater_than(0.0);
    imu_rate_output_ = ps.get("FastLIO.localization.imu_rate_output", true)
                           .with_description("publish at IMU rate when LOCKED");
    initial_frame_ = ps.get("FastLIO.common.initial_frame", std::string("odom"));
    base_link_frame_ = ps.get("FastLIO.common.base_link_frame", std::string("base_link"));
    imu_window_size_ = ps.get("FastLIO.common.imu_window_size", 10).greater_than(0);

    // FasterLIO core parameters (vectors deliberately use code defaults: the FINS
    // parameter server does not load yaml sequences, same as FAST-LIO).
    lightning::FasterLIOParams fp;
    fins::ParamLoader fl("FastLIO.fasterlio");
    fp.max_iteration = fl.get("max_iteration", 4).greater_than(0);
    fp.esti_plane_threshold = fl.get("esti_plane_threshold", 0.15f).greater_than(0.0f);
    fp.filter_size_scan = fl.get("filter_size_scan", 0.2f).greater_than(0.0f);
    fp.filter_size_map = fl.get("filter_size_map", 0.25f).greater_than(0.0f);
    fp.gyr_cov = fl.get("gyr_cov", 0.1f).greater_than(0.0f);
    fp.acc_cov = fl.get("acc_cov", 0.1f).greater_than(0.0f);
    fp.b_gyr_cov = fl.get("b_gyr_cov", 0.0001f).greater_than(0.0f);
    fp.b_acc_cov = fl.get("b_acc_cov", 0.0001f).greater_than(0.0f);
    fp.blind = fl.get("blind", 0.3).greater_than(0.0);
    fp.point_filter_num = fl.get("point_filter_num", 3).greater_than(0);
    fp.lidar_type = fl.get("lidar_type", 4);
    fp.ivox_grid_resolution = fl.get("ivox_grid_resolution", 0.25f).greater_than(0.0f);
    fp.ivox_nearby_type = fl.get("ivox_nearby_type", 18);
    fp.kf_dis_th = fl.get("kf_dis_th", 0.5).greater_than(0.0);
    fp.kf_angle_th = fl.get("kf_angle_th", 10.0).greater_than(0.0);
    fp.enable_icp_part = fl.get("enable_icp_part", false);
    fp.min_pts = fl.get("min_pts", 500).greater_than(0);
    fp.plane_icp_weight = fl.get("plane_icp_weight", 300.0f).greater_than(0.0f);
    fp.imu_filter = fl.get("imu_filter", false);
    fp.use_aa = fl.get("use_aa", false);
    fp.skip_lidar_num = fl.get("skip_lidar_num", 0);

    mapper_ = std::make_shared<lightning::LaserMapping>();
    if (!mapper_->ApplyParams(fp)) {
      logger->error("FasterLIO: failed to apply core parameters");
      return;
    }
    if (fixed_map_enabled_) {
      if (map_path_.empty()) {
        logger->error("FasterLIO: fixed-map enabled but map_path is empty");
        return;
      }
      mapper_->EnableLocalizationMode();
    }

    is_running_ = true;
    backend_thread_ = std::thread(&FasterLIO::backend_loop, this);
    logger->info("FasterLIO initialized (fixed_map={}, map={})", fixed_map_enabled_, map_path_);
  }

  void pause() override {
    is_running_ = false;
    trigger_cv_.notify_all();
    if (backend_thread_.joinable()) backend_thread_.join();
  }

  void reset() override { pause(); }

  void on_imu(const fins::Msg<sensor_msgs::msg::Imu> &msg) {
    if (!mapper_) return;
    // 与 lightning slam_node 一致：倾斜安装的 IMU 必须等 TF 外参注入后才能
    // 预旋到水平车体系。TF 未到前缓存原始消息，避免用倾斜数据初始化 LIO。
    {
      std::lock_guard<std::mutex> lock(extrinsic_mutex_);
      if (!refactor_enabled_ && fixed_map_enabled_) {
        if (pending_imu_.size() < 4000) pending_imu_.push_back(*msg.ptr());
        return;
      }
    }
    feed_imu(*msg.ptr(), msg.acq_time);
  }

  void feed_imu(const sensor_msgs::msg::Imu &msg, const fins::AcqTime &acq_time) {
    auto imu = std::make_shared<lightning::IMU>();
    imu->timestamp = rclcpp::Time(msg.header.stamp).seconds();
    lightning::Vec3d gyr(msg.angular_velocity.x, msg.angular_velocity.y,
                         msg.angular_velocity.z);
    lightning::Vec3d acc(msg.linear_acceleration.x, msg.linear_acceleration.y,
                         msg.linear_acceleration.z);
    if (acc.norm() < 2.0) acc *= 9.81;  // Airy IMU is in g units
    {
      std::lock_guard<std::mutex> lock(extrinsic_mutex_);
      if (refactor_enabled_) {
        acc = R_pre_ * acc;
        gyr = R_pre_ * gyr;
      }
    }
    imu->angular_velocity = gyr;
    imu->linear_acceleration = acc;
    mapper_->ProcessIMU(imu);

    // IMU-rate odometry prediction on a private filter copy.
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (!imu_rate_live_) return;
      double dt = (last_imu_time_ < 0.0) ? 0.005 : (imu->timestamp - last_imu_time_);
      last_imu_time_ = imu->timestamp;
      if (dt <= 0.0 || dt > 0.5) return;
      kf_copy_.Predict(dt, Q_, imu->angular_velocity, imu->linear_acceleration);
      auto state = kf_copy_.GetX();
      // 角速度：预旋后的 gyro 减陀螺零偏，滑动窗口平滑（FAST_LIO 同款）。
      lightning::Vec3d omega_body = imu->angular_velocity - state.bg_;
      if (imu_window_size_ > 0) {
        ang_vel_window_.push_back(omega_body);
        ang_vel_sum_ += omega_body;
        if (ang_vel_window_.size() > static_cast<size_t>(imu_window_size_)) {
          ang_vel_sum_ -= ang_vel_window_.front();
          ang_vel_window_.pop_front();
        }
        omega_body = ang_vel_sum_ / static_cast<double>(ang_vel_window_.size());
      }
      publish_pose(state, acq_time, omega_body);
    }
  }

  void on_lidar(const fins::Msg<sensor_msgs::msg::PointCloud2> &msg) {
    if (!mapper_) return;
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg.ptr());
    mapper_->ProcessPointCloud2(cloud);
    last_acq_time_ = msg.acq_time;
    notify_backend();
  }

  void on_livox(const fins::Msg<livox_ros_driver2::msg::CustomMsg> &msg) {
    if (!mapper_) return;
    auto cloud = std::make_shared<livox_ros_driver2::msg::CustomMsg>(*msg.ptr());
    mapper_->ProcessPointCloud2(cloud);
    last_acq_time_ = msg.acq_time;
    notify_backend();
  }

  void on_transform(const geometry_msgs::msg::TransformStamped &msg) {
    std::deque<sensor_msgs::msg::Imu> to_feed;
    {
      std::lock_guard<std::mutex> lock(extrinsic_mutex_);
      Eigen::Quaterniond q(msg.transform.rotation.w, msg.transform.rotation.x,
                           msg.transform.rotation.y, msg.transform.rotation.z);
      base_R_lidar_ = q.normalized().toRotationMatrix();
      base_T_lidar_ << msg.transform.translation.x, msg.transform.translation.y,
          msg.transform.translation.z;

      // 照搬 lightning slam_node 的 refactor 路径：用 TF 的 lidar->base 外参覆盖
      // LIO 内核外参，使内部 body 系直接等于 base_link，world 系水平；同时把
      // 倾斜安装的 IMU 数据预旋到车体系（R_pre = R_bl * lidar_to_imu_R^T）。
      // 注意：lidar_to_imu_R 必须在注入前取（注入后 GetLidarImuR 会变成 R_bl）。
      if (mapper_ && !refactor_enabled_) {
        lightning::Mat3d lidar_to_imu_R = mapper_->GetLidarImuR();
        mapper_->SetLidarExtrinsic(base_R_lidar_, base_T_lidar_);
        R_pre_ = base_R_lidar_ * lidar_to_imu_R.transpose();
        refactor_enabled_ = true;
        logger->info("[FasterLIO] TF extrinsic injected (lidar->base), IMU pre-rotation enabled");
        // TF 到达前缓存的 IMU 现在用预旋统一喂入（slam_node 同款）。
        to_feed.swap(pending_imu_);
      }
      extrinsic_received_ = true;
    }
    // feed_imu 内部会锁 extrinsic_mutex_，必须在锁外调用。
    for (const auto &imu_msg : to_feed) feed_imu(imu_msg, fins::AcqTime{});
  }

  void on_initial_alignment(const geometry_msgs::msg::TransformStamped &msg) {
    std::lock_guard<std::mutex> lock(alignment_mutex_);
    pending_alignment_ = msg;
    alignment_pending_ = true;
  }

 private:
  void notify_backend() {
    {
      std::lock_guard<std::mutex> lock(trigger_mtx_);
      has_new_data_ = true;
    }
    trigger_cv_.notify_one();
  }

  void backend_loop() {
    while (is_running_) {
      {
        std::unique_lock<std::mutex> lock(trigger_mtx_);
        trigger_cv_.wait(lock, [this] { return !is_running_ || has_new_data_; });
        has_new_data_ = false;
      }
      if (!is_running_) break;

      apply_pending_alignment();
      if (!mapper_->Run()) continue;

      if (!fixed_map_active_) {
        publish_scan_rate(last_acq_time_);
        continue;
      }

      double res = mapper_->GetLidarResidualMean();
      int feat = mapper_->GetEffectFeatNum();
      // 配准质量只由残差和有效点数决定。状态跳变是真实运动（快速旋转、断流后
      // 帧间隔变大），不是匹配失败；用它当门限会误判并触发无谓的 LOST/重定位。
      bool valid = feat >= min_effective_points_ && res <= max_mean_residual_;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (valid) {
          bad_matches_ = 0;
          tracking_status_ = (++good_matches_ >= good_frames_to_lock_) ? "LOCKED" : "TRACKING";
        } else {
          good_matches_ = 0;
          tracking_status_ = "DEGRADED";
          if (++bad_matches_ >= failures_before_lost_) {
            fixed_map_active_ = false;
            tracking_status_ = "LOST";
          }
          logger->warn("[FixedMap] Rejected update: features={} residual={:.3f}", feat, res);
        }
      }

      // Refresh the IMU-rate snapshot under the same lock used by on_imu.
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        kf_copy_ = mapper_->SnapshotIMUFilter();
        Q_ = mapper_->GetQ();
        last_imu_time_ = -1.0;
        std::lock_guard<std::mutex> slock(state_mutex_);
        imu_rate_live_ = imu_rate_output_ && fixed_map_active_ &&
                         tracking_status_ == "LOCKED" && bad_matches_ == 0;
      }
      publish_scan_rate(last_acq_time_);
    }
  }

  void apply_pending_alignment() {
    geometry_msgs::msg::TransformStamped msg;
    {
      std::lock_guard<std::mutex> lock(alignment_mutex_);
      if (!alignment_pending_) return;
      msg = pending_alignment_;
      alignment_pending_ = false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (fixed_map_active_) return;
    // 与 FAST-LIO apply_pending_alignment 一致：对齐时间戳相对当前采集时刻不能过旧
    double age = fins::to_seconds(last_acq_time_) - rclcpp::Time(msg.header.stamp).seconds();
    if (msg.header.frame_id != "map" || msg.child_frame_id != initial_frame_ ||
        age < -0.1 || age > max_alignment_age_) {
      tracking_status_ = "INITIALIZATION_REJECTED";
      logger->warn("[FixedMap] Rejected alignment: frame/child mismatch or age={:.3f}s", age);
      return;
    }
    Eigen::Quaterniond q(msg.transform.rotation.w, msg.transform.rotation.x,
                         msg.transform.rotation.y, msg.transform.rotation.z);
    lightning::Vec3d t(msg.transform.translation.x, msg.transform.translation.y,
                       msg.transform.translation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !t.allFinite()) return;

    Eigen::Isometry3d alignment = Eigen::Isometry3d::Identity();
    alignment.linear() = q.normalized().toRotationMatrix();
    alignment.translation() = t;

    lightning::Mat3d Rb;
    lightning::Vec3d tb;
    {
      std::lock_guard<std::mutex> elock(extrinsic_mutex_);
      Rb = base_R_lidar_;
      tb = base_T_lidar_;
    }
    Eigen::Isometry3d to_internal = Eigen::Isometry3d::Identity();
    if (refactor_enabled_) {
      // refactor 后 LIO 内部 body 系已被 SetLidarExtrinsic 改成 base_link（水平），
      // odom 系即 base 世界系，而 alignment 是 BBS 给的 map->odom。地图点在 map 系，
      // 变到内部系只需 alignment^-1。若再乘 B^-1（base->main_lidar，pitch≈40°），
      // 会把安装倾角灌进先验地图，配准再把车体拉倾斜 -> 斜插地底。
      to_internal = alignment.inverse();
    } else {
      // 降级路径（未注入 TF 外参，内部仍是 IMU 系）：照搬 FAST_LIO 的
      // p_internal = B^-1 * alignment^-1 * p_map。
      Eigen::Isometry3d B = Eigen::Isometry3d::Identity();
      B.linear() = Rb;
      B.translation() = tb;
      to_internal = (B * alignment).inverse();
    }
    lightning::SE3 map_to_internal(lightning::SO3(to_internal.linear()),
                                   to_internal.translation());

    if (!mapper_->LoadPriorMap(map_path_, map_leaf_size_, map_to_internal)) {
      tracking_status_ = "INITIALIZATION_REJECTED";
      logger->error("[FixedMap] failed to load prior map");
      return;
    }
    map_to_odom_ = alignment;
    fixed_map_active_ = true;
    good_matches_ = bad_matches_ = 0;
    tracking_status_ = "TRACKING";
    logger->info("[FixedMap] prior loaded, entering TRACKING");
  }

  void publish_scan_rate(const fins::AcqTime &ts) {
    if (!mapper_) return;
    // LOCKED + IMU-rate: odometry/TF/localization_pose are published from the
    // IMU callback; only scan-rate status/confidence/cloud are emitted here.
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (!imu_rate_live_) {
        auto st = mapper_->GetState();
        // scan-rate 角速度：相邻 lidar 帧姿态差 / dt（FAST_LIO 同款），世界系→车体系
        lightning::Vec3d omega_body = lightning::Vec3d::Zero();
        if (last_scan_rot_.valid) {
          omega_body = st.rot_.matrix().transpose() *
                       (st.rot_ * last_scan_rot_.rot.inverse()).log() /
                       std::max(1e-6, st.timestamp_ - last_scan_rot_.time);
        }
        last_scan_rot_.rot = st.rot_;
        last_scan_rot_.time = st.timestamp_;
        last_scan_rot_.valid = true;
        publish_pose(st, ts, omega_body);
      }
    }
    publish_clouds(ts);
    if (required("localization_status")) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      std_msgs::msg::String m;
      m.data = tracking_status_;
      send("localization_status", m, ts);
    }
    if (required("localization_confidence")) {
      std_msgs::msg::Float64 m;
      m.data = fixed_map_active_ ? 1.0 : 0.0;
      send("localization_confidence", m, ts);
    }
  }

  // ang_vel_body: 车体(base_link)系角速度 rad/s（已减零偏、已预旋到车体系）。
  void publish_pose(const lightning::NavState &state, const fins::AcqTime &ts,
                    const lightning::Vec3d &ang_vel_body) {
    lightning::Mat3d Rb;
    lightning::Vec3d tb;
    {
      std::lock_guard<std::mutex> lock(extrinsic_mutex_);
      Rb = base_R_lidar_;
      tb = base_T_lidar_;
    }
    lightning::Mat3d R_WB = state.rot_.matrix();
    lightning::Vec3d pos = state.pos_;
    lightning::Mat3d R_odom_base;
    lightning::Vec3d t_odom_base;
    if (refactor_enabled_) {
      // TF 外参已注入 LIO，内部 body==base_link，state 即 odom->base_link。
      // 与 lightning slam_node 的 refactor 分支完全一致。
      R_odom_base = R_WB;
      t_odom_base = pos;
    } else {
      // 降级路径（无 tf_static / Livox）：照搬 FAST_LIO 的共轭式。
      lightning::Mat3d R_L_I = mapper_->GetLidarImuR();
      lightning::Vec3d T_L_I = mapper_->GetLidarImuT();
      lightning::Mat3d Rbi = Rb.transpose();
      R_odom_base = Rb * R_WB * R_L_I * Rbi;
      t_odom_base = Rb * (R_WB * (R_L_I * (-Rbi * tb) + T_L_I) + pos) + tb;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = fins::to_ros_msg_time(ts);
    odom.header.frame_id = initial_frame_;
    odom.child_frame_id = base_link_frame_;
    odom.pose.pose.position.x = t_odom_base(0);
    odom.pose.pose.position.y = t_odom_base(1);
    odom.pose.pose.position.z = t_odom_base(2);
    Eigen::Quaterniond q(R_odom_base);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // ---- twist：参考 FAST_LIO laserMapping.cpp 的计算 ----
    // state.vel_ 是 IMU(body) 原点在世界系下的速度；转到 IMU 体系后做刚体
    // 速度修正，再转到 base_link。lightning 预旋后 IMU 体系已对齐车体，
    // 故 ang_vel_body 直接作为车体角速度，不再乘 R_LI^T。
    lightning::Vec3d vel_imu_in_imu = R_WB.transpose() * state.vel_;
    // p_base 相对 IMU 原点的杆臂（IMU 体系）：lightning 外参已注入为 lidar->base，
    // body 即 base，杆臂近似为零；保留刚体项以兼容降级路径。
    lightning::Vec3d lever_imu = lightning::Vec3d::Zero();
    if (!refactor_enabled_) {
      lightning::Mat3d R_L_I = mapper_->GetLidarImuR();
      lightning::Vec3d T_L_I = mapper_->GetLidarImuT();
      lever_imu = R_L_I * (-Rb.transpose() * tb) + T_L_I;
    }
    lightning::Vec3d vel_base = vel_imu_in_imu + ang_vel_body.cross(lever_imu);
    odom.twist.twist.linear.x = vel_base(0);
    odom.twist.twist.linear.y = vel_base(1);
    odom.twist.twist.linear.z = vel_base(2);
    odom.twist.twist.angular.x = ang_vel_body(0);
    odom.twist.twist.angular.y = ang_vel_body(1);
    odom.twist.twist.angular.z = ang_vel_body(2);
    if (required("odometry")) send("odometry", odom, ts);
    if (required("$T_{odom}^{base}$")) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = odom.child_frame_id;
      tf.transform.translation.x = t_odom_base(0);
      tf.transform.translation.y = t_odom_base(1);
      tf.transform.translation.z = t_odom_base(2);
      tf.transform.rotation = odom.pose.pose.orientation;
      send("$T_{odom}^{base}$", tf, ts);
    }

    if (fixed_map_active_ && required("localization_pose")) {
      Eigen::Isometry3d global = Eigen::Isometry3d::Identity();
      global.linear() = R_odom_base;
      global.translation() = t_odom_base;
      Eigen::Isometry3d pose = map_to_odom_ * global;
      geometry_msgs::msg::PoseWithCovarianceStamped p;
      p.header.stamp = odom.header.stamp;
      p.header.frame_id = "map";
      p.pose.pose.position.x = pose.translation().x();
      p.pose.pose.position.y = pose.translation().y();
      p.pose.pose.position.z = pose.translation().z();
      Eigen::Quaterniond pq(pose.linear());
      p.pose.pose.orientation.w = pq.w();
      p.pose.pose.orientation.x = pq.x();
      p.pose.pose.orientation.y = pq.y();
      p.pose.pose.orientation.z = pq.z();
      for (int i = 0; i < 6; ++i) p.pose.covariance[i * 6 + i] = 0.01;
      send("localization_pose", p, ts);
    }
    if (fixed_map_active_ && required("$T_{map}^{odom}$")) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = odom.header.stamp;
      tf.header.frame_id = "map";
      tf.child_frame_id = initial_frame_;
      Eigen::Quaterniond mq(map_to_odom_.linear());
      tf.transform.rotation.w = mq.w();
      tf.transform.rotation.x = mq.x();
      tf.transform.rotation.y = mq.y();
      tf.transform.rotation.z = mq.z();
      tf.transform.translation.x = map_to_odom_.translation().x();
      tf.transform.translation.y = map_to_odom_.translation().y();
      tf.transform.translation.z = map_to_odom_.translation().z();
      send("$T_{map}^{odom}$", tf, ts);
    }
  }

  static pcl::PointCloud<pcl::PointXYZI>::Ptr to_ros_cloud(
      const lightning::CloudPtr &src, const std::string &frame_id, const fins::AcqTime &ts) {
    auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    out->reserve(src->size());
    for (const auto &p : src->points) {
      pcl::PointXYZI q;
      q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = p.intensity;
      out->push_back(q);
    }
    // PCL2ROS 直接透传 header：必须在这里设 frame_id/stamp，否则发布出去的
    // /cloud_registered 没有坐标系（与 FAST-LIO 行为一致）。
    out->header.frame_id = frame_id;
    out->header.stamp = fins::to_microseconds(ts);
    out->width = out->size();
    out->height = 1;
    out->is_dense = false;
    return out;
  }

  void publish_clouds(const fins::AcqTime &ts) {
    auto state = mapper_->GetState();
    lightning::Mat3d Rb;
    lightning::Vec3d tb;
    {
      std::lock_guard<std::mutex> lock(extrinsic_mutex_);
      Rb = base_R_lidar_;
      tb = base_T_lidar_;
    }
    lightning::Mat3d Rw = state.rot_.matrix();
    lightning::Vec3d pw = state.pos_;
    lightning::Mat3d R_ol;
    lightning::Vec3d t_ol;
    if (refactor_enabled_) {
      // TF 外参已注入：state 是 odom->base，lidar 点在 body(=base) 系，
      // 故 odom<-lidar = R_WB * R_bl, t = R_WB * t_bl + pos（slam_node 同款）。
      R_ol = Rw * Rb;
      t_ol = Rw * tb + pw;
    } else {
      // 降级路径：FAST_LIO 同款（IMU 世界系 + yaml 外参 + base TF）。
      lightning::Mat3d R_L_I = mapper_->GetLidarImuR();
      lightning::Vec3d T_L_I = mapper_->GetLidarImuT();
      R_ol = Rb * Rw * R_L_I;
      t_ol = Rb * (Rw * T_L_I + pw) + tb;
    }
    if (required("cloud")) {
      send("cloud", to_ros_cloud(mapper_->GetCloudWithTransform(R_ol, t_ol),
                                 initial_frame_, ts), ts);
    }
    if (required("cloud_body")) {
      // base_link 系点云：lidar body 点经 lidar->base 外参 Rb/tb
      send("cloud_body", to_ros_cloud(mapper_->GetCloudWithTransform(Rb, tb),
                                      base_link_frame_, ts), ts);
    }
  }

  std::shared_ptr<lightning::LaserMapping> mapper_;
  fins::AcqTime last_acq_time_{};
  std::thread backend_thread_;
  std::mutex trigger_mtx_;
  std::condition_variable trigger_cv_;
  std::atomic<bool> is_running_{false};
  bool has_new_data_{false};

  std::mutex alignment_mutex_;
  geometry_msgs::msg::TransformStamped pending_alignment_;
  bool alignment_pending_{false};
  std::mutex extrinsic_mutex_;
  lightning::Mat3d base_R_lidar_ = lightning::Mat3d::Identity();
  lightning::Vec3d base_T_lidar_ = lightning::Vec3d::Zero();
  // TF 外参注入 LIO 后：IMU 数据预旋矩阵 R_pre = R_bl * lidar_to_imu_R^T，
  // 并使内部 body 系 == base_link，发布/建图口径与 lightning slam_node refactor 一致。
  lightning::Mat3d R_pre_ = lightning::Mat3d::Identity();
  bool refactor_enabled_{false};
  bool extrinsic_received_{false};
  std::deque<sensor_msgs::msg::Imu> pending_imu_;  // TF 注入前缓存的原始 IMU

  std::mutex state_mutex_;
  std::string tracking_status_ = "WAITING_INITIALIZATION";
  int good_matches_ = 0, bad_matches_ = 0;
  bool fixed_map_active_ = false;
  Eigen::Isometry3d map_to_odom_ = Eigen::Isometry3d::Identity();

  std::mutex snapshot_mutex_;
  lightning::ESKF kf_copy_;
  lightning::ESKF::ProcessNoiseType Q_ = lightning::ESKF::ProcessNoiseType::Identity();
  bool imu_rate_live_ = false;
  double last_imu_time_ = -1.0;
  // twist 计算：IMU-rate 角速度滑动窗口（FAST_LIO 同款），scan-rate 角速度取自相邻帧姿态差。
  int imu_window_size_ = 10;
  std::deque<lightning::Vec3d> ang_vel_window_;
  lightning::Vec3d ang_vel_sum_ = lightning::Vec3d::Zero();
  struct ScanRot { lightning::SO3 rot; double time = 0.0; bool valid = false; };
  ScanRot last_scan_rot_;

  bool fixed_map_enabled_ = false;
  bool imu_rate_output_ = true;
  std::string map_path_, initial_frame_ = "odom", base_link_frame_ = "base_link";
  double map_leaf_size_ = 0.15, max_mean_residual_ = 1.11;
  double max_alignment_age_ = 60.0;
  int min_effective_points_ = 30, failures_before_lost_ = 5, good_frames_to_lock_ = 3;
};

EXPORT_NODE(FasterLIO)

#endif
