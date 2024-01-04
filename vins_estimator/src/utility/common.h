/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#pragma once

#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>


struct VioState {
    /** 状态量： P, V, Q, Ba, Bg, G, TimeOffset, Extrinsics */
    size_t seq_ = 0;
    bool valid_ = false;
    double timestamp_ = 0;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bias_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_gyr_ = Eigen::Vector3d::Zero();
    double time_offset_c2i_ = 0;
    Eigen::Vector3d grav_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d T_l2i = Eigen::Vector3d::Zero();
    Eigen::Quaterniond R_l2i = Eigen::Quaterniond::Identity();

    double latest_tracking_usage_;
    double latest_procmeas_usage_; /*滞后一次*/
    double visualz_imgLK_usage_;
    double visualz_state_usage_; /*滞后一次*/
    double latest_whole_usage_; /*近似结果*/
};


struct RunningStatus {
    /** 1. 状态量
     * P, V, Q, Ba, Bg, G, TimeOffset
    */
    size_t seq_ = 0;
    bool valid_ = false;
    double timestamp_ = 0;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bias_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_gyr_ = Eigen::Vector3d::Zero();
    double time_offset_c2i_ = 0;
    Eigen::Vector3d T_l2i = Eigen::Vector3d::Zero();
    Eigen::Quaterniond R_l2i = Eigen::Quaterniond::Identity();

    /** 2. 各种耗时
     * 预处理（Lidar预处理、IMU处理+点云运动补偿、ikdtree动态裁剪、点云降采样）
     * ikdtree初始化+可视化
     * ieskf算法
     * ikdtree更新
    */
    double lidar_preproc_ = 0;
    double imu_proc_ = 0;
    double input_dsample_ = 0;
    double ikdtree_trim_ = 0;
    double visualize_tree_ = 0;
    double ieskf_sum_ = 0;
    double ieskf_piece1_ = 0;
    double ieskf_piece2_ = 0;
    double ieskf_piece3_ = 0;
    double ieskf_piece4_ = 0;
    double ikdtree_update_ = 0;
    double lio_pipeline_ = 0;
    double sys_running_time_ = 0;

    RunningStatus() = default;
};


class LogRater {
   public:
    LogRater(bool mode_time = true, double every_n = 1.0, int init_cnts = 1.0)
        : mode_by_time_(mode_time), log_every_secs_(every_n), 
        log_every_cnts_(every_n), initial_cnts_(init_cnts) {}
    ~LogRater() {}

    bool Sample(double timestamp = -1.0) 
    {
        if (mode_by_time_) {
            all_cnts_++;
            if (timestamp - last_log_time_ > log_every_secs_ || all_cnts_ <= initial_cnts_) {
                last_log_time_ = timestamp;
                return true;
            }
            return false;
        }
        else {
            all_cnts_++;
            if (all_cnts_ % log_every_cnts_ == 0 || all_cnts_ <= initial_cnts_) {
                return true;
            }
            return false;
        }
        return false;
    }

    size_t GetCounts() { return all_cnts_; }

   private:
    bool mode_by_time_ = true;
    double last_log_time_ = -1.0;
    double log_every_secs_ = 1.0;
    int    log_every_cnts_ = 1;
    size_t all_cnts_ = 0;
    size_t last_log_cnts_ = 0;
    size_t initial_cnts_ = 1;
};