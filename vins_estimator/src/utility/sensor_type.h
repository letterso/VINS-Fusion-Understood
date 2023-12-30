/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <boost/shared_ptr.hpp>
#include <boost/make_unique.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>


// namespace VinsNS {

/// 定义一些传感器观测的数据结构,尽量和ROS消息类型一致

struct ImuData
{
    double timestamp = 0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d orientation_covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d angular_velocity_covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Matrix3d linear_acceleration_covariance = Eigen::Matrix3d::Zero();

    ImuData() = default;
    ImuData(double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr)
        : timestamp(t), linear_acceleration(acc), angular_velocity(gyr) {}

    using Ptr = std::shared_ptr<ImuData>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct WheelData
{
    double timestamp = 0;                   // 时间戳 (s)
    double wheel_base = 0;                  // 轴距 (meter) /*unused*/
    double wheel_track = 0;                 // 轮距 (meter) /*unused*/
    double motor_speed = 0;                 // 动力机械速度 (m/s) /*unused*/
    double steer_angle = 0;                 // 转向角 (radius)
    Eigen::Vector4d wheel_speed = Eigen::Vector4d::Zero();      // 轮速 (m/s)
    Eigen::Vector4d wheel_pulse = Eigen::Vector4d::Zero();      // 脉冲数 (number)
    Eigen::Vector4d wheel_angle = Eigen::Vector4d::Zero();      // 轮角 (radius)
    Eigen::Vector4d wheel_pressure = Eigen::Vector4d::Zero();   // 胎压 (bar)

    using Ptr = std::shared_ptr<WheelData>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/// 单个GPS读数的状态
enum class GnssStatus {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};

struct GnssData
{
    //
    double timestamp = 0;                           // 时间戳(s)
    GnssStatus status = GnssStatus::GNSS_NOT_EXIST; // 状态位
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    using Ptr = std::shared_ptr<GnssData>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// } // namespace VinsNS
