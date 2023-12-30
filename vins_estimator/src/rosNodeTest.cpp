/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <memory>
#include <cassert>

#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "ui_pangolin/ui_window.h"
#include "utility_ros/RosVisualization.h"

// 预留gflag变量
DEFINE_string(config_path, "./config/vins_xxx.yaml", "配置文件路径");
DEFINE_double(config_param, 0.0, "配置参数");
DEFINE_bool(config_option, true, "配置选项");

std::unique_ptr<VinsEstimator> vins_estimator_;
std::unique_ptr<UiWindow> ui_window_;

// queue<sensor_msgs::ImuConstPtr> imu_buf;
// queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex mtx_buf;

std::atomic<bool> flag_exit_sync;

void Image0Callback(const sensor_msgs::ImageConstPtr &img_msg) {
    mtx_buf.lock();
    img0_buf.push(img_msg);
    mtx_buf.unlock();
}

void Image1Callback(const sensor_msgs::ImageConstPtr &img_msg) {
    mtx_buf.lock();
    img1_buf.push(img_msg);
    mtx_buf.unlock();
}

cv::Mat RosMsgToCvMat(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat cv_img = ptr->image.clone();
    return cv_img;
}

// extract images with same timestamp from two topics
void SyncImageMsgs() {
    while(1) {
        if (flag_exit_sync.load()) {
            LOG(INFO) << "msgs loop terminating ...";
            break;
        }
        if(STEREO) {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            mtx_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty()) {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003) {
                    img0_buf.pop();
                    LOG(WARNING) << "[Sync] throw img0 (too old)";
                }
                else if(time0 > time1 + 0.003) {
                    img1_buf.pop();
                    LOG(WARNING) << "[Sync] throw img1 (too old)";
                }
                else {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = RosMsgToCvMat(img0_buf.front());
                    img0_buf.pop();
                    image1 = RosMsgToCvMat(img1_buf.front());
                    img1_buf.pop();
                }
            }
            mtx_buf.unlock();
            if(!image0.empty()) {
                vins_estimator_->inputImage(time, image0, image1);
            }
        }
        else {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            mtx_buf.lock();
            if(!img0_buf.empty()) {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = RosMsgToCvMat(img0_buf.front());
                img0_buf.pop();
            }
            mtx_buf.unlock();
            if(!image.empty()) {
                vins_estimator_->inputImage(time, image);
            }
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    vins_estimator_->inputIMU(t, acc, gyr);
    if (ui_window_) {
        ui_window_->UpdateRawImu(ImuData(t, acc, gyr));
    }
    return;
}

void FeatureCallback(const sensor_msgs::PointCloudConstPtr &feature_msg) {
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++) {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5) {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        // ROS_ASSERT(z == 1);
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    vins_estimator_->inputFeature(t, featureFrame);
    return;
}

void RestartCallback(const std_msgs::BoolConstPtr &restart_msg) {
    if (restart_msg->data == true) {
        // ROS_WARN("restart the estimator!");
        LOG(WARNING) << "restart the estimator!";
        vins_estimator_->clearState();
        vins_estimator_->setParameter();
    }
    return;
}

void ImuSwitchCallback(const std_msgs::BoolConstPtr &switch_msg) {
    if (switch_msg->data == true) {
        //ROS_WARN("use IMU!");
        LOG(WARNING) << "Enable IMU in VINS system.";
        vins_estimator_->changeSensorType(1, STEREO);
    }
    else {
        //ROS_WARN("disable IMU!");
        LOG(WARNING) << "Disable IMU in VINS system.";
        vins_estimator_->changeSensorType(0, STEREO);
    }
    return;
}

void CamSwitchCallback(const std_msgs::BoolConstPtr &switch_msg) {
    if (switch_msg->data == true) {
        //ROS_WARN("use stereo!");
        LOG(WARNING) << "Enable Stereo-Mode in VINS system.";
        vins_estimator_->changeSensorType(USE_IMU, 1);
    }
    else {
        //ROS_WARN("use mono camera (left)!");
        LOG(WARNING) << "Enable Mono-Mode in VINS system.";
        vins_estimator_->changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    // FLAGS_log_dir="."; /*默认[/tmp/程序名.INFO]*/
    // google::ParseCommandLineFlags(&argc, &argv, true); //预留

    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle nh_("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc < 2) {
        printf("please intput: rosrun vins vins_node [config file] [ui option] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    readParameters(config_file);

    if (argc == 3) {
        string ui_option = argv[2];
        printf("ui_option: %s\n", ui_option);
        ENABLE_UI = true;
    }

    registerPub(nh_);
    vins_estimator_.reset(new VinsEstimator);
    vins_estimator_->setParameter();
    vins_estimator_->setLatestOdometryHandle(pubLatestOdometry);
    vins_estimator_->setTrackingImageHandle(pubTrackImage);
    vins_estimator_->setStatisticsHandle(printStatistics);
    vins_estimator_->setVioStatesHandle(
        [&](const VinsEstimator& estimator) {
            pubOdometry(estimator);
            pubKeyPoses(estimator);
            pubCameraPose(estimator);
            pubPointCloud(estimator);
            pubKeyframe(estimator);
            pubTF(estimator);
            if (ui_window_) {
                ui_window_->UpdateVinsStatus(estimator);
            }
        }
    );

    if (ENABLE_UI) {
        ui_window_.reset(new UiWindow);
        ui_window_->Init();
    }

    flag_exit_sync.store(false);

#ifdef EIGEN_DONT_PARALLELIZE
    // ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
    LOG(WARNING) << "EIGEN_DONT_PARALLELIZE";
#endif

    // ROS_WARN("waiting for image and imu ...");
    LOG(WARNING) << "system inited, waiting for image and imu ...";

    ros::Subscriber sub_imu;
    if(USE_IMU) {
        sub_imu = nh_.subscribe(IMU_TOPIC, 2000, ImuCallback, ros::TransportHints().tcpNoDelay());
    }
    ros::Subscriber sub_feature = nh_.subscribe("/feature_tracker/feature", 2000, FeatureCallback);
    ros::Subscriber sub_img0 = nh_.subscribe(IMAGE0_TOPIC, 100, Image0Callback);
    ros::Subscriber sub_img1;
    if(STEREO) {
        sub_img1 = nh_.subscribe(IMAGE1_TOPIC, 100, Image1Callback);
    }
    ros::Subscriber sub_restart = nh_.subscribe("/vins_restart", 100, RestartCallback);
    ros::Subscriber sub_imu_switch = nh_.subscribe("/vins_imu_switch", 100, ImuSwitchCallback);
    ros::Subscriber sub_cam_switch = nh_.subscribe("/vins_cam_switch", 100, CamSwitchCallback);

    std::thread sync_thread{SyncImageMsgs};
    ros::spin();

    LOG(INFO) << "program exit ...";
    flag_exit_sync.store(true);
    sync_thread.join();
    LOG(INFO) << "sync msgs thread joined.";
    return 0;
}
