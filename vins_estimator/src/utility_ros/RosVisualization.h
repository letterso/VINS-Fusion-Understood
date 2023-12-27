/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <fstream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

#include "CameraPoseVisualization.h"
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"


// extern ros::Publisher pub_odometry;
// extern ros::Publisher pub_path, pub_pose;
// extern ros::Publisher pub_cloud, pub_map;
// extern ros::Publisher pub_key_poses;
// extern ros::Publisher pub_ref_pose, pub_cur_pose;
// extern ros::Publisher pub_key;
// extern nav_msgs::Path path;
// extern ros::Publisher pub_pose_graph;
// extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);

// 工作在sensor频率
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

// 工作在sensor频率
void pubTrackImage(const cv::Mat &imgTrack, const double t);

// ========================== 以下所有，都工作在VIO频率 ========================== //

void printStatistics(const VinsEstimator &estimator, double t);

void pubOdometry(const VinsEstimator &estimator, const std_msgs::Header &header = std_msgs::Header());

// void pubInitialGuess(const VinsEstimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const VinsEstimator &estimator, const std_msgs::Header &header = std_msgs::Header());

void pubCameraPose(const VinsEstimator &estimator, const std_msgs::Header &header = std_msgs::Header());

void pubPointCloud(const VinsEstimator &estimator, const std_msgs::Header &header = std_msgs::Header());

void pubKeyframe(const VinsEstimator &estimator);

void pubTF(const VinsEstimator &estimator, const std_msgs::Header &header = std_msgs::Header());

// void pubRelocalization(const VinsEstimator &estimator);

// void pubCar(const VinsEstimator & estimator, const std_msgs::Header &header);

