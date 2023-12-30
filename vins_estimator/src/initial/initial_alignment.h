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

#pragma once

#include <iostream>
#include <map>

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include "../estimator/feature_manager.h"

using namespace Eigen;
using namespace std;

/// @brief VINS层的一个处理单元，含图像帧、IMU预积分等所有必要信息
class ImageFrame {
   public:
    ImageFrame(){};
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t)
        :t{_t}, is_key_frame{false} {
        points = _points;
    }

    // 帧特征点（FeatID, <CamID, xyz_uv_VxVy>）
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;

    double t;
    Matrix3d R;
    Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
};

/// @brief 陀螺仪bias校正
void solveGyroscopeBias(map<double, ImageFrame> &_all_img_frames, Vector3d* _Bgs);

/// @brief 
bool VisualIMUAlignment(map<double, ImageFrame> &_all_img_frames, Vector3d* _Bgs, Vector3d &g, VectorXd &x);