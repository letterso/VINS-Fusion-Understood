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
#include <vector>
#include <map>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>

#include "../utility/utility.h"

// ========================== constant params ========================== //

const double FOCAL_LENGTH = 460.0;  //系统常量
const int WINDOW_SIZE = 10;         //系统常量：滑窗规模
const int NUM_OF_F = 1000;          //系统常量：滑窗中feature数量上限

// #define UNIT_SPHERE_ERROR        //计算重投影误差的一个option，作者已关闭

// ========================== configurable system params ========================== //

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string IMU_TOPIC;

extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;

extern int ROLLING_SHUTTER;                 //使用的相机是否是卷帘快门
extern int ROW, COL;                        //如名
extern int NUM_OF_CAM;                      //如名
extern int STEREO;                          //是否为双目配置
extern int USE_IMU;                         //是否启用IMU
extern int MULTIPLE_THREAD;                 //是否在Vins前端（Estimator）中使用独立线程运行算法pipeline
extern std::string FISHEYE_MASK;            //未使用
extern std::vector<std::string> CAM_NAMES;  //相机内参文件的路径

extern double INIT_DEPTH;                   //视觉feature深度默认初值
extern double MIN_PARALLAX;                 //判定keyframe的最小视差

extern double ACC_N, ACC_W;                 //加计噪声、加计bias噪声
extern double GYR_N, GYR_W;                 //陀螺仪相应的噪声

extern double TD;                           //sensor间时间同步误差，可作为状态量估计

extern std::vector<Eigen::Matrix3d> RIC;    //外参旋转，可作为状态量估计
extern std::vector<Eigen::Vector3d> TIC;    //外参平移，可作为状态量估计

extern Eigen::Vector3d G;                   //重力常量（3*1矢量）

extern int ESTIMATE_TD;                     //是否在线估计这个时延
extern int ESTIMATE_EXTRINSIC;              //是否在线估计外参

extern double BIAS_ACC_THRESHOLD;           //unused
extern double BIAS_GYR_THRESHOLD;           //unused
extern double SOLVER_TIME;                  //优化求解允许的最大耗时
extern int NUM_ITERATIONS;                  //优化求解允许的最大迭代次数

extern int MAX_CNT;         //允许最多追踪多少个特征点，通常是150或200
extern int MIN_DIST;        //控制特征点密度的像素距离参数，使特征尽可能均匀分布
extern double F_THRESHOLD;  //用基础矩阵剔除outlier时的阈值（实际未启用）
extern int FLOW_BACK;       //是否启用反向追踪检验，通常是启用的
extern int SHOW_TRACK;      //是否在独立opencv窗口中显示图像，用于debug

extern std::map<int, Eigen::Vector3d> pts_gt;   //for debug purpose

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION {
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder {
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder {
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
