/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>

#include <vector>
#include <numeric>
#include <algorithm>
#include <cassert>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <glog/logging.h>

#include "parameters.h"
#include "../utility/tic_toc.h"

/** NOTE: 理解【特征点】和【像素点】的区别
 * 我们认为【特征点】是环境中的一个客观存在，不以观测位置为转移，不以图像帧为转移。
 * 而【像素点】是图像中的一个像素，像素点可以表达特征点在图像中的存在。
*/

/// @brief 一个（对）像素点，是特征点在一帧图像中的表达，是特征点的最小表达单位。
class FeaturePerFrame {
   public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }

    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point) {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }

    double cur_td;
    Vector3d point, pointRight;         //特征点的像素坐标（去相机畸变后的），第三维实际无效
    Vector2d uv, uvRight;               //特征点的原始像素坐标（未去畸变）
    Vector2d velocity, velocityRight;   //像素速度
    bool is_stereo;                     //表征特征点在当前帧中是否被双目共视到
};

/// @brief 因为一个特征点可能在多个帧（frame）中被追踪到，并且在不同帧中的像素坐标都不一样，
/// 这个结构体就是锚定FeatureID，保存特征点在所有帧中的观测信息。
class FeaturePerId {
   public:
    vector<FeaturePerFrame> feature_per_frame; /*特征点在不同帧中的观测描述*/
    const int feature_id;
    int start_frame;
    int used_num;
    double estimated_depth;
    int solve_flag; /* 0 haven't solve yet; 1 solve succ; 2 solve fail */

    FeaturePerId(int _featureId, int _startFrame)
        : feature_id(_featureId), start_frame(_startFrame)
        , used_num(0), estimated_depth(-1.0), solve_flag(0) {}

    // 当前，观测到这个特征点的最后一帧的id
    int endFrame() { return start_frame + feature_per_frame.size() - 1; }
};


/// @brief 如名，持有特征点，负责特征点的三角化&深度恢复，负责xxx
class FeatureManager {
   public:
    FeatureManager(Matrix3d _Rs[]);

    void clearState();
    // void setRic(Matrix3d _ric[]);

    /// @brief 核心接口，干两件事：特征点入容器，计算这个帧上特征点的平均视差并判断是否为关键帧
    bool addFeatureCheckParallax(int _frameCount, 
        const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_image, double td);

    /// @brief 返回共视特征点在左右目中的【像素坐标】，服务于VIO中计算某种相对位姿（在线标定和初始化SfM）
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frameCountL, int frameCountR);

    /** ----- README: 特征点管理之深度值管理：以下4个函数提供了管理特征点深度值的全部接口！ ----- */

    /// @brief [原getFeatureCount]统计被跟踪到的次数>=4次的特征点的数量
    int getRobustFeatureCount();

    /// @brief [原getDepthVector]统计被跟踪到的次数>=4次的特征点的深度信息，给到外部
    VectorXd getRobustFeatureDepthVec();

    /// @brief [原setDepth]接上个函数，对跟踪次数>=4次的特征点，外部（VIO）做完深度估计后，将结果返回
    void setRobustFeatureDepth(const VectorXd &x);

    /// @brief [原clearDepth]清空持有的全部特征点的深度信息 
    void clearAllFeatureDepth();

    /// @brief 核心接口，初始化和常规VIO都需要调这个，所谓三角化是对未初始化的特征点初始化一个深度信息。
    /// @param frameCnt 需要初始化的帧
    /// @param Ps 滑窗中的状态量（位置）
    /// @param Rs 滑窗中的状态量（姿态）
    /// @param tic 外参平移
    /// @param ric 外参旋转
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);

    /// @brief 功能函数，通过位姿和像素坐标，计算单个特征点的深度值
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                          Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);

    /// @brief xxx
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);

    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                        vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);

    /// @brief xxx
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, 
                              Eigen::Matrix3d new_R, Eigen::Vector3d new_P);

    void removeBack();
    void removeFront(int _frameCount);

    /// @brief xxx
    void removeOutlier(set<int> &outlierIndex);

    /// @brief xxx
    void removeFailures();

    // 这几个都会被外部直接访问
    list<FeaturePerId> features_;   //当前持有的特征点
    double last_average_parallax_;  //貌似没啥用
    int new_feature_num_;           //当前输入帧中，新出现的feature的数量
    int last_track_num_;            //当前输入帧中，已经存在的feature的数量
    int long_track_num_;            //当前输入帧中，已经被观测到>=4次的feature的数量

   private:

    /// @brief 功能函数，针对一个被多次观测到feature，计算其视差
    double compensatedParallax2(const FeaturePerId &it_per_id, int _frameCount);

    // const Matrix3d *Rs;             // unused
    // Matrix3d ric[2];                // unused
};

#endif // FEATURE_MANAGER_H