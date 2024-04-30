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

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

#define LET_NET
#include "net.h"
#define LET_WIDTH 640 // 512
#define LET_HEIGHT 512 // 384

using namespace std;
using namespace Eigen;
using namespace camodocal;

// bool inBorder(const cv::Point2f &pt);
// void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
// void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker {
   public:
    FeatureTracker();

    /// @brief 顾名思义，读入相机内参
    void readIntrinsicParameter(const vector<string> &calib_file);

    /// @brief FeatureTracker类对外的主要接口，完成特征点的追踪和补充，返回追踪到的特征点
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(
        double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

    /// @brief 在上层逻辑中，将检测到的outlier传进来，下一帧中将不再追踪
    void removeOutliers(set<int> &removePtsIds);

    /// @brief FeatureTracker类的对外接口之一，给入下一帧中的feature位置的预测
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);

    /// @brief 在完成track后，向外界绘制track结果的图像，用于ui显示
    cv::Mat getTrackImage();

    /// @brief 调用opencv，在独立窗口显示图像
    void showUndistortion(const string &name);

    // ********************** 以下为实际意义上的“私有函数” ********************** //

    // 貌似是用FundamentalMatrix+RANSAC来过滤outlier特征，但作者实际未启用
    // 个人猜测是过滤outlier这个事现在由外层做了，类内无需考虑
    void rejectWithF();

    // [原setMask函数]当特征点数量不足时，需要提取新的角点，这时要把现有特征点用MASK屏蔽掉
    void setCurrFeatureAsMask();

    // void undistortedPoints(); /*空函数*/

    // 根据相机内参，对track到的特征点做去畸变（也即像素去畸变）
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &PTs, camodocal::CameraPtr CAM);

    // 计算特征点在前后帧图像之间的移动速度（也即像素移动速度），在像素去畸变之后进行
    vector<cv::Point2f> ptsVelocity(vector<int> &IDs, vector<cv::Point2f> &PTs, 
        map<int, cv::Point2f> &curIdPtMap, map<int, cv::Point2f> &preIdPtMap);

    // void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
    //     vector<cv::Point2f> PTs1, vector<cv::Point2f> PTs2); /*空函数*/

    // letnet初始化
    void letnetInit();

    // letnet推理获取权重和特征
    void letnetProcess(const cv::Mat &imageBgr);

    // 选择用于跟踪的特征点
    void letnetFeaturesToTrack(cv::InputArray image,
                               cv::OutputArray corners,
                               const int &maxCorners,
                               const double &qualityLevel,
                               const double &minDistance,
                               const cv::InputArray &mask,
                               int blockSize = 3);

    // 将track结果绘制到图像上，供外界显示
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
        vector<int> &curLeftIds, vector<cv::Point2f> &curLeftPts, 
        vector<cv::Point2f> &curRightPts, 
        map<int, cv::Point2f> &prevPtsMap);

    // 计算像素距离
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);

    // 判断像素是否位于图像内
    bool inBorder(const cv::Point2f &pt);

    // ************* 以下，我们按照各成员的含义和出现顺序，重新排列了它们，并给出注释 ************* //

    bool flag_stereo_cam_;                  //通过读入的相机内参的数量，自动判断是否为双目 
    vector<camodocal::CameraPtr> m_camera;  //相机类（含内参），用于像素坐标去畸变，双目的话有两个

    int pt_id_;                             //[原n_id]全周期内的特征点id值，持续累加

    double cur_time;                        //新帧进入后，首先赋值这些变量
    cv::Mat cur_img;                        //当前帧（双目时特指左目，右目无需保存图像）
    int row_, col_;                         //图像行列数

    double prev_time;                       //上一帧时间戳，用于计算feature移动速度
    cv::Mat prev_img;                       //上一帧图像（双目时特指左目）

    vector<cv::Point2f> cur_pts_;           //当前帧左目上的feature，新帧进入后会先清空
    vector<cv::Point2f> cur_right_pts_;     //同上，当前帧右目图像上的feature

    vector<int> cur_ids_, iDs_right_;       //当前追踪到的特征点的ID，是标识不同特征点的唯一信息
    vector<int> tracked_times_;             //[track_cnt]当前追踪到的特征点，一共被多少帧图像追踪到；会自行清退失去跟踪的特征

    bool has_predict_feats_;                //当前帧是否存在feature位置预测，可用于加快追踪速度
    vector<cv::Point2f> predict_pts_;       //当前帧feature的位置预测，由外部给定
    vector<cv::Point2f> predict_pTs_show;   //开发过程中调试用，可忽略

    cv::Mat exist_pts_mask_;                //提取新corner特征时用到的MASK，255代表可提取，0代表MASK
    // cv::Mat fisheye_mask_;               //未用到，可能是预留fisheye方案
    vector<cv::Point2f> new_pts_;           //追踪到的特征点不足时，需要补充新的角特征

    vector<cv::Point2f> prev_pts_; //, cur_pts_, cur_right_pts_;             //上一帧、当前帧、当前帧右目，的feature点（也即像素坐标）
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_right_un_pts;          //去畸变之后的像素坐标
    vector<cv::Point2f> pts_velocity_, right_pts_velocity_;                 //feature像素在前后帧之间的移动速度

    map<int, cv::Point2f> cur_un_pts_map_, prev_un_pts_map_;                //存放【特征点ID】和【特征点像素坐标(去畸变后的)】构成的键值对
    map<int, cv::Point2f> cur_right_un_pts_map_, prev_right_un_pts_map_;    //同上，但对应特征点【在右目图像上的像素坐标(去畸变后的)】
    map<int, cv::Point2f> prev_pts_map_;                                    //当前帧处理完成后，保存特征点【在左目上的原始像素坐标】，用于绘制追踪结果以可视化
    cv::Mat img_track_show_;                                                //[原imTrack]绘制track结果到这个图像上，用于ui显示

    ncnn::Net net_;
    cv::Mat score_;
    cv::Mat desc_, last_desc_;
    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0};
    const float mean_vals_inv[3] = {0, 0, 0};
    const float norm_vals_inv[3] = {255.f, 255.f, 255.f};
};
