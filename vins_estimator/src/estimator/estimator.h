/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <mutex>
#include <atomic>
#include <thread>
#include <cassert>
#include <vector>
#include <queue>
#include <unordered_map>
#include <map>
#include <functional>

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/pose_manifold.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"


namespace vinslitex {
} // namespace vinslitex

namespace VinsNS {
} // namespace VinsNS


/** ###### VINS工作状态关键指标，可UI追踪 ######
 * 
 * 前后帧之间追踪的feature数量，新增的feature数量，左右目之间追踪的feature数量
 * 这三个环节的耗时（以ms计）
 * 
 * 滑窗优化的指标信息
*/


/// @brief VINS前端顶层类
class VinsEstimator {
   public:
    VinsEstimator();
    ~VinsEstimator();

    using handleLatestOdometry = std::function<void(
        const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)>;
    using handleTrackingImage = std::function<void(const cv::Mat &imgTrack, const double t)>;
    using handleStatistics = std::function<void(const VinsEstimator &estimator, double t)>;
    using handleVinsStatus = std::function<void(const VinsEstimator &estimator)>;

    // ========================== interfaces ========================== //

    /** XXX: 设置回调时多张个心眼，避免线程冲突或死锁类似的问题*/
    void setLatestOdometryHandle(const handleLatestOdometry& func_object) { handle_latest_odom_ = func_object; }
    void setTrackingImageHandle(const handleTrackingImage& func_object) { handle_track_image_ = func_object; }
    void setStatisticsHandle(const handleStatistics& func_object) { handle_statistics_ = func_object; }
    void setVinsStatusHandle(const handleVinsStatus& func_object) { handle_vio_states_ = func_object; }

    void clearState();
    void setParameter();
    void changeSensorType(int use_imu, int use_stereo);

    /// @brief 从外部人为给定初始位姿
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r); 

    /// @brief 入IMU缓存队列，IMU递推一次
    void inputIMU(double t, const Vector3d &linearAcc, const Vector3d &angularVel);

    /// @brief 入Images缓存队列，做即时预处理（也即特征点追踪）
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

    /// @brief 从外部直接给出预处理后的特征点追踪
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &frame_feats);

    /// @brief 凑齐一组观测，处理一次，【数据流pipeline】在这里（包括：数据校验、imu积分预测、调用VIO算法、向外输出结果、时效统计）
    void processMeasurements();

    // ========================== internals: pipeline modules ========================== //

    /// @brief 基于静止假设，用若干个imu观测，初始化VIO系统的第一个pose
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    /// @brief 这里是VIO流程中的imu处理步骤，包括（1）IMU预积分，（2）IMU常规积分递推一次、作为滑窗最新帧的预测（因此procImu必须在procImg之前进行！）
    void processIMU(double t, double dt, const Vector3d &_linearAcc, const Vector3d &_angularVelo);

    /// @brief 递推一次VIO系统，【算法pipeline】在这里（包括：feature初始化深度、BA优化、移除外点、滑动一次窗口、更新latest结果）
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_img_feats, const double _img_time);

    /// @brief 【仅单目初始化】仅用于Mono方案的初始化
    bool initialStructure();

    /// @brief 【仅单目初始化】正序搜索滑窗中的帧，尝试和最新帧构建相对位姿，返回成功的帧和位姿结果 [原relativePose]
    bool searchRelativePose(Matrix3d &_relativeR, Vector3d &_relativeT, int &_frameSeq);

    /// @brief 【仅单目初始化】Mono方案时，IMU预积分与视觉结构对齐    命名错误？应该是 visualInertialAlign    /// [原名,应该是作者笔误了]
    bool visualInitialAlign();

    /// @brief [原optimization()]执行优化？
    void runOptimization();

    /// @brief 完成一轮优化后，滑动一次窗口，初始化阶段和VIO阶段都会调用
    void slideWindow();

    /// @brief 检出重投影误差超过阈值的feature
    void outliersRejection(set<int> &removeIndex);

    /// @brief xxx
    void predictPtsInNextFrame();

    /// @brief 检测是否有极端情况（有导致系统失败的风险），实际未启用
    bool failureDetection();

    /// @brief 在完成一次滑窗优化后，调用这个函数递推最新的状态
    void updateLatestStates();

    /// @brief 基于当前最新状态，用IMU递推更新状态（使用简单中值积分法）
    void fastPredictIMU(double t, Eigen::Vector3d _linearAcc, Eigen::Vector3d _angularVelo);

    // ========================== internals: tool funcs ========================== //

    bool IMUAvailable(double t);
    bool getIMUInterval(double t0, double t1, 
        vector<pair<double, Eigen::Vector3d>> &accVector, 
        vector<pair<double, Eigen::Vector3d>> &gyrVector);

    // [vector2double]将状态量转化为ceres参数
    void StateToCeresParam();

    // [double2vector]将ceres优化结果保存到状态量中 
    void CeresParamToState();

    // 移除
    void slideWindowNew();
    void slideWindowOld();

    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);

    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &Ric_i, Vector3d &Tic_i,
                             Matrix3d &Rj, Vector3d &Pj, Matrix3d &Ric_j, Vector3d &Tic_j, 
                             double Depth, Vector3d &UVi, Vector3d &UVj);


    // ========================== internals: members ========================== //

    /// @brief 任务阶段{初始化阶段,常规VIO阶段}
    enum SolverFlag {
        INITIAL,
        NON_LINEAR
    };

    /// @brief 边缘化选项{最老帧,次新帧}
    enum MarginalizationFlag {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mtxBuf;                                      //启用Proc线程情况下的数据锁
    std::mutex mtxProc;                                     //Proc线程数据锁
    std::mutex mtxPropagate;                                //IMU正向传播数据锁

    queue<pair<double, Eigen::Vector3d>> accBuf;            //原始imu观测队列
    queue<pair<double, Eigen::Vector3d>> gyrBuf;            //原始imu观测队列
    queue<pair<double, map<int, vector<pair<
        int, Eigen::Matrix<double, 7, 1>>>>>> featureBuf;   //图像帧队列（帧时刻+帧特征点，7*1向量构成了特征点的一次观测{xyz_uv_VxVy}）

    map<double, ImageFrame> all_image_frame_;               //保存图像帧的容器，主要用于Mono方案下的SfM初始化

    std::thread track_thread_;                              //unused
    std::thread process_thread_;                            //用于执行processMeas的独立线程
    bool flag_init_proc_thread_;                            //线程是否已启动
    std::atomic<bool> flag_proc_exit_;                      //析构时退出线程

    FeatureTracker f_tracker_;                              //顾名思义，负责补充和追踪特种点
    FeatureManager f_manager_;                              //顾名思义，持有和管理所有特征点
    MotionEstimator motion_estimator_;                      //顾名思义，用于从一对儿像素坐标恢复位姿
    InitializeExtrinRotation init_extrin_rot_;              //顾名思义，旋转外参自标定模块

    SolverFlag flag_solver_type_;                           //当前的任务阶段（初始化阶段or常规VIO阶段）
    MarginalizationFlag flag_marglize_type_;                //当前要边缘化哪一帧（abbr:marglz,marglize,margnlztn）

    bool flag_init_1st_imu_pose_;                           //是否初始化了第一个ImuPose（主要是初始化重力方向）
    bool flag_open_extrin_esti_;                            //[原openExEstimation]在优化问题中，是否优化外参（若系统启用了外参估计，会一直置true）

    Matrix3d R_IC_[2];                                      //系统状态量：[原ric]外参旋转（最多两个相机） //根据崔华坤网络文章，应该是“Camera到IMU的外参”？
    Vector3d T_IC_[2];                                      //系统状态量：[原tic]外参平移（最多两个相机）

    Vector3d Grav_;                                         //系统状态量：[原g]SLAM/VIO系下的重力向量
    double Tdiff_;                                          //系统状态量：[原td]Cam和IMU间的时间同步误差，用于将feature对齐到IMU时刻上

    Vector3d Ps[(WINDOW_SIZE + 1)];                         //滑窗状态量：位置
    Vector3d Vs[(WINDOW_SIZE + 1)];                         //滑窗状态量：速度
    Matrix3d Rs[(WINDOW_SIZE + 1)];                         //滑窗状态量：姿态
    Vector3d Bas[(WINDOW_SIZE + 1)];                        //滑窗状态量：加计零偏
    Vector3d Bgs[(WINDOW_SIZE + 1)];                        //滑窗状态量：陀螺仪零偏
    double Headers[(WINDOW_SIZE + 1)];                      //滑窗中的时间戳（注意是image原始时间戳，未对齐到IMU时刻上）

    int frame_count;                                        //当前正在处理的帧，在滑窗中的索引，最大不超过10；注意滑窗中第一帧的索引总为0！
    std::atomic<int> num_optimz_tracked_feats_;             //滑窗优化中，追踪的鲁邦特征数量
    std::atomic<int> num_optimz_tracked_times_;             //滑窗优化中，鲁邦特征被追踪的帧次

    int num_input_image_;                                   //inputIMage函数收到的图像帧的计数
    double prev_time_, curr_time_;                          //上一图像帧、当前图像帧的时间戳（注意是对齐到IMU上的时刻，已经补齐了时间同步误差）
    double curr_sys_state_time_;                            //当前系统状态的时间戳（可用于外部发布当前VIO状态）
    std::atomic<double> curr_tracking_usage_;               //光流追踪的耗时
    std::atomic<double> curr_procmeas_usage_;               //一次ProcMeas处理的耗时
    std::atomic<double> ui_show_imgLK_usage_;               //UI显示的耗时
    std::atomic<double> ui_show_state_usage_;               //UI显示的耗时

    bool first_first_imu_;                                  //系统中，IMU普通积分的第一帧
    Vector3d acc_0, gyr_0;                                  //IMU普通积分递推的左帧（k-1 >> k）

    IntegrationBase *tmp_pre_integration_;                  //这个预积分器在完成预积分后，会把所有权移交图像帧；然后指针重置为新的预积分器等待下一轮

    IntegrationBase* pre_integrations_[(WINDOW_SIZE + 1)];  //滑窗帧对应的IMU预积分器
    vector<double> dt_buff_[(WINDOW_SIZE + 1)];             //Slide窗口相关[原dt_buf]
    vector<Vector3d> acc_buff_[(WINDOW_SIZE + 1)];          //Slide窗口相关[原linear_acc_buf]
    vector<Vector3d> gyr_buff_[(WINDOW_SIZE + 1)];          //Slide窗口相关[原angular_vel_buf]

    Matrix3d back_R0;                                       //执行Slide窗口前，滑窗中最老帧的姿态（与边缘化最老帧有关）
    Vector3d back_P0;                                       //执行Slide窗口前，滑窗中最老帧的位置（与边缘化最老帧有关）

    Matrix3d last_R0;                                       //执行完一次VIO的完整流程后，最终滑窗中的，最老帧姿态（可用于系统失效判断）
    Vector3d last_P0;                                       //执行完一次VIO的完整流程后，最终滑窗中的，最老帧位置（可用于系统失效判断）

    Matrix3d last_Rn;                                       //执行完一次VIO的完整流程后，最终滑窗中的，最新帧姿态（可用于系统失效判断）
    Vector3d last_Pn;                                       //执行完一次VIO的完整流程后，最终滑窗中的，最新帧位置（可用于系统失效判断）

    int sum_slide_old_, sum_slide_new_;                     //Slide计数，实际没啥用[原sum_of_back,sum_of_front]
    bool flag_failure_occur_;                               //系统失效标志位，实际没起作用

    // vector<Vector3d> point_cloud;                        //unused
    // vector<Vector3d> margin_cloud;                       //unused
    vector<Vector3d> key_poses_;                            //滑窗中所有帧的位置坐标（用于输出可视化）

    // ceres参数块，用于ceres优化（ceres要求必须是double对象/数组）
    double para_Pose_[WINDOW_SIZE + 1][SIZE_POSE];          //第一维是帧，第二维是xyz_QxQyQzQw
    double para_Speed_Bias_[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Features_[NUM_OF_F][SIZE_FEATURE];          //第一维为特征idx，第二维为帧idx，变量为【逆深度】
    double para_Ex_Pose_[2][SIZE_POSE];                     //第一维是相机index
    // double para_Retrive_Pose_[SIZE_POSE];/*未启用*/
    double para_Td_[1][1];
    // double para_Tr_[1][1];/*未启用*/

    MarginalizationManager* last_margnlztn_manager_;        //收集边缘化所需信息，执行边缘化，获得边缘化因子
    vector<double*> last_margnlztn_param_blocks_;           //边缘化因子对应的参数块，指针实际指向类的ceres参数块

    // int loop_window_index;  //unused

    // 当前系统的最新状态,来源于滑窗或者imu递推预测（可用于输出可视化等）
    double latest_time_;
    Eigen::Quaterniond latest_Q_;
    Eigen::Vector3d latest_P_, latest_V_;
    Eigen::Vector3d latest_Ba_, latest_Bg_;
    Eigen::Vector3d latest_Acc_0_, latest_Gyr_0_;

    // 这俩似乎都没啥用
    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    // 单目方案初始化时的一个时间戳
    double initial_timestamp;

// ========================== internals: private members ========================== //

   private:
    handleLatestOdometry handle_latest_odom_;   //be careful about deadlock.
    handleTrackingImage handle_track_image_;    //be careful about deadlock.
    handleStatistics handle_statistics_;        //be careful about deadlock.
    handleVinsStatus handle_vio_states_;        //be careful about deadlock.

};
