/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"

VinsEstimator::VinsEstimator(): f_manager_{Rs} {
    flag_proc_exit_.store(false);
    flag_init_proc_thread_ = false;
    clearState();
    LOG(INFO) << "vins estimator constructed. ";
}

VinsEstimator::~VinsEstimator() {
    LOG(INFO) << "vins estimator destructing ... ";
    if (MULTIPLE_THREAD) {
        flag_proc_exit_.store(true);
        process_thread_.join();
        LOG(INFO) << "proc measurements thread joined. ";
    }
}

// ========================== interfaces ========================== //

void VinsEstimator::clearState() {
    mtxProc.lock();
    while(!accBuf.empty()) {
        accBuf.pop();
    }
    while(!gyrBuf.empty()) {
        gyrBuf.pop();
    }
    while(!featureBuf.empty()) {
        featureBuf.pop();
    }

    prev_time_ = -1;
    curr_time_ = 0;
    curr_sys_state_time_ = 0;
    curr_tracking_usage_.store(0);
    curr_procmeas_usage_.store(0);
    ui_show_imgLK_usage_.store(0);
    ui_show_state_usage_.store(0);
    flag_open_extrin_esti_ = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    num_input_image_ = 0;
    flag_init_1st_imu_pose_ = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buff_[i].clear();
        acc_buff_[i].clear();
        gyr_buff_[i].clear();

        if (pre_integrations_[i] != nullptr) {
            delete pre_integrations_[i];
        }
        pre_integrations_[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        T_IC_[i] = Vector3d::Zero();
        R_IC_[i] = Matrix3d::Identity();
    }

    first_first_imu_ = false,
    sum_slide_old_ = 0;
    sum_slide_new_ = 0;
    frame_count = 0;
    num_optimz_tracked_feats_.store(0);
    num_optimz_tracked_times_.store(0);
    flag_solver_type_ = INITIAL;
    initial_timestamp = 0;
    all_image_frame_.clear();

    if (tmp_pre_integration_ != nullptr) {
        delete tmp_pre_integration_;
    }
    if (last_margnlztn_manager_ != nullptr) {
        delete last_margnlztn_manager_;
    }

    tmp_pre_integration_ = nullptr;
    last_margnlztn_manager_ = nullptr;
    last_margnlztn_param_blocks_.clear();

    f_manager_.clearState();

    flag_failure_occur_ = 0;

    mtxProc.unlock();
}

void VinsEstimator::setParameter() {
    mtxProc.lock();
    for (int i = 0; i < NUM_OF_CAM; i++) {
        T_IC_[i] = TIC[i];
        R_IC_[i] = RIC[i];
        LOG(INFO) << "[Params] exitrinsic cam " << i << ": \nR: \n" 
            << R_IC_[i] << "\nT: \n" << T_IC_[i].transpose();
    }

    // f_manager_.setRic(R_IC_);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    Tdiff_ = TD;
    Grav_ = G;
    LOG(INFO) << "[Params] set g: " << Grav_.transpose();
    f_tracker_.readIntrinsicParameter(CAM_NAMES);

    LOG(INFO) << "[Params] MULTIPLE_THREAD is: " << MULTIPLE_THREAD;
    if (MULTIPLE_THREAD && !flag_init_proc_thread_) {
        flag_init_proc_thread_ = true;
        process_thread_ = std::thread(&VinsEstimator::processMeasurements, this);
        LOG(INFO) << "[Params] seperate thread (proc meas) inited.";
    }
    mtxProc.unlock();
}

void VinsEstimator::changeSensorType(int use_imu, int use_stereo) {
    bool restart = false;
    mtxProc.lock();
    if(!use_imu && !use_stereo) {
        LOG(ERROR) << "[change sensor type] at least use two sensors!";
    }
    else {
        if(USE_IMU != use_imu) {
            USE_IMU = use_imu;
            if(USE_IMU) {
                // reuse imu; restart system
                LOG(INFO) << "need to restart system to reuse imu.";
                restart = true;
            }
            else {
                if (last_margnlztn_manager_ != nullptr) {
                    delete last_margnlztn_manager_;
                }
                tmp_pre_integration_ = nullptr;
                last_margnlztn_manager_ = nullptr;
                last_margnlztn_param_blocks_.clear();
            }
        }
        
        STEREO = use_stereo;
        LOG(INFO) << "[change sensor type] use imu:" << USE_IMU << ", use stereo:" << STEREO;
    }
    mtxProc.unlock();
    if(restart) {
        LOG(INFO) << "[change sensor type] restart VINS system.";
        clearState();
        setParameter();
    }
}

void VinsEstimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) {
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

void VinsEstimator::inputIMU(double t, const Vector3d &linearAcc, const Vector3d &angularVel) {
    mtxBuf.lock();
    accBuf.push(make_pair(t, linearAcc));
    gyrBuf.push(make_pair(t, angularVel));
    mtxBuf.unlock();

    // 进入常规VINS阶段后，每进一次IMU都要即时递推之
    if (flag_solver_type_ == NON_LINEAR) {
        mtxPropagate.lock();
        fastPredictIMU(t, linearAcc, angularVel);
        // pubLatestOdometry(latest_P_, latest_Q_, latest_V_, t);
        if (handle_latest_odom_) {
            handle_latest_odom_(latest_P_, latest_Q_, latest_V_, t);
        }
        mtxPropagate.unlock();
    }
}

void VinsEstimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1) {
    /* 图像预处理，特征追踪，入缓存队列 */

    num_input_image_++;
    TicToc f_track_timer;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> frame_feats;

    if(_img1.empty()) /*单目*/ {
        frame_feats = f_tracker_.trackImage(t, _img);
    }
    else /*双目*/ {
        frame_feats = f_tracker_.trackImage(t, _img, _img1);
    }
    const double tracking_usage = f_track_timer.toc();
    curr_tracking_usage_.store(tracking_usage);
    LOG(INFO) << "[InputImgs] count=" << num_input_image_ 
        << ", tracking feats took " << tracking_usage << "ms >>>>>>|";

    if (SHOW_TRACK && handle_track_image_) /*UI显示*/ {
        cv::Mat imgTrack = f_tracker_.getTrackImage();
        // pubTrackImage(imgTrack, t);
        handle_track_image_(imgTrack, t);
    }
    ui_show_imgLK_usage_.store(f_track_timer.toc() - tracking_usage);
    
    if(MULTIPLE_THREAD) {
        // 多线程下，异步运行算法（跳帧？）
        if(num_input_image_ % 2 == 0) {
            mtxBuf.lock();
            featureBuf.push(make_pair(t, frame_feats));
            mtxBuf.unlock();
        }
    }
    else {
        // 单线程下，主线程同步运行算法
        mtxBuf.lock();
        featureBuf.push(make_pair(t, frame_feats));
        mtxBuf.unlock();
        processMeasurements();
    }
    
}

void VinsEstimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &frame_feats) {
    mtxBuf.lock();
    featureBuf.push(make_pair(t, frame_feats));
    mtxBuf.unlock();

    if(!MULTIPLE_THREAD) {
        processMeasurements();
    }
}

void VinsEstimator::processMeasurements() {
    while (1) {
        if (flag_proc_exit_.load()) {
            LOG(INFO) << "proc measurements loop terminating ... ";
            break;
        }
        // 开始处理一组measurement
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> f_frame;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty()) {
            TicToc proc_meas_timer;
            f_frame = featureBuf.front();
            curr_time_ = f_frame.first + Tdiff_;

            // 1. 准备数据
            while(1) {
                if ((!USE_IMU || IMUAvailable(f_frame.first + Tdiff_))) {
                    break;
                }
                else {
                    LOG(INFO) << "waiting for imu ... ";
                    if (! MULTIPLE_THREAD) {
                        return;
                    }
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                    if (flag_proc_exit_.load()) {
                        LOG(INFO) << "proc meas loop terminating ... ";
                        return;
                    }
                }
            }
            mtxBuf.lock();
            if(USE_IMU) {
                getIMUInterval(prev_time_, curr_time_, accVector, gyrVector);
            }
            featureBuf.pop();
            mtxBuf.unlock();

            // 2. IMU积分递推+更新预测
            if(USE_IMU) {
                if(!flag_init_1st_imu_pose_) {
                    initFirstIMUPose(accVector);
                }
                for(size_t i = 0; i < accVector.size(); i++) {
                    double dt;
                    if(i == 0) {
                        dt = accVector[i].first - prev_time_;
                    }
                    else if (i == accVector.size() - 1) {
                        dt = curr_time_ - accVector[i - 1].first;
                    }
                    else {
                        dt = accVector[i].first - accVector[i - 1].first;
                    }
                    //积分递推一次，并更新最新帧状态量（预测更新）
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }

            // 3. 完成一次处理
            mtxProc.lock();
            processImage(f_frame.second, f_frame.first);
            double t_step = curr_time_ - prev_time_;
            prev_time_ = curr_time_;

            // 4. 向外输出结果
            TicToc ui_show_timer;
            curr_sys_state_time_ = f_frame.first;
            // printStatistics(*this, 0);
            if (handle_statistics_) {
                handle_statistics_(*this, 0);
            }
            // std_msgs::Header ros_header;
            // ros_header.frame_id = "world";
            // ros_header.stamp = ros::Time(f_frame.first);
            // pubOdometry(*this); //, ros_header);
            // pubKeyPoses(*this); //, ros_header);
            // pubCameraPose(*this); //, ros_header);
            // pubPointCloud(*this); //, ros_header);
            // pubKeyframe(*this);
            // pubTF(*this); //, ros_header);
            if (handle_vio_states_) {
                handle_vio_states_(*this);
            }
            double ui_usage = ui_show_timer.toc();
            mtxProc.unlock();

            // 5. 统计运行时效和日志
            static int proc_counts = 0;
            static double recent_tstep = 0;
            static double recent_usage = 0;
            double curr_usage = proc_meas_timer.toc();
            curr_procmeas_usage_.store(curr_usage);
            ui_show_state_usage_.store(ui_usage);
            double freq = 0.0; 
            proc_counts++;
            if (proc_counts <= 2) {
                recent_usage = curr_usage;
                recent_tstep = t_step;
                freq = -1;
            } else {
                recent_usage = 0.3 * curr_usage + 0.7 * recent_usage;
                recent_tstep = 0.3 * (t_step) + 0.7 * recent_tstep;
                freq = 1.0 / recent_tstep;
            }
            LOG(INFO) << "[ProcMeasures] count=" << proc_counts << ", working freq " << freq 
                << "Hz, took " << curr_usage << "ms, ave " << recent_usage << "ms >>>>>>||";
        }

        if (!MULTIPLE_THREAD) {
            // 单线程模式下，外部主动逐次调用
            break;
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

// ========================== internals: pipeline funcs ========================== //

void VinsEstimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector) {
    /* 描述：在静止假设下，结合重力方向，初始化第一个姿态 */
    LOG(INFO) << "[ Estimator | InitWithImu ] init first imu pose ... ";
    flag_init_1st_imu_pose_ = true;
    Eigen::Vector3d aveAcc(0, 0, 0);
    const int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++) {
        aveAcc = aveAcc + accVector[i].second;
    }
    aveAcc = aveAcc / n;
    printf("[ Estimator | InitWithImu ] with %d imus, ave acc [%f, %f, %f]\n", 
        n, aveAcc.x(), aveAcc.y(), aveAcc.z());

    Matrix3d R0 = Utility::g2R(aveAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    LOG(INFO) << "init R0 (first pose rotation) as:\n" << Rs[0];
    //Vs[0] = Vector3d(5, 0, 0);
}

void VinsEstimator::processIMU(double t, double dt, const Vector3d &_linearAcc, const Vector3d &_angularVelo) {
    /* 递推一次常规IMU积分（作为最新帧预测）, 以及递推IMU预积分 */
    if (!first_first_imu_) {
        first_first_imu_ = true;
        acc_0 = _linearAcc;
        gyr_0 = _angularVelo;
    }

    if (!pre_integrations_[frame_count]) {
        pre_integrations_[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    // 只要滑窗已经初始化了，就用imu更新滑窗最新帧状态【也即预测】
    if (frame_count != 0) {
        tmp_pre_integration_->push_back(dt, _linearAcc, _angularVelo);

        pre_integrations_[frame_count]->push_back(dt, _linearAcc, _angularVelo);
        dt_buff_[frame_count].push_back(dt);
        acc_buff_[frame_count].push_back(_linearAcc);
        gyr_buff_[frame_count].push_back(_angularVelo);

        //去bias
        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - Grav_;
        Vector3d un_gyr = 0.5 * (gyr_0 + _angularVelo) - Bgs[j];
        //更新预测：姿态
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (_linearAcc - Bas[j]) - Grav_;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        //更新预测：位置和速度
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = _linearAcc;
    gyr_0 = _angularVelo; 
}

void VinsEstimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_img_feats, const double _img_time) {
    /** 这个函数的逻辑是： VINS算法pipeline：VIO初始化，VIO常规运行
     * 
    */

    LOG(INFO) << "[ProcImage] Adding feature count:" << _img_feats.size();

    // 检查视差大小，决定是否为关键帧
    if (f_manager_.addFeatureCheckParallax(frame_count, _img_feats, Tdiff_)) {
        flag_marglize_type_ = MARGIN_OLD;
        //LOG(INFO) << "[ProcImage] new keyframe, will slide out oldest ";
    }
    else {
        flag_marglize_type_ = MARGIN_SECOND_NEW;
        //LOG(INFO) << "[ProcImage] non-keyframe, will slide out second-newest ";
    }

    LOG(INFO) << "[ProcImage] Solving (frame count): " << frame_count;
    LOG(INFO) << "[ProcImage] number of robust features: " << f_manager_.getRobustFeatureCount();

    Headers[frame_count] = _img_time;
    ImageFrame cur_img_frame(_img_feats, _img_time);
    cur_img_frame.pre_integration = tmp_pre_integration_;
    all_image_frame_.insert(make_pair(_img_time, cur_img_frame));

    // 这个临时预积分器重置为新的，用于下一帧图像的预积分
    tmp_pre_integration_ = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2) /* no extrinsic prior at all, calib rotation! */ {
        LOG(INFO) << "[ SelfCalib ] ProcImage: calibrating extrinsic param, rotation movement is needed! ";
        if (frame_count != 0) {
            Matrix3d calibed_Rot_IC;
            vector<pair<Vector3d, Vector3d>> corres = f_manager_.getCorresponding(frame_count - 1, frame_count);
            if (init_extrin_rot_.CalibrationExRotation(corres, pre_integrations_[frame_count]->delta_q, calibed_Rot_IC)) {
                LOG(INFO) << "[ SelfCalib ] ProcImage: init extrinsic rotation succeed.";
                LOG(INFO) << "[ SelfCalib ] ProcImage: calibrated extrinsic rotation: \n" << calibed_Rot_IC;
                R_IC_[0] = calibed_Rot_IC;
                RIC[0] = calibed_Rot_IC;
                ESTIMATE_EXTRINSIC = 1; /* optimize around prior. */
            }
        }
    }

    if (flag_solver_type_ == INITIAL) {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU) {
            if (frame_count == WINDOW_SIZE) {
                bool success = false;
                if(ESTIMATE_EXTRINSIC != 2 && (_img_time - initial_timestamp) > 0.1) {
                    success = initialStructure();
                    initial_timestamp = _img_time;   
                }
                if(success) {
                    runOptimization();
                    updateLatestStates();
                    flag_solver_type_ = NON_LINEAR;
                    slideWindow();
                    LOG(INFO) << "ProcImage: Initialization finish! ";
                }
                else {
                    slideWindow();
                }
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU) {
            f_manager_.initFramePoseByPnP(frame_count, Ps, Rs, T_IC_, R_IC_);
            f_manager_.triangulate(frame_count, Ps, Rs, T_IC_, R_IC_);
            if (frame_count == WINDOW_SIZE) {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame_.begin(); frame_it != all_image_frame_.end(); frame_it++) {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame_, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++) {
                    pre_integrations_[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                runOptimization();
                updateLatestStates();
                flag_solver_type_ = NON_LINEAR;
                slideWindow();
                LOG(INFO) << "ProcImage: Initialization finish! ";
            }
        }

        // stereo-only initilization
        if(STEREO && !USE_IMU) {
            f_manager_.initFramePoseByPnP(frame_count, Ps, Rs, T_IC_, R_IC_);
            f_manager_.triangulate(frame_count, Ps, Rs, T_IC_, R_IC_);
            runOptimization();
            if(frame_count == WINDOW_SIZE) {
                runOptimization();
                updateLatestStates();
                flag_solver_type_ = NON_LINEAR;
                slideWindow();
                LOG(INFO) << "ProcImage: Initialization finish! ";
            }
        }

        if(frame_count < WINDOW_SIZE) {
            frame_count++; /* 唯一改变取值的地方，意味着索引的最大取值正是WIN_SIZE */
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else /*常规VIO流程*/ {
        TicToc t_solve_vio;

        //step01：滑窗最新帧状态量已经是IMU积分预测了，用预测位姿来初始化新feature的深度
        if(!USE_IMU) {
            /*特例：若未使用IMU，用sfm初始化feature深度+粗略估算位姿 */
            f_manager_.initFramePoseByPnP(frame_count, Ps, Rs, T_IC_, R_IC_);
        }
        f_manager_.triangulate(frame_count, Ps, Rs, T_IC_, R_IC_);

        //step02：执行滑窗BA优化
        runOptimization();

        //step03：移除外点（重投影误差大于阈值的feature）
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager_.removeOutlier(removeIndex);
        if (! MULTIPLE_THREAD) {
            /** XXX: 如果同步线程才启用？暂不太理解 */
            f_tracker_.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
        LOG(INFO) << "[ProcImage] solve vio took " << t_solve_vio.toc() << "ms";

        //step04：监控系统失败
        if (failureDetection()) {
            LOG(INFO) << "[ WARN ] ProcImage: failure detection! ";
            flag_failure_occur_ = 1;
            clearState();
            setParameter();
            LOG(INFO) << "[ WARN ] ProcImage: system reboot! ";
            return;
        }

        //step05：滑动一次
        slideWindow();
        f_manager_.removeFailures();

        //step06：prepare output of VINS
        key_poses_.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            key_poses_.push_back(Ps[i]);
        }

        last_Rn = Rs[WINDOW_SIZE];
        last_Pn = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }  
}

bool VinsEstimator::initialStructure() {
    TicToc t_sfm;

    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame_.begin(), frame_it++; frame_it != all_image_frame_.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame_.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame_.begin(), frame_it++; frame_it != all_image_frame_.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // LOG(INFO) << "frame g " << tmp_g.transpose();
        }
        var = sqrt(var / ((int)all_image_frame_.size() - 1));
        //printf("[ WARN ] IMU variation %f! \n", var);
        if(var < 0.25) {
            LOG(INFO) << "IMU excitation not enouth! ";
            //return false;
        }
    }

    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager_.features_) {
        int frame_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            frame_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(frame_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int frame_Seq;
    if (!searchRelativePose(relative_R, relative_T, frame_Seq)) {
        LOG(INFO) << "Not enough features or parallax; Move device around ";
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, frame_Seq,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points)) {
        // LOG(INFO) << "[ DBG ] global SFM failed! ";
        flag_marglize_type_ = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame_.begin( );
    for (int i = 0; frame_it != all_image_frame_.end( ); frame_it++) {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i]) {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i]) {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points) {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second) {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end()) {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6) {
            LOG(INFO) << "pts_3_vector size " << pts_3_vector.size();
            // LOG(INFO) << "[ DBG ] Not enough points for solve pnp! ";
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
            // LOG(INFO) << "[ DBG ] solve pnp fail! ";
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else {
        LOG(INFO) << "misalign visual structure with IMU ";
        return false;
    }

}

bool VinsEstimator::searchRelativePose(Matrix3d &_relativeR, Vector3d &_relativeT, int &_frameSeq) {
    // find previous frame which contians enough correspondance and parallex with newest frame

    /** 根据作者注释：找到【和最新帧具有足够数量特征关联、以及明显视差】的previous帧
     * 所以，函数名 relativePose 是啥意思？ 
     * UPDATE: 虽然我明白了，但是吐槽一下这个命名真不友好。
    */

    // 所有滑窗帧，逐一与最新帧计算相对位姿，并返回计算成功的帧的索引，和相对位姿结果
    for (int i = 0; i < WINDOW_SIZE; i++) {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager_.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20) {
            double sum_parallax = 0;
            double ave_parallax = 0;
            for (int j = 0; j < int(corres.size()); j++) {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            ave_parallax = 1.0 * sum_parallax / int(corres.size());

            /// TODO: 下边这个460，应该替换成系统常量参数 FOCAL_LENGTH 
            if (ave_parallax * 460 > 30 && motion_estimator_.solveRelativeRT(corres, _relativeR, _relativeT)) {
                _frameSeq = i;
                // printf("[ DBG ] ave_parallax %f choose frame %d and newest frame to triAngulate the whole structure \n", 
                //     ave_parallax * 460, _frameSeq);
                // LOG(INFO) << "ave parallax " << ave_parallax << ", choose frame " 
                //     << _frameSeq << " and newest frame to triAngulate the whole structure.";
                return true;
            }
        }
    }
    return false;
}

bool VinsEstimator::visualInitialAlign() {
    /**
     * 视觉和惯性的对其,对应崔华坤大佬文章(https://mp.weixin.qq.com/s/9twYJMOE8oydAzqND0UmFw)中的同名部分
     * 根据VIO课程第七讲，一共分为5步：
     * 1.估计旋转外参 
     * 2.估计陀螺仪bias 
     * 3.估计中立方向,速度.尺度初始值 
     * 4.对重力加速度进一步优化 
     * 5.将轨迹对其到世界坐标系 
    */

    // TicToc t_g;
    VectorXd x;
    //solve scale
    bool success = VisualIMUAlignment(all_image_frame_, Bgs, Grav_, x/*output*/);
    if(!success) {
        // LOG(INFO) << "[ DBG ] solve g failed! ";
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++) {
        Matrix3d Ri = all_image_frame_[Headers[i]].R;
        Vector3d Pi = all_image_frame_[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame_[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations_[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame_.begin(); frame_i != all_image_frame_.end(); frame_i++) {
        if(frame_i->second.is_key_frame) {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(Grav_);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Grav_ = R0 * Grav_;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++) {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    LOG(INFO) << "g0     " << Grav_.transpose();
    LOG(INFO) << "my R0  " << Utility::R2ypr(Rs[0]).transpose();

    f_manager_.clearAllFeatureDepth();
    f_manager_.triangulate(frame_count, Ps, Rs, T_IC_, R_IC_);

    return true;
}

void VinsEstimator::runOptimization() {
    /** 这个函数的逻辑是：
     * 
    */

    TicToc t_whole_optimz, t_prepare_optimz;
    StateToCeresParam();

    // 优化问题对象
    ceres::Problem problem;

    // 指定鲁邦核函数
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);

    // ############################## 开始添加各种参数块，也即优化变量 ##############################

    // 首先是滑窗帧pose和bias到优化变量
    for (int i = 0; i < frame_count + 1; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose_[i], SIZE_POSE, local_parameterization);
        if(USE_IMU) {
            problem.AddParameterBlock(para_Speed_Bias_[i], SIZE_SPEEDBIAS);
        }
    }

    // 如果未启用imu，则固定第一帧的pose 
    /// TODO: 补充这么做的原因？
    if(!USE_IMU) {
        problem.SetParameterBlockConstant(para_Pose_[0]);
    }

    // 添加外参到优化变量，若未启用外参估计或激励不足，则设为常量
    for (int i = 0; i < NUM_OF_CAM; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose_[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || flag_open_extrin_esti_) {
            // LOG(INFO) << "estimate extinsic param ";
            flag_open_extrin_esti_ = 1;
        }
        else {
            // LOG(INFO) << "frozen extinsic param ";
            problem.SetParameterBlockConstant(para_Ex_Pose_[i]);
        }
    }

    // 添加sensor间时延到优化变量，若未启用或激励不足，则设为常量
    problem.AddParameterBlock(para_Td_[0], 1);
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2) {
        problem.SetParameterBlockConstant(para_Td_[0]);
    }

    // ############################## 开始添加各种残差块，也即约束 ##############################

    // 添加边缘化约束，若存在的话
    if (last_margnlztn_manager_ && last_margnlztn_manager_->valid) {
        // construct new marginlization factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_margnlztn_manager_);
        problem.AddResidualBlock(marginalization_factor, NULL, last_margnlztn_param_blocks_);
    }

    // 添加imu预积分约束，若存在的话
    if(USE_IMU) {
        for (int i = 0; i < frame_count; i++) {
            int j = i + 1;
            if (pre_integrations_[j]->sum_dt > 10.0) {continue;}
            IMUFactor* imu_factor = new IMUFactor(pre_integrations_[j]);
            problem.AddResidualBlock(imu_factor, NULL, 
                para_Pose_[i], para_Speed_Bias_[i], para_Pose_[j], para_Speed_Bias_[j]);
        }
    }

    // 添加特征点的重投影约束【重点】
    int f_cnt = 0, f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager_.features_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) {continue;} /*跟踪次数过少的feature不予添加(不稳定)*/

        f_cnt++;
        ++feature_index;
        int frame_i = it_per_id.start_frame, frame_j = frame_i - 1; /*原名imu_i/imu_j，大概是粘过来的代码*/
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            frame_j++;
            if (frame_i != frame_j) {
                /*添加一次（左目的）帧间重投影误差*/
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *visual_factor = new ProjectionTwoFrameOneCamFactor(
                    pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(visual_factor, loss_function, 
                                        para_Pose_[frame_i], para_Pose_[frame_j], para_Ex_Pose_[0], 
                                        para_Features_[feature_index], para_Td_[0]);
            }

            if(STEREO && it_per_frame.is_stereo) {
                Vector3d pts_j_right = it_per_frame.pointRight;
                /*添加一次（双目的）帧间重投影误差*/
                if(frame_i != frame_j) {
                    ProjectionTwoFrameTwoCamFactor *visual_factor = new ProjectionTwoFrameTwoCamFactor(
                        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(visual_factor, loss_function, 
                                            para_Pose_[frame_i], para_Pose_[frame_j], para_Ex_Pose_[0], para_Ex_Pose_[1], 
                                            para_Features_[feature_index], para_Td_[0]);
                }
                /*添加一次同一帧（左右目）之间的重投影误差*/
                else {
                    ProjectionOneFrameTwoCamFactor *visual_factor = new ProjectionOneFrameTwoCamFactor(
                        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(visual_factor, loss_function, 
                                            para_Ex_Pose_[0], para_Ex_Pose_[1], 
                                            para_Features_[feature_index], para_Td_[0]);
                }
               
            }
            f_m_cnt++;
        }
    }

    num_optimz_tracked_feats_.store(f_cnt);
    num_optimz_tracked_times_.store(f_m_cnt);
    // printf("[ DBG ] visual measurement count: %d \n", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare_optimz.toc());

    // ############################## 设置求解参数 ##############################

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (flag_marglize_type_ == MARGIN_OLD) {
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    }
    else {
        options.max_solver_time_in_seconds = SOLVER_TIME;
    }

    // ############################## 求解优化问题 ##############################

    TicToc t_ceres_solve;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // LOG(INFO) << summary.BriefReport();
    // printf("[ DBG ] Iterations : %d \n", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_ceres_solve.toc());

    /// HERE: 保存优化结果到状态量
    CeresParamToState();

    //printf("frame_count: %d \n", frame_count);

    // ############################## 执行边缘化，计算新的边缘化因子 ##############################

    // 滑窗未满时，无需边缘化
    if(frame_count < WINDOW_SIZE) { return; }

    // 边缘化最老帧或次新帧
    TicToc t_whole_marginlz;

    if (flag_marglize_type_ == MARGIN_OLD) /*边缘化最老帧*/ {
        /*用于收集边缘化所需要的全部信息（被边缘化帧及其关联的帧、特征、因子），而后执行边缘化，获得新的边缘化因子，用于下一轮滑窗优化*/
        MarginalizationManager *curr_margnlztn_manager = new MarginalizationManager();

        /*因为边缘化操作需要访问当前状态量的取值（线性化点），因此先更新状态量到ceres参数块中*/
        StateToCeresParam();

        //上一轮的边缘化因子，若与当前最老帧相关联，则需要被收集起来
        if (last_margnlztn_manager_ && last_margnlztn_manager_->valid) {
            vector<int> drop_set; /*标识需要被边缘化掉的参数的索引*/
            for (int i = 0; i < static_cast<int>(last_margnlztn_param_blocks_.size()); i++) {
                /*以下，若上一轮边缘化因子的关联状态量里有当前最老帧，则drop_set非空，意味着需要把上一轮边缘化因子纳入本轮边缘化*/
                /*如果上一轮边缘化的是次新帧，则大概率条件不满足而drop_set为空，于是在本轮边缘化中不做处理，这符合逻辑*/
                if (last_margnlztn_param_blocks_[i] == para_Pose_[0] /*地址相等*/ ||
                    last_margnlztn_param_blocks_[i] == para_Speed_Bias_[0]) {
                    drop_set.push_back(i);
                }
            }
            // add previous margnliz factor to current margnliz process
            MarginalizationFactor *cost_func = new MarginalizationFactor(last_margnlztn_manager_);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                cost_func, NULL, last_margnlztn_param_blocks_, drop_set);
            curr_margnlztn_manager->addResidualBlockInfo(residual_block_info);
        }

        //与最老帧关联的imu预积分因子，需要被收集起来
        if(USE_IMU) {
            if (pre_integrations_[1]->sum_dt < 10.0) {
                IMUFactor* cost_func = new IMUFactor(pre_integrations_[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                    cost_func, NULL/*loss func*/,
                    vector<double*>{para_Pose_[0], para_Speed_Bias_[0], 
                                    para_Pose_[1], para_Speed_Bias_[1]},
                    vector<int>{0, 1}/*drop set*/);
                curr_margnlztn_manager->addResidualBlockInfo(residual_block_info);
            }
        }

        //最老帧持有的特征，若被其它帧看到，这种共视约束需要被收集起来
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager_.features_) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4){
                    continue; /*不稳定特征不纳入边缘化，跳过*/
                }

                ++feature_index;
                int frame_i = it_per_id.start_frame, frame_j = frame_i - 1;
                if (frame_i != 0) {
                    continue; /*不被当前最老帧持有，跳过*/
                }

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                for (auto &it_per_frame : it_per_id.feature_per_frame) {
                    frame_j++;
                    /*将（左目）共视帧约束纳入边缘化*/
                    if(frame_i != frame_j) {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *cost_func = new ProjectionTwoFrameOneCamFactor(
                            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                            cost_func, loss_function,
                            vector<double*>{para_Pose_[frame_i], para_Pose_[frame_j], 
                                para_Ex_Pose_[0], para_Features_[feature_index], para_Td_[0]},
                            vector<int>{0, 3}/*drop set*/);
                        curr_margnlztn_manager->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo) {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        /*将（双目）共视帧约束纳入边缘化*/
                        if(frame_i != frame_j) {
                            ProjectionTwoFrameTwoCamFactor *cost_func = new ProjectionTwoFrameTwoCamFactor(
                                pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                                cost_func, loss_function,
                                vector<double*>{para_Pose_[frame_i], para_Pose_[frame_j], 
                                    para_Ex_Pose_[0], para_Ex_Pose_[1], 
                                    para_Features_[feature_index], para_Td_[0]},
                                vector<int>{0, 4}/*drop set*/);
                            curr_margnlztn_manager->addResidualBlockInfo(residual_block_info);
                        }
                        /*将（左右目）之间的共视约束纳入边缘化*/
                        else {
                            ProjectionOneFrameTwoCamFactor *cost_func = new ProjectionOneFrameTwoCamFactor(
                                pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                                cost_func, loss_function,
                                vector<double*>{para_Ex_Pose_[0], para_Ex_Pose_[1], para_Features_[feature_index], para_Td_[0]},
                                vector<int>{2}/*drop set*/);
                            curr_margnlztn_manager->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        curr_margnlztn_manager->preMarginalize();
        // printf("[ DBG ] pre marginalization %f ms \n", t_pre_margin.toc());
        
        TicToc t_margin;
        curr_margnlztn_manager->marginalize();
        // printf("[ DBG ] marginalization %f ms \n", t_margin.toc());

        // 丢掉最老帧之后，状态量位置索引需要重置
        std::unordered_map<long, double*> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose_[i])] = para_Pose_[i - 1];
            if(USE_IMU) {
                addr_shift[reinterpret_cast<long>(para_Speed_Bias_[i])] = para_Speed_Bias_[i - 1];
            }
        }
        for (int i = 0; i < NUM_OF_CAM; i++) {
            addr_shift[reinterpret_cast<long>(para_Ex_Pose_[i])] = para_Ex_Pose_[i];
        }
        addr_shift[reinterpret_cast<long>(para_Td_[0])] = para_Td_[0];//TODO:这句话是啥意思？

        vector<double*> parameter_blocks = curr_margnlztn_manager->getParameterBlocks(addr_shift);

        // 保存新的边缘化因子，用于下一轮滑窗优化
        if (last_margnlztn_manager_) {
            delete last_margnlztn_manager_;
        }
        last_margnlztn_manager_ = curr_margnlztn_manager;
        last_margnlztn_param_blocks_ = parameter_blocks;

    }
    else /*边缘化次新帧*/ {
        if (last_margnlztn_manager_ && 
            std::count(std::begin(last_margnlztn_param_blocks_), 
                std::end(last_margnlztn_param_blocks_), para_Pose_[WINDOW_SIZE - 1])) {
            /*用于收集边缘化所需要的全部信息（被边缘化帧及其关联的帧、特征、因子），而后执行边缘化，获得新的边缘化因子，用于下一轮滑窗优化*/
            MarginalizationManager *curr_margnlztn_manager = new MarginalizationManager();

            /*因为边缘化操作需要访问当前状态量的取值（线性化点），因此先更新状态量到ceres参数块中*/
            StateToCeresParam();

            // 同上，不再重复注释
            if (last_margnlztn_manager_ && last_margnlztn_manager_->valid) {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_margnlztn_param_blocks_.size()); i++) {
                    assert(last_margnlztn_param_blocks_[i] != para_Speed_Bias_[WINDOW_SIZE - 1]);
                    if (last_margnlztn_param_blocks_[i] == para_Pose_[WINDOW_SIZE - 1]) {
                        drop_set.push_back(i);
                    }
                }
                // add previous margnliz factor to current margnliz process
                MarginalizationFactor *cost_func = new MarginalizationFactor(last_margnlztn_manager_);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                    cost_func, NULL, last_margnlztn_param_blocks_, drop_set);
                curr_margnlztn_manager->addResidualBlockInfo(residual_block_info);
            }

            /// NOTE: 边缘次新帧时，特征观测和相关约束直接丢掉；因此不像边缘化最老帧时那么麻烦
            // 另外，预积分器不重置，接着预积分即可，相当于imu因子的边缘化已经被考虑了

            TicToc t_pre_margin;
            curr_margnlztn_manager->preMarginalize();
            // printf("[ DBG ] end pre marginalization, %f ms \n", t_pre_margin.toc());

            TicToc t_margin;
            curr_margnlztn_manager->marginalize();
            // printf("[ DBG ] end marginalization, %f ms \n", t_margin.toc());

            // 因为丢掉了次新帧，所以状态量位置索引需要重置
            std::unordered_map<long, double*> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                if (i == WINDOW_SIZE - 1) {
                    continue;
                }
                else if (i == WINDOW_SIZE) {
                    addr_shift[reinterpret_cast<long>(para_Pose_[i])] = para_Pose_[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_Speed_Bias_[i])] = para_Speed_Bias_[i - 1];
                }
                else {
                    addr_shift[reinterpret_cast<long>(para_Pose_[i])] = para_Pose_[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_Speed_Bias_[i])] = para_Speed_Bias_[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++) {
                addr_shift[reinterpret_cast<long>(para_Ex_Pose_[i])] = para_Ex_Pose_[i];
            }
            addr_shift[reinterpret_cast<long>(para_Td_[0])] = para_Td_[0];

            vector<double*> parameter_blocks = curr_margnlztn_manager->getParameterBlocks(addr_shift);

            // 保存新的边缘化因子，用于下一轮滑窗优化
            if (last_margnlztn_manager_) {
                delete last_margnlztn_manager_; 
            }
            last_margnlztn_manager_ = curr_margnlztn_manager;
            last_margnlztn_param_blocks_ = parameter_blocks;

        }
    }

    //printf("whole marginalization costs: %f \n", t_whole_marginlz.toc());
    //printf("whole time for ceres: %f \n", t_whole_optimz.toc());
}

void VinsEstimator::slideWindow() {
    // TicToc t_margin;
    if (flag_marglize_type_ == MARGIN_OLD) {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE) {
            for (int i = 0; i < WINDOW_SIZE; i++) {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU) {
                    std::swap(pre_integrations_[i], pre_integrations_[i + 1]);

                    dt_buff_[i].swap(dt_buff_[i + 1]);
                    acc_buff_[i].swap(acc_buff_[i + 1]);
                    gyr_buff_[i].swap(gyr_buff_[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU) {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations_[WINDOW_SIZE];
                pre_integrations_[WINDOW_SIZE] = new IntegrationBase{
                    acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buff_[WINDOW_SIZE].clear();
                acc_buff_[WINDOW_SIZE].clear();
                gyr_buff_[WINDOW_SIZE].clear();
            }

            if (true || flag_solver_type_ == INITIAL) {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame_.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame_.erase(all_image_frame_.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else {
        if (frame_count == WINDOW_SIZE) {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU) {
                for (unsigned int i = 0; i < dt_buff_[frame_count].size(); i++) {
                    double tmp_dt = dt_buff_[frame_count][i];
                    Vector3d tmp_linear_acc = acc_buff_[frame_count][i];
                    Vector3d tmp_angular_velo = gyr_buff_[frame_count][i];

                    pre_integrations_[frame_count - 1]->push_back(tmp_dt, tmp_linear_acc, tmp_angular_velo);

                    dt_buff_[frame_count - 1].push_back(tmp_dt);
                    acc_buff_[frame_count - 1].push_back(tmp_linear_acc);
                    gyr_buff_[frame_count - 1].push_back(tmp_angular_velo);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations_[WINDOW_SIZE];
                pre_integrations_[WINDOW_SIZE] = new IntegrationBase{
                    acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buff_[WINDOW_SIZE].clear();
                acc_buff_[WINDOW_SIZE].clear();
                gyr_buff_[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void VinsEstimator::outliersRejection(set<int> &removeIndex) {
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager_.features_) {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int frame_i = it_per_id.start_frame, frame_j = frame_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            frame_j++;
            if (frame_i != frame_j) {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[frame_i], Ps[frame_i], R_IC_[0], T_IC_[0], 
                                                    Rs[frame_j], Ps[frame_j], R_IC_[0], T_IC_[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor
            if(STEREO && it_per_frame.is_stereo) {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(frame_i != frame_j) {
                    double tmp_error = reprojectionError(Rs[frame_i], Ps[frame_i], R_IC_[0], T_IC_[0], 
                                                        Rs[frame_j], Ps[frame_j], R_IC_[1], T_IC_[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else {
                    double tmp_error = reprojectionError(Rs[frame_i], Ps[frame_i], R_IC_[0], T_IC_[0], 
                                                        Rs[frame_j], Ps[frame_j], R_IC_[1], T_IC_[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }

        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3) {
            removeIndex.insert(it_per_id.feature_id);
        }
    }
}

void VinsEstimator::predictPtsInNextFrame() {
    //LOG(INFO) << "predict pts in next frame ";
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager_.features_) {
        if(it_per_id.estimated_depth > 0) {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count) {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = R_IC_[0] * (depth * it_per_id.feature_per_frame[0].point) + T_IC_[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = R_IC_[0].transpose() * (pts_local - T_IC_[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }

    f_tracker_.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

bool VinsEstimator::failureDetection() {
    return false; //似乎作者关闭了这个检测，所以这个函数我们暂且不深究

    if (f_manager_.last_track_num_ < 2) {
        printf(" too few features ( %d) to work, system failure. \n", f_manager_.last_track_num_);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5) {
        printf(" big IMU acc bias estimation %f \n", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0) {
        printf(" big IMU gyr bias estimation %f \n", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (T_IC_(0) > 1) {
        printf(" big extri param estimation %d \n", T_IC_(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_Pn).norm() > 5) {
        //LOG(INFO) << " big translation ";
        //return true;
    }
    if (abs(tmp_P.z() - last_Pn.z()) > 1) {
        //LOG(INFO) << " big z translation ";
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_Rn;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50) {
        LOG(INFO) << " big delta_angle  ";
        //return true;
    }
    return false;
}

void VinsEstimator::updateLatestStates() {
    mtxPropagate.lock();
    latest_time_ = Headers[frame_count] + Tdiff_;
    latest_P_ = Ps[frame_count];
    latest_Q_ = Rs[frame_count];
    latest_V_ = Vs[frame_count];
    latest_Ba_ = Bas[frame_count];
    latest_Bg_ = Bgs[frame_count];
    latest_Acc_0_ = acc_0;
    latest_Gyr_0_ = gyr_0;
    mtxBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mtxBuf.unlock();
    while(!tmp_accBuf.empty()) {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mtxPropagate.unlock();
}

void VinsEstimator::fastPredictIMU(double t, Eigen::Vector3d _linearAcc, Eigen::Vector3d _angularVelo) {
    double dt = t - latest_time_;
    latest_time_ = t;
    Eigen::Vector3d un_acc_0 = latest_Q_ * (latest_Acc_0_ - latest_Ba_) - Grav_;
    Eigen::Vector3d un_gyr = 0.5 * (latest_Gyr_0_ + _angularVelo) - latest_Bg_;
    latest_Q_ = latest_Q_ * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q_ * (_linearAcc - latest_Ba_) - Grav_;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P_ = latest_P_ + dt * latest_V_ + 0.5 * dt * dt * un_acc;
    latest_V_ = latest_V_ + dt * un_acc;
    latest_Acc_0_ = _linearAcc;
    latest_Gyr_0_ = _angularVelo;
}

// ========================== internals: tool funcs ========================== //

bool VinsEstimator::IMUAvailable(double t) {
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

bool VinsEstimator::getIMUInterval(double t0, double t1, 
    vector<pair<double, Eigen::Vector3d>> &accVector, 
    vector<pair<double, Eigen::Vector3d>> &gyrVector) {
    if(accBuf.empty()) {
        LOG(INFO) << "imu buffer empty, no imu received. ";
        return false;
    }
    //printf("get imu from time interval: %f %f\n", t0, t1);
    //printf("current imu buffer front time %f, end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first) {
        while (accBuf.front().first <= t0) {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1) {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else {
        LOG(INFO) << "wait for imu ... ";
        return false;
    }
    return true;
}

void VinsEstimator::StateToCeresParam() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        para_Pose_[i][0] = Ps[i].x();
        para_Pose_[i][1] = Ps[i].y();
        para_Pose_[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose_[i][3] = q.x();
        para_Pose_[i][4] = q.y();
        para_Pose_[i][5] = q.z();
        para_Pose_[i][6] = q.w();

        if(USE_IMU) {
            para_Speed_Bias_[i][0] = Vs[i].x();
            para_Speed_Bias_[i][1] = Vs[i].y();
            para_Speed_Bias_[i][2] = Vs[i].z();

            para_Speed_Bias_[i][3] = Bas[i].x();
            para_Speed_Bias_[i][4] = Bas[i].y();
            para_Speed_Bias_[i][5] = Bas[i].z();

            para_Speed_Bias_[i][6] = Bgs[i].x();
            para_Speed_Bias_[i][7] = Bgs[i].y();
            para_Speed_Bias_[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        para_Ex_Pose_[i][0] = T_IC_[i].x();
        para_Ex_Pose_[i][1] = T_IC_[i].y();
        para_Ex_Pose_[i][2] = T_IC_[i].z();
        Quaterniond q{R_IC_[i]};
        para_Ex_Pose_[i][3] = q.x();
        para_Ex_Pose_[i][4] = q.y();
        para_Ex_Pose_[i][5] = q.z();
        para_Ex_Pose_[i][6] = q.w();
    }

    VectorXd inv_depths = f_manager_.getRobustFeatureDepthVec();
    for (int i = 0; i < f_manager_.getRobustFeatureCount(); i++) {
        para_Features_[i][0] = inv_depths(i);
    }

    para_Td_[0][0] = Tdiff_;
}

void VinsEstimator::CeresParamToState() {
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (flag_failure_occur_) {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        flag_failure_occur_ = 0;
    }

    if(USE_IMU) {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose_[0][6],
                                                          para_Pose_[0][3],
                                                          para_Pose_[0][4],
                                                          para_Pose_[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
            // printf("[ DBG ] euler singular point! ";
            rot_diff = Rs[0] * Quaterniond(para_Pose_[0][6],
                                           para_Pose_[0][3],
                                           para_Pose_[0][4],
                                           para_Pose_[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++) {

            Rs[i] = rot_diff * Quaterniond(para_Pose_[i][6], para_Pose_[i][3], para_Pose_[i][4], para_Pose_[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose_[i][0] - para_Pose_[0][0],
                                    para_Pose_[i][1] - para_Pose_[0][1],
                                    para_Pose_[i][2] - para_Pose_[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_Speed_Bias_[i][0],
                                            para_Speed_Bias_[i][1],
                                            para_Speed_Bias_[i][2]);

                Bas[i] = Vector3d(para_Speed_Bias_[i][3],
                                  para_Speed_Bias_[i][4],
                                  para_Speed_Bias_[i][5]);

                Bgs[i] = Vector3d(para_Speed_Bias_[i][6],
                                  para_Speed_Bias_[i][7],
                                  para_Speed_Bias_[i][8]);
            
        }
    }
    else {
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            Rs[i] = Quaterniond(para_Pose_[i][6], para_Pose_[i][3], para_Pose_[i][4], para_Pose_[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose_[i][0], para_Pose_[i][1], para_Pose_[i][2]);
        }
    }

    if(USE_IMU) {
        for (int i = 0; i < NUM_OF_CAM; i++) {
            T_IC_[i] = Vector3d(para_Ex_Pose_[i][0],
                              para_Ex_Pose_[i][1],
                              para_Ex_Pose_[i][2]);
            R_IC_[i] = Quaterniond(para_Ex_Pose_[i][6],
                                 para_Ex_Pose_[i][3],
                                 para_Ex_Pose_[i][4],
                                 para_Ex_Pose_[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd inv_depths = f_manager_.getRobustFeatureDepthVec();
    for (int i = 0; i < f_manager_.getRobustFeatureCount(); i++) {
        inv_depths(i) = para_Features_[i][0];
    }
    f_manager_.setRobustFeatureDepth(inv_depths);

    if(USE_IMU) {
        Tdiff_ = para_Td_[0][0];
    }
}

void VinsEstimator::slideWindowNew() {
    sum_slide_new_++;
    f_manager_.removeFront(frame_count);
}

void VinsEstimator::slideWindowOld() {
    sum_slide_old_++;

    bool shift_depth = flag_solver_type_ == NON_LINEAR ? true : false;
    if (shift_depth) {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * R_IC_[0];
        R1 = Rs[0] * R_IC_[0];
        P0 = back_P0 + back_R0 * T_IC_[0];
        P1 = Ps[0] + Rs[0] * T_IC_[0];
        f_manager_.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else {
        f_manager_.removeBack();
    }
}

void VinsEstimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void VinsEstimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

double VinsEstimator::reprojectionError(
    Matrix3d &Ri, Vector3d &Pi, Matrix3d &Ric_i, Vector3d &Tic_i,
    Matrix3d &Rj, Vector3d &Pj, Matrix3d &Ric_j, Vector3d &Tic_j, 
    double Depth, Vector3d &UVi, Vector3d &UVj) {
    /** 描述：纯粹地计算重投影误差 */
    Vector3d pts_w = Ri * (Ric_i * (Depth * UVi) + Tic_i) + Pi;
    Vector3d pts_cj = Ric_j.transpose() * (Rj.transpose() * (pts_w - Pj) - Tic_j);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - UVj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}


