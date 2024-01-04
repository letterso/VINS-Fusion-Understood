/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

FeatureManager::FeatureManager(Matrix3d _Rs[]) 
    // : Rs(_Rs) 
{
    for (int i = 0; i < NUM_OF_CAM; i++) {
        // ric[i].setIdentity();
    }
}

void FeatureManager::clearState() {
    features_.clear();
}

// void FeatureManager::setRic(Matrix3d _ric[]) {
//     for (int i = 0; i < NUM_OF_CAM; i++) {
//         // ric[i] = _ric[i];
//     }
// }

bool FeatureManager::addFeatureCheckParallax(int _frameCount, 
    const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_image, double td) {
    /** 描述：
     * 
    */

    // printf("[ DBG ] input feature: %d", (int)_image.size());
    // printf("[ DBG ] num of feature: %d", getRobustFeatureCount());

    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num_ = 0;
    last_average_parallax_ = 0;
    new_feature_num_ = 0;
    long_track_num_ = 0;

    // id_pts: {FeatID, vector{FeatPointInLeftImage, FeatPointInRightImage}}
    for (auto &id_pts /*one feature*/ : _image) {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td); /*左目*/
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2) /*双目时也保存右目*/ {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(features_.begin(), features_.end(), 
            [feature_id](const FeaturePerId &it) {return it.feature_id == feature_id;});

        if (it == features_.end()) {
            features_.push_back(FeaturePerId(feature_id, _frameCount));
            features_.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num_++;
        }
        else if (it->feature_id == feature_id) {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num_++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num_++;
        }
    }

    //if (_frameCount < 2 || last_track_num_ < 20)
    //if (_frameCount < 2 || last_track_num_ < 20 || new_feature_num_ > 0.5 * last_track_num_)

    // 如果满足4个条件之一，直接认为是关键帧
    if (_frameCount < 2 || last_track_num_ < 20 || long_track_num_ < 40 
        || new_feature_num_ > 0.5 * last_track_num_) {
        return true;
    }

    // 仅对已经被多次观测到的feature们，计算一个平均视差
    for (auto &it_per_id : features_) {
        if (it_per_id.start_frame <= _frameCount - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= _frameCount - 1) {
            parallax_sum += compensatedParallax2(it_per_id, _frameCount);
            parallax_num++;
        }
    }

    // 根据视差判断是否为关键帧
    if (parallax_num == 0) {
        return true;
    }
    else {
        // printf("[ DBG ] parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        // printf("[ DBG ] current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax_ = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX; /*如果平均视差超过阈值，认定为出现新关键帧*/
    }
}

int FeatureManager::getRobustFeatureCount() {
    int cnt = 0;
    for (auto &it : features_) {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4) {
            cnt++;
        }
    }
    return cnt;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frameCountL, int frameCountR) {
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : features_) {
        if (it.start_frame <= frameCountL && it.endFrame() >= frameCountR) {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frameCountL - it.start_frame;
            int idx_r = frameCountR - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setRobustFeatureDepth(const VectorXd &x) {
    int feature_index = -1;
    for (auto &it_per_id : features_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) { continue; }

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0) {
            it_per_id.solve_flag = 2;
        }
        else {
            it_per_id.solve_flag = 1;
        }
    }
}

void FeatureManager::clearAllFeatureDepth() {
    for (auto &it_per_id : features_)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getRobustFeatureDepthVec() {
    VectorXd dep_vec(getRobustFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : features_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) { continue; }
        #if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth; /*使用【逆深度】*/
        #else
        dep_vec(++feature_index) = it_per_id->estimated_depth; /*使用【普通深度】*/
        #endif
    }
    return dep_vec;
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]) {
    /** 函数功能描述：遍历所有特征点，如果尚未三角化（实质是初始化深度信息），初始化之。 */

    for (auto &it_per_id : features_) {

        // 如果已经三角化过了（深度值有效），跳过
        if (it_per_id.estimated_depth > 0) { continue; }

        // 双目方案下且特征点被左右目都看到了，就用双目视觉初始化其深度，然后结束
        if (STEREO && it_per_id.feature_per_frame[0].is_stereo) {
            int frame_i = it_per_id.start_frame;/*观察到该feature的首帧，深度要表达在这个帧下*/
            Eigen::Matrix<double, 3, 4> leftPose;/*滑窗状态量是IMU位姿，这里需要计算出cam系的位姿*/
            Eigen::Vector3d t0 = Ps[frame_i] + Rs[frame_i] * tic[0];/*cam系的位置*/
            Eigen::Matrix3d R0 = Rs[frame_i] * ric[0];/*cam系的姿态*/
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            Eigen::Matrix<double, 3, 4> rightPose;/*同理*/
            Eigen::Vector3d t1 = Ps[frame_i] + Rs[frame_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[frame_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;/*像素坐标*/
            Eigen::Vector3d point3d;/*恢复出的3D空间坐标*/
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            // std::cout << "point0 " << point0.transpose() << std::endl;
            // std::cout << "point1 " << point1.transpose() << std::endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;/**/
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0) {
                it_per_id.estimated_depth = depth;/*计算成功*/
            }
            else {
                it_per_id.estimated_depth = INIT_DEPTH;/*计算失败，使用默认值*/
            }
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, 
                point3d.x(), point3d.y(), point3d.z(), ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue; /*初始化任务完成，直接结束*/
        }
        // 单目or双目但只被单目看到了，若有>=2帧共视到了这个特征点，就用相邻帧初始化其深度，然后结束
        else if (it_per_id.feature_per_frame.size() > 1) {
            int frame_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[frame_i] + Rs[frame_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[frame_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            frame_i++;/*单目时，移步下一帧*/
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[frame_i] + Rs[frame_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[frame_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue; /*初始化任务完成，直接结束*/
        }

        /// NOTE: 如果上边的初始化逻辑都没进去，继续往下，尝试用所有共视帧初始化深度 
        // 只有特征点的共视帧足够多（>=4），才能初始化深度值
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) {
            continue;
        }

        int frame_i = it_per_id.start_frame;
        int frame_j = frame_i - 1; //往前取一帧，但j是看不到当前特征点的呀，怎么理解？

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[frame_i] + Rs[frame_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[frame_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        // 特征点的所有共视帧联合估计特征点深度，构建为最小二乘问题
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            frame_j++;

            Eigen::Vector3d t1 = Ps[frame_j] + Rs[frame_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[frame_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (frame_i == frame_j) {
                continue;
            }
        }
        // ROS_ASSERT(svd_idx == svd_A.rows());
        assert(svd_idx == svd_A.rows());

        // 这里相当于求解最小二乘问题
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;
        LOG(INFO) << "############# 特征点初始化深度值(BySVD): FeatID " << it_per_id.feature_id 
            << ", svd estimation depth " << it_per_id.estimated_depth;

        if (it_per_id.estimated_depth < 0.1) {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
    Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d/*output*/) {
    /** 描述：已知两个cam的位姿，已知特征在两个cam中的像素坐标，计算特征在第一个cam坐标系中的三维坐标（也即三角化） */

    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3); /*通过三角化，得到了特征的3D坐标*/
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]) {
    /** 描述：
     * 
    */

    if(frameCnt > 0) {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : features_) {
            if (it_per_id.estimated_depth > 0) {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1) {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D)) {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            // std::cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << std::endl;
            // std::cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << std::endl;
        }
    }
}

bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
    vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D) {
    /** 描述：
     * 
    */

    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4) {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ) {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    // std::cout << "r " << endl << r << std::endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::removeFront(int _frameCount) {
    for (auto it = features_.begin(), it_next = features_.begin(); it != features_.end(); it = it_next) {
        it_next++;

        if (it->start_frame == _frameCount) {
            it->start_frame--;
        }
        else {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < _frameCount - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                features_.erase(it);
        }
    }
}

void FeatureManager::removeBack() {
    for (auto it = features_.begin(), it_next = features_.begin(); it != features_.end(); it = it_next) {
        it_next++;

        if (it->start_frame != 0) {
            it->start_frame--;
        }
        else {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                features_.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, 
    Eigen::Matrix3d new_R, Eigen::Vector3d new_P) {
    /** 描述：
     * 
    */

    for (auto it = features_.begin(), it_next = features_.begin(); it != features_.end(); it = it_next) {
        it_next++;

        if (it->start_frame != 0) {
            it->start_frame--;
        }
        else {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2) /*不够鲁邦的特征，直接丢掉*/ {
                features_.erase(it);
                continue;
            }
            else {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1) {
            features_.erase(it);
        }
        */
    }
}

void FeatureManager::removeOutlier(set<int> &outlierIndex) {
    std::set<int>::iterator itSet;
    for (auto it = features_.begin(), it_next = features_.begin(); it != features_.end(); it = it_next) {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end()) {
            features_.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeFailures() {
    for (auto it = features_.begin(), it_next = features_.begin();
         it != features_.end(); it = it_next) {
        it_next++;
        if (it->solve_flag == 2)
            features_.erase(it);
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int _frameCount) {
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[_frameCount - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[_frameCount - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = _frameCount - 2;
    //int r_j = _frameCount - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

