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

#include "feature_tracker.h"

namespace {

// double distance(cv::Point2f pt1, cv::Point2f pt2) {
//     double dx = pt1.x - pt2.x;
//     double dy = pt1.y - pt2.y;
//     return sqrt(dx * dx + dy * dy);
// }

struct greaterThanPtr {
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

void reduceVector(vector<cv::Point2f> &v, vector<uchar> masK_status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (masK_status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> masK_status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (masK_status[i])
            v[j++] = v[i];
    v.resize(j);
}

} // namespace

FeatureTracker::FeatureTracker() {
    flag_stereo_cam_ = 0;
    pt_id_ = 0;
    has_predict_feats_ = false;

#ifdef LET_NET
    letnetInit();
#endif
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file) {
    for (size_t i = 0; i < calib_file.size(); i++) {
        // printf("[FeatureTracker] reading camera %d intrinsics from: %s \n", i, calib_file[i].c_str());
        LOG(INFO) << "read cam " << i << " intrinsics from: " << calib_file[i];
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2) {
        flag_stereo_cam_ = true;
        LOG(INFO) << "Oh yse, we work in Stereo Mode.";
    }
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(
    double _cur_time, const cv::Mat &_img, const cv::Mat &_img1) {
    /** 这个函数的逻辑是： 是FeatureTrackEr类的pipeline，承担FeatTracking的职责。
     * 首先在前后帧（的左目）之间做光流追踪，若特征数量不足则补充新的角点特征；
     * 然后在当前帧的左右目（若有）之间做光流追踪，并保存在独立的容器中；
     * 最后，返回所有追踪到的特征点的信息，包括{FeatID, CamID, xyz_uv_VxVy}。
    */

    TicToc track_timer;
    cur_time = _cur_time;
    cur_img = _img;
    row_ = cur_img.rows;
    col_ = cur_img.cols;
    cv::Mat rightImg = _img1;
    cur_pts_.clear();

    /*
    {
        // HE直方图增强算法，一种比较古老的对比度增强算法，有两种变体，AHE和CLAHE，后者相比前者改善了放大噪声的问题。
        // 因此显然，这里的作用是对左右目图像做对比度增强，作者已关闭此功能，大概是仅在开发过程中测试过。
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty()) { clahe->apply(rightImg, rightImg); }
    }
    */

#ifdef LET_NET
    letnetProcess(cur_img);
#endif

    // 若非初始帧，则在前后帧（仅左目）之间进行常规的feature追踪
    if (prev_pts_.size() > 0) {
        TicToc lk_timer;
        vector<uchar> lk_status; /*这个变量记录了LK追踪成功与否，1表示成功，0为失败，下文中会一直用它记录追踪结果*/
        vector<float> lk_err;    /*追踪成功的点的某种误差，反正下文中也不会用到，不管了*/

#ifdef LET_NET
        float k_w = float(prev_img.cols) / float(LET_WIDTH);
        float k_h = float(prev_img.rows) / float(LET_HEIGHT);

        // resize
        std::vector<cv::Point2f> prev_pts_scale, cur_pts_scale;
        prev_pts_scale.resize(prev_pts_.size());
        for (int i = 0; i < int(prev_pts_.size()); i++)
        {
            prev_pts_scale[i].x = prev_pts_[i].x / k_w;
            prev_pts_scale[i].y = prev_pts_[i].y / k_h;
        }

        if (false)// 使用预测错误匹配更多
        { 
            cur_pts_scale.resize(predict_pts_.size());
            for (int i = 0; i < int(predict_pts_.size()); i++)
            {
                cur_pts_scale[i].x = predict_pts_[i].x / k_w;
                cur_pts_scale[i].y = predict_pts_[i].y / k_h;
            }

            /* 因为有预测所以可以收紧参数以加快搜索速度 */
            cv::calcOpticalFlowPyrLK(last_desc_, desc_, prev_pts_scale, cur_pts_scale /*输出的追踪结果，允许给出初值*/,
                                     lk_status, lk_err, cv::Size(21, 21) /*指定金字塔每层上的搜索窗口的size*/, 2 /*指定金字塔层数为2层*/,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01) /*指定搜索的终止条件*/,
                                     cv::OPTFLOW_USE_INITIAL_FLOW /*指定启用cur_pts_中给定的初值，以便加速搜索*/);
            int succ_num = 0;
            for (size_t i = 0; i < lk_status.size(); i++)
            {
                if (lk_status[i])
                {
                    succ_num++;
                }
            }

            /* 如果追踪失败（点数太少），则放宽搜索的参数，重新追踪一次 */
            if (succ_num < 10)
            {
                cv::calcOpticalFlowPyrLK(last_desc_, desc_, prev_pts_scale, cur_pts_scale, lk_status, lk_err, cv::Size(21, 21), 3);
            }
        }
        else
        {
            /* 如果没有给出预测，直接使用更宽松的参数，追踪一次 */
            cv::calcOpticalFlowPyrLK(last_desc_, desc_, prev_pts_scale, cur_pts_scale, lk_status, lk_err, cv::Size(21, 21), 3);
        }

        // [reverse check]若启用了反向校验（由k帧向k-1帧追踪，和正向追踪结果做校验），执行之
        if(FLOW_BACK) {
            vector<uchar> reverse_status; /*reverse track status*/
            vector<cv::Point2f> reverse_pts_scale = prev_pts_scale;
            /* 因为有预测，所以同样使用收窄的搜索参数 */
            cv::calcOpticalFlowPyrLK(desc_, last_desc_, cur_pts_scale, reverse_pts_scale, reverse_status, lk_err, cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            for(size_t i = 0; i < lk_status.size(); i++) {
                /* 若正方向追踪都成功了且正反追踪结果都是同一个点，则认定追踪成功，否则追踪失败 */
                if(lk_status[i] && reverse_status[i] && distance(prev_pts_scale[i], reverse_pts_scale[i]) <= 0.5) {
                    lk_status[i] = 1;
                }
                else {
                    lk_status[i] = 0;
                }
            }
        }

        // resize
        cur_pts_.resize(cur_pts_scale.size());
        for (int i = 0; i < int(cur_pts_scale.size()); i++)
        {
            cur_pts_[i].x = cur_pts_scale[i].x * k_w;
            cur_pts_[i].y = cur_pts_scale[i].y * k_h;
        }

        // subpixel refinement
        cv::cornerSubPix(cur_img,
                         cur_pts_,
                         cv::Size(3, 3),
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                         5, 0.01));
#else
        if(has_predict_feats_) {
            cur_pts_ = predict_pts_;
            /* 针对左目上的featrure，在前后帧之间进行追踪，注意此处的丰富形参，因为有预测所以可以收紧参数以加快搜索速度 */
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts_, cur_pts_ /*输出的追踪结果，允许给出初值*/, 
                lk_status, lk_err, cv::Size(21, 21) /*指定金字塔每层上的搜索窗口的size*/, 1/*指定金字塔层数为2层*/, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01) /*指定搜索的终止条件*/, 
                cv::OPTFLOW_USE_INITIAL_FLOW /*指定启用cur_pts_中给定的初值，以便加速搜索*/ );
            int succ_num = 0;
            for (size_t i = 0; i < lk_status.size(); i++) {
                if (lk_status[i]) {
                    succ_num++;
                }
            }

            /* 如果追踪失败（点数太少），则放宽搜索的参数，重新追踪一次 */
            if (succ_num < 10) {
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts_, cur_pts_, lk_status, lk_err, cv::Size(21, 21), 3);
            }
        }
        else {
            /* 如果没有给出预测，直接使用更宽松的参数，追踪一次 */
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts_, cur_pts_, lk_status, lk_err, cv::Size(21, 21), 3);
        }

        // [reverse check]若启用了反向校验（由k帧向k-1帧追踪，和正向追踪结果做校验），执行之
        if(FLOW_BACK) {
            vector<uchar> reverse_status; /*reverse track status*/
            vector<cv::Point2f> reverse_pts = prev_pts_;
            /* 因为有预测，所以同样使用收窄的搜索参数 */
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts_, reverse_pts, reverse_status, lk_err, cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts_, reverse_pts, reverse_status, lk_err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < lk_status.size(); i++) {
                /* 若正方向追踪都成功了且正反追踪结果都是同一个点，则认定追踪成功，否则追踪失败 */
                if(lk_status[i] && reverse_status[i] && distance(prev_pts_[i], reverse_pts[i]) <= 0.5) {
                    lk_status[i] = 1;
                }
                else {
                    lk_status[i] = 0;
                }
            }
        }
#endif

        // 追踪成功的feature，还必须位于图像范围内；这里给出的是真正的（左目）最终结果
        for (int i = 0; i < int(cur_pts_.size()); i++) {
            if (lk_status[i] && !inBorder(cur_pts_[i])) {
                lk_status[i] = 0;
            }
        }

        // 移除追踪失败的feature
        reduceVector(prev_pts_, lk_status);
        reduceVector(cur_pts_, lk_status);
        reduceVector(cur_ids_, lk_status);
        reduceVector(tracked_times_, lk_status);

        LOG(INFO) << "[LK] left: prev2curr, tracked " << cur_ids_.size() << " feats, took " << lk_timer.toc() << "ms";

    } /*完成左目图像的追踪*/

    // 此时留下的都是追踪成功的feature，计数加1
    for (auto &cnt : tracked_times_) {
        cnt++;
    }

    // 在当前特征点数量不足时，提取新的特征点，初始化没有点时直接进入该函数
    if (1) {
        TicToc set_mask_timer;
        setCurrFeatureAsMask();
        // printf("[ DBG ] set MASK took %fms \n", set_mask_timer.toc());

        TicToc add_feats_timer;
#ifdef LET_NET
        const int max_new_corners = MAX_CNT - static_cast<int>(cur_pts_.size());
        
        if (max_new_corners > 0)
        {
            if (exist_pts_mask_.empty()) { LOG(INFO) << "search new corners: MASK is empty."; }
            if (exist_pts_mask_.type() != CV_8UC1) { LOG(INFO) << "search new corners: MASK type wrong."; }

            cv::resize(exist_pts_mask_, exist_pts_mask_, cv::Size(LET_WIDTH, LET_HEIGHT));
            letnetFeaturesToTrack(score_, new_pts_, max_new_corners, 0.0001, MIN_DIST, exist_pts_mask_);
            float k_w = float(prev_img.cols) / float(LET_WIDTH);
            float k_h = float(prev_img.rows) / float(LET_HEIGHT);
            for (auto &new_pt_ : new_pts_)
            {
                new_pt_.x *= k_w;
                new_pt_.y *= k_h;
            }
            // subpixel refinement
            cv::cornerSubPix(cur_img,
                             new_pts_,
                             cv::Size(3, 3),
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                              5, 0.01));
            for (auto &pxl : new_pts_)
            {
                cur_pts_.push_back(pxl);
                cur_ids_.push_back(pt_id_++);
                tracked_times_.push_back(1);
            }
        }
#else
        // 将特征点数量补齐至设定数量
        const int max_new_corners = MAX_CNT - static_cast<int>(cur_pts_.size());
        if (max_new_corners > 0) {
            if(exist_pts_mask_.empty()) { LOG(INFO) << "search new corners: MASK is empty."; }
            if (exist_pts_mask_.type() != CV_8UC1) { LOG(INFO) << "search new corners: MASK type wrong."; }
            /*提取出新的N个最强的角点，可选Harris角点或MinEigenVal角点, 默认使用后者*/
            cv::goodFeaturesToTrack(cur_img, new_pts_, max_new_corners, 0.01, MIN_DIST, exist_pts_mask_);
        }
        else {
            new_pts_.clear();
        }
        for (auto &pxl : new_pts_) {
            cur_pts_.push_back(pxl);
            cur_ids_.push_back(pt_id_++);
            tracked_times_.push_back(1);
        }
#endif
        LOG(INFO) << "[LK] detected & added new feats " << new_pts_.size() << ", took " << add_feats_timer.toc() << "ms";
    }
        // 当前帧左目图像特征点，去畸变&保存坐标，计算像素移动速度
        cur_un_pts = undistortedPts(cur_pts_, m_camera[0]);
        pts_velocity_ = ptsVelocity(cur_ids_, cur_un_pts, cur_un_pts_map_ /*输出项*/, prev_un_pts_map_);

        // 如果是双目方案，在【当前帧】的【左右图像】之间进行光流追踪；顺便
        if(!_img1.empty() && flag_stereo_cam_) {
            TicToc lk_timer;
            iDs_right_.clear();
            cur_right_pts_.clear();
            cur_right_un_pts.clear();
            right_pts_velocity_.clear();
            cur_right_un_pts_map_.clear();
            if(!cur_pts_.empty()) {
                // printf("stereo image; track feature on right image \n");
                vector<cv::Point2f> reverseLeftPts;
                vector<uchar> lkStatusL2R, lkStatusR2L;
                vector<float> lkErr;
                // cur left >>>> cur right （无预测情况下，左目到右目光流追踪）
                cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts_, cur_right_pts_, lkStatusL2R, lkErr, cv::Size(21, 21), 3);
                if(FLOW_BACK) {
                    // reverse check: cur right >>>> cur left （无预测情况下，右目到左目光流追踪）
                    cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts_, reverseLeftPts, lkStatusR2L, lkErr, cv::Size(21, 21), 3);
                    for(size_t i = 0; i < lkStatusL2R.size(); i++) {
                        if(lkStatusL2R[i] && lkStatusR2L[i] && inBorder(cur_right_pts_[i]) && distance(cur_pts_[i], reverseLeftPts[i]) <= 0.5) {
                            lkStatusL2R[i] = 1;
                        }
                        else {
                            lkStatusL2R[i] = 0;
                        }
                    }
                }

                // 保存有效结果，注意{右目特征点数量<=左目特征点数量}
                iDs_right_ = cur_ids_;
                reduceVector(cur_right_pts_, lkStatusL2R);
                reduceVector(iDs_right_, lkStatusL2R);
                // ###### only keep left-right pts (以下为作者屏蔽) ######
                // reduceVector(cur_pts_, lkStatusL2R);
                // reduceVector(cur_ids_, lkStatusL2R);
                // reduceVector(tracked_times_, lkStatusL2R);
                // reduceVector(cur_un_pts, lkStatusL2R);
                // reduceVector(pts_velocity_, lkStatusL2R);

                // 当前帧右目图像特征点，去畸变&保存坐标，计算像素移动速度
                cur_right_un_pts = undistortedPts(cur_right_pts_, m_camera[1]);
                right_pts_velocity_ = ptsVelocity(iDs_right_, cur_right_un_pts, cur_right_un_pts_map_ /*输出项*/, prev_right_un_pts_map_);

                LOG(INFO) << "[LK] curr: left2right, tracked " << iDs_right_.size() << " feats, took " << lk_timer.toc() << "ms";
            }
        } /*完成[当前帧左右图像间]的追踪*/

        // 在当前左目图像上绘制追踪结果
        if(SHOW_TRACK) {
            drawTrack(cur_img, rightImg, cur_ids_, cur_pts_, cur_right_pts_, prev_pts_map_);
        }

        // 更新成员变量，currXX变成prevXX
        prev_time = cur_time;
        prev_img = cur_img;
        prev_pts_ = cur_pts_;
        prev_un_pts = cur_un_pts;
        prev_un_pts_map_ = cur_un_pts_map_;
        prev_right_un_pts_map_ = cur_right_un_pts_map_;
        has_predict_feats_ = false;
        prev_pts_map_.clear();
        for(size_t i = 0; i < cur_pts_.size(); i++) {
            prev_pts_map_[cur_ids_[i]] = cur_pts_[i];
        }

        // 开始写入最终结果，以向外返回
        /* 结构说明：{FeatureID, vector<{CamID, 像素信息(xyz_uv_VxVy)}>} */
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> feats_frame;

        // 保存特征点在左目上的像素位置、移动速度等信息
        for (size_t i = 0; i < cur_ids_.size(); i++) {
            int feat_id = cur_ids_[i];
            double x, y, z;
            x = cur_un_pts[i].x;
            y = cur_un_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_pts_[i].x;
            p_v = cur_pts_[i].y;
            int camera_id = 0; /*左目ID为0*/
            double velocity_x, velocity_y;
            velocity_x = pts_velocity_[i].x;
            velocity_y = pts_velocity_[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            feats_frame[feat_id].emplace_back(camera_id, xyz_uv_velocity);
        }

        // 双目方案下，也保存特征点在右目上的像素位置、速度等信息
        /*若特征点在右目上也被观测到，则特征点相应vector的size会是2 */
        if (!_img1.empty() && flag_stereo_cam_) {
            for (size_t i = 0; i < iDs_right_.size(); i++) {
                int feat_id = iDs_right_[i];
                double x, y, z;
                x = cur_right_un_pts[i].x;
                y = cur_right_un_pts[i].y;
                z = 1;
                double p_u, p_v;
                p_u = cur_right_pts_[i].x;
                p_v = cur_right_pts_[i].y;
                int camera_id = 1; /*右目ID为1*/
                double velocity_x, velocity_y;
                velocity_x = right_pts_velocity_[i].x;
                velocity_y = right_pts_velocity_[i].y;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                feats_frame[feat_id].emplace_back(camera_id, xyz_uv_velocity);
            }
        }

        // printf("feature tracking whole duration: %fms \n", track_timer.toc());
        return feats_frame;
    }

    void FeatureTracker::removeOutliers(set<int> & removePtsIds)
    {
        std::set<int>::iterator itSet;
        vector<uchar> masK_status;
        for (size_t i = 0; i < cur_ids_.size(); i++)
        {
            itSet = removePtsIds.find(cur_ids_[i]);
            if (itSet != removePtsIds.end())
                masK_status.push_back(0);
            else
                masK_status.push_back(1);
        }

        reduceVector(prev_pts_, masK_status);
        reduceVector(cur_ids_, masK_status);
        reduceVector(tracked_times_, masK_status);
    }

    void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> & predictPts)
    {
        /** 这个函数的逻辑是： 顾名思义，设置下一帧上feature的位置预测 */
        has_predict_feats_ = true;
        predict_pts_.clear();
        predict_pTs_show.clear();
        map<int, Eigen::Vector3d>::iterator itPredict;
        for (size_t i = 0; i < cur_ids_.size(); i++)
        {
            int id = cur_ids_[i];
            itPredict = predictPts.find(id);
            if (itPredict != predictPts.end())
            {
                Eigen::Vector2d tmp_uv;
                m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
                predict_pts_.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
                predict_pTs_show.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            }
            else
            {
                predict_pts_.push_back(prev_pts_[i]);
            }
        }
    }

    cv::Mat FeatureTracker::getTrackImage()
    {
        return img_track_show_;
    }

    void FeatureTracker::showUndistortion(const string &name)
    {
        cv::Mat undistortedImg(row_ + 600, col_ + 600, CV_8UC1, cv::Scalar(0));
        vector<Eigen::Vector2d> distortedp, undistortedp;
        for (int i = 0; i < col_; i++)
            for (int j = 0; j < row_; j++)
            {
                Eigen::Vector2d a(i, j);
                Eigen::Vector3d b;
                m_camera[0]->liftProjective(a, b);
                distortedp.push_back(a);
                undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
                // printf("%f,%f -> %f,%f,%f) \n", a.x(), a.y(), b.x(), b.y(), b.z());
            }
        for (int i = 0; i < int(undistortedp.size()); i++)
        {
            cv::Mat pp(3, 1, CV_32FC1);
            pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col_ / 2;
            pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row_ / 2;
            pp.at<float>(2, 0) = 1.0;
            // LOG(INFO) << trackerData[0].K;
            // printf("%lf %lf \n", p.at<float>(1, 0), p.at<float>(0, 0));
            // printf("%lf %lf \n", pp.at<float>(1, 0), pp.at<float>(0, 0));
            if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row_ + 600 &&
                pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col_ + 600)
            {
                // xxx
                undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) =
                    cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
            }
            else
            {
                // printf("[ ERROR ] (%f %f) -> (%f %f) \n", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
            }
        }
        // turn the following code on if you need
        // cv::imshow(name, undistortedImg);
        // cv::waitKey(0);
    }

    // ********************** 以下为实际意义上的“私有函数” ********************** //

    void FeatureTracker::rejectWithF()
    {
        if (cur_pts_.size() >= 8)
        {
            // printf("[ DBG ] FM ransac begins \n");
            TicToc t_f;
            vector<cv::Point2f> curUnPts(cur_pts_.size()), un_prev_pts(prev_pts_.size());
            for (unsigned int i = 0; i < cur_pts_.size(); i++)
            {
                Eigen::Vector3d tmp_p;
                m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts_[i].x, cur_pts_[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col_ / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row_ / 2.0;
                curUnPts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts_[i].x, prev_pts_[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col_ / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row_ / 2.0;
                un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }

            vector<uchar> masK_status;
            cv::findFundamentalMat(curUnPts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, masK_status);
            int size_a = cur_pts_.size();
            reduceVector(prev_pts_, masK_status);
            reduceVector(cur_pts_, masK_status);
            reduceVector(cur_un_pts, masK_status);
            reduceVector(cur_ids_, masK_status);
            reduceVector(tracked_times_, masK_status);
            // printf("[ DBG ] FM ransac: %d -> %lu: %f \n", size_a, cur_pts_.size(), 1.0 * cur_pts_.size() / size_a);
            // printf("[ DBG ] FM ransac costs: %fms \n", t_f.toc());
        }
    }

    void FeatureTracker::setCurrFeatureAsMask()
    {
        /** 这个函数的逻辑是： 将当前特征点的位置设置为MASK，
         * 后续如果需要补充新的特征点，将跳过这些MASK区域以避免重复特征
         */

        exist_pts_mask_ = cv::Mat(row_, col_, CV_8UC1, cv::Scalar(255)); /*先重置为可提取区域*/

        // prefer to keep features that are tracked for long time
        vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id; /*{追踪次数，像素坐标，特征点id}*/

        for (unsigned int i = 0; i < cur_pts_.size(); i++)
            cnt_pts_id.push_back(make_pair(tracked_times_[i], make_pair(cur_pts_[i], cur_ids_[i])));

        // 按照点的被跟踪次数，从多到少进行排序
        sort(cnt_pts_id.begin(), cnt_pts_id.end(),
             [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
             { return a.first > b.first; });

        cur_pts_.clear();
        cur_ids_.clear();
        tracked_times_.clear();

        for (auto &it : cnt_pts_id)
        {
            if (exist_pts_mask_.at<uchar>(it.second.first) == 255)
            {
                cur_pts_.push_back(it.second.first);
                cur_ids_.push_back(it.second.second);
                tracked_times_.push_back(it.first);
                /* 已有feature附近半径范围内，全部填充为0，也即MASK区域*/
                cv::circle(exist_pts_mask_, it.second.first, MIN_DIST, 0, -1);
            }
        }
    }

    vector<cv::Point2f> FeatureTracker::undistortedPts(
        vector<cv::Point2f> & PTs, camodocal::CameraPtr CAM)
    {
        /** 这个函数的逻辑是： 顾名思义，用相机内参对特征点像素坐标去畸变 */

        vector<cv::Point2f> un_pts;
        for (unsigned int i = 0; i < PTs.size(); i++)
        {
            Eigen::Vector2d a(PTs[i].x, PTs[i].y);
            Eigen::Vector3d b;
            CAM->liftProjective(a, b);
            un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        }
        return un_pts;
    }

    vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> & IDs, vector<cv::Point2f> & PTs,
                                                    map<int, cv::Point2f> & curIdPtMap, map<int, cv::Point2f> & preIdPtMap)
    {
        /** 这个函数的逻辑是： 计算特征点像素的移动速度，并将curr帧的特征点保存到map中 */

        vector<cv::Point2f> pts_velocity_;
        curIdPtMap.clear();
        for (unsigned int i = 0; i < IDs.size(); i++)
        {
            curIdPtMap.insert(make_pair(IDs[i], PTs[i]));
        }

        // caculate points velocity
        if (!preIdPtMap.empty())
        {
            double dt = cur_time - prev_time;

            for (unsigned int i = 0; i < PTs.size(); i++)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = preIdPtMap.find(IDs[i]);
                if (it != preIdPtMap.end())
                {
                    double v_x = (PTs[i].x - it->second.x) / dt;
                    double v_y = (PTs[i].y - it->second.y) / dt;
                    pts_velocity_.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity_.push_back(cv::Point2f(0, 0));
            }
        }
        else
        {
            for (unsigned int i = 0; i < cur_pts_.size(); i++)
            {
                pts_velocity_.push_back(cv::Point2f(0, 0));
            }
        }
        return pts_velocity_;
    }

    void FeatureTracker::letnetInit()
    {
        score_ = cv::Mat(LET_HEIGHT, LET_WIDTH, CV_32FC1);
        desc_ = cv::Mat(LET_HEIGHT, LET_WIDTH, CV_8UC1);
        last_desc_ = cv::Mat(LET_HEIGHT, LET_WIDTH, CV_8UC1);
        net_.load_param(THIS_COM "/model/model_gray.param");
        net_.load_model(THIS_COM "/model/model_gray.bin");
    }

    void FeatureTracker::letnetProcess(const cv::Mat &imageBgr)
    {
        last_desc_ = desc_.clone();
        cv::Mat img;
        cv::resize(imageBgr, img, cv::Size(LET_WIDTH, LET_HEIGHT));
        ncnn::Extractor ex = net_.create_extractor();
        ex.set_light_mode(true);
        // opencv to ncnn
        ncnn::Mat ncnn_in = ncnn::Mat::from_pixels(img.data, ncnn::Mat::PIXEL_GRAY, img.cols, img.rows);
        ncnn_in.substract_mean_normalize(mean_vals, norm_vals);

        // extract
        ncnn::Mat ncnn_score, ncnn_desc;
        ex.input("input", ncnn_in);
        ex.extract("score", ncnn_score);
        ex.extract("descriptor", ncnn_desc);
        
        // ncnn to opencv
        ncnn_score.substract_mean_normalize(mean_vals_inv, norm_vals_inv);
        ncnn_desc.substract_mean_normalize(mean_vals_inv, norm_vals_inv);

        // out1.to_pixels(score.data, ncnn::Mat::PIXEL_GRAY);
        memcpy((unsigned char *)score_.data, ncnn_score.data, LET_HEIGHT * LET_WIDTH * sizeof(float));
        ncnn_desc.to_pixels(desc_.data, ncnn::Mat::PIXEL_GRAY);
        // cv::imwrite("desc.png", desc);
    }

    void FeatureTracker::letnetFeaturesToTrack(cv::InputArray image,
                                               cv::OutputArray _corners,
                                               const int &maxCorners,
                                               const double &qualityLevel,
                                               const double &minDistance,
                                               const cv::InputArray &_mask,
                                               int blockSize)
    {
        CV_Assert(qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0);
        CV_Assert(_mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(image)));

        cv::Mat eig = image.getMat(), tmp;
        double maxVal = 0;
        // 图像，最小值，最大值，最小值指针，最大值指针
        cv::minMaxLoc(eig, 0, &maxVal, 0, 0, _mask); // 获取最大值
        // 基于maxVal * qualityLevel二值化，获取大于maxVal * qualityLevel的像素
        // 所有大于maxVal * qualityLevel的都为候选点，这里qualityLevel设置得很小，
        // 实际基本所有非0点都为候选点
        cv::threshold(eig, eig, maxVal * qualityLevel, 0, cv::THRESH_TOZERO);
        // 膨胀
        cv::dilate(eig, tmp, cv::Mat());

        cv::Size imgsize = eig.size();
        std::vector<const float *> tmpCorners;

        // 获取候选点
        cv::Mat Mask = _mask.getMat();
        for (int y = 1; y < imgsize.height - 1; y++)
        {
            const float *eig_data = (const float *)eig.ptr(y);
            const float *tmp_data = (const float *)tmp.ptr(y);
            const uchar *mask_data = Mask.data ? Mask.ptr(y) : 0;

            for (int x = 1; x < imgsize.width - 1; x++)
            {
                float val = eig_data[x];
                if (val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]))
                    tmpCorners.push_back(eig_data + x);
            }
        }

        std::vector<cv::Point2f> corners;
        std::vector<float> cornersQuality;
        size_t i, j, total = tmpCorners.size(), ncorners = 0;

        if (total == 0)
        {
            _corners.release();
            return;
        }

        std::sort(tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

        if (minDistance >= 1)
        {
            // Partition the image into larger grids
            int w = eig.cols;
            int h = eig.rows;

            const int cell_size = cvRound(minDistance); // 取整
            const int grid_width = (w + cell_size - 1) / cell_size;
            const int grid_height = (h + cell_size - 1) / cell_size;

            std::vector<std::vector<cv::Point2f>> grid(grid_width * grid_height);

            double minDistanceSqrt = minDistance * minDistance;
            for (i = 0; i < total; i++)
            {
                int ofs = (int)((const uchar *)tmpCorners[i] - eig.ptr());
                int y = (int)(ofs / eig.step);
                int x = (int)((ofs - y * eig.step) / sizeof(float));

                bool good = true;

                int x_cell = x / cell_size;
                int y_cell = y / cell_size;

                int x1 = x_cell - 1;
                int y1 = y_cell - 1;
                int x2 = x_cell + 1;
                int y2 = y_cell + 1;

                // boundary check
                x1 = std::max(0, x1);
                y1 = std::max(0, y1);
                x2 = std::min(grid_width - 1, x2);
                y2 = std::min(grid_height - 1, y2);

                for (int yy = y1; yy <= y2; yy++)
                {
                    for (int xx = x1; xx <= x2; xx++)
                    {
                        std::vector<cv::Point2f> &m = grid[yy * grid_width + xx];

                        if (m.size())
                        {
                            for (j = 0; j < m.size(); j++)
                            {
                                float dx = x - m[j].x;
                                float dy = y - m[j].y;

                                if (dx * dx + dy * dy < minDistanceSqrt)
                                {
                                    good = false;
                                    goto break_out;
                                }
                            }
                        }
                    }
                }

            break_out:

                if (good)
                {
                    grid[y_cell * grid_width + x_cell].push_back(cv::Point2f((float)x, (float)y));

                    cornersQuality.push_back(*tmpCorners[i]);

                    corners.push_back(cv::Point2f((float)x, (float)y));
                    ++ncorners;

                    if (maxCorners > 0 && (int)ncorners == maxCorners)
                        break;
                }
            }
        }
        else
        {
            for (i = 0; i < total; i++)
            {
                cornersQuality.push_back(*tmpCorners[i]);

                int ofs = (int)((const uchar *)tmpCorners[i] - eig.ptr());
                int y = (int)(ofs / eig.step);
                int x = (int)((ofs - y * eig.step) / sizeof(float));

                corners.push_back(cv::Point2f((float)x, (float)y));
                ++ncorners;

                if (maxCorners > 0 && (int)ncorners == maxCorners)
                    break;
            }
        }

        cv::Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
    }

    void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, vector<cv::Point2f> &curRightPts, map<int, cv::Point2f> &prevPtsMap)
    {
        /** 这个函数的逻辑是： 顾名思义，将追踪结果绘制在图像上，以供可视化 */

        // int rows = imLeft.rows;
        int cols = imLeft.cols;
        if (!imRight.empty() && flag_stereo_cam_)
            cv::hconcat(imLeft, imRight, img_track_show_);
        else
            img_track_show_ = imLeft.clone();
        if(img_track_show_.channels() == 1)
            cv::cvtColor(img_track_show_, img_track_show_, cv::COLOR_GRAY2RGB);

        for (size_t j = 0; j < curLeftPts.size(); j++)
        {
            double len = std::min(1.0, 1.0 * tracked_times_[j] / 20);
            cv::circle(img_track_show_, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        if (!imRight.empty() && flag_stereo_cam_)
        {
            for (size_t i = 0; i < curRightPts.size(); i++)
            {
                cv::Point2f rightPt = curRightPts[i];
                rightPt.x += cols;
                cv::circle(img_track_show_, rightPt, 2, cv::Scalar(0, 255, 0), 2);
                // cv::Point2f leftPt = curLeftPtsTrackRight[i];
                // cv::line(img_track_show_, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
            }
        }

        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curLeftIds.size(); i++)
        {
            int id = curLeftIds[i];
            mapIt = prevPtsMap.find(id);
            if (mapIt != prevPtsMap.end())
            {
                cv::arrowedLine(img_track_show_, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
            }
        }

        // draw prediction
        /*
        for(size_t i = 0; i < predict_pTs_show.size(); i++) {
            cv::circle(img_track_show_, predict_pTs_show[i], 2, cv::Scalar(0, 170, 255), 2);
        }
        */
        // printf("predict pts size %d \n", (int)predict_pTs_show.size());

        // cv::Mat imCur2Compress;
        // cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
    }

    double FeatureTracker::distance(cv::Point2f & pt1, cv::Point2f & pt2)
    {
        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        return sqrt(dx * dx + dy * dy);
    }

bool FeatureTracker::inBorder(const cv::Point2f &pt) {
    const int BORDER_SIZE = 1;
    int pixel_x = cvRound(pt.x);
    int pixel_y = cvRound(pt.y);
    return (BORDER_SIZE <= pixel_x && pixel_x < col_ - BORDER_SIZE && 
            BORDER_SIZE <= pixel_y && pixel_y < row_ - BORDER_SIZE);
}


