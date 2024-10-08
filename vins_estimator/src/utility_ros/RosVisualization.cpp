/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "RosVisualization.h"

using namespace ros;
using namespace Eigen;

ros::Publisher pub_odometry;
ros::Publisher pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud;
ros::Publisher pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;
ros::Publisher pub_image_track;

nav_msgs::Path ros_path;
CameraPoseVisualization cam_model_viz(1, 0, 0, 1);

static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);
// size_t pub_counter = 0;

void registerPub(ros::NodeHandle &n) {
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);

    cam_model_viz.setScale(0.1);
    cam_model_viz.setLineWidth(0.01);
}

// ========================== 以下，工作在sensor频率 ========================== //

void pubLatestOdometry(const Eigen::Vector3d &P, 
    const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t) {
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void pubTrackImage(const cv::Mat &imgTrack, const double t) {
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}

// ========================== 以下所有，都工作在VIO频率 ========================== //

void printStatistics(const VinsEstimator &estimator, double t) {
    if (estimator.flag_solver_type_ != VinsEstimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f \r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    // ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    // ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    // std::cout << "[DBG] position: " << estimator.Ps[WINDOW_SIZE].transpose() << std::endl;
    // std::cout << "[DBG] orientation: " << estimator.Vs[WINDOW_SIZE].transpose() << std::endl;
    if (ESTIMATE_EXTRINSIC) {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++) {
            // printf("[ DBG ] calibration result for camera %d \n", i);
            // ROS_DEBUG_STREAM("extirnsic Tic: " << estimator.T_IC_[i].transpose());
            // ROS_DEBUG_STREAM("extrinsic Ric: " << Utility::R2ypr(estimator.R_IC_[i]).transpose());
            // std::cout << "[DBG] extirnsic Tic: " << estimator.T_IC_[i].transpose() << std::endl;
            // std::cout << "[DBG] extrinsic Ric: " << Utility::R2ypr(estimator.R_IC_[i]).transpose() << std::endl;

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.R_IC_[i];
            eigen_T.block<3, 1>(0, 3) = estimator.T_IC_[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    // printf("[ DBG ] vo solver costs: %f ms \n", t);
    // printf("[ DBG ] average of time %f ms \n", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    // printf("[ DBG ] sum of path %f \n", sum_of_path);
    if (ESTIMATE_TD) {
        // printf("[printStatis] td %f \n", estimator.Tdiff_);
        LOG(INFO) << "[printStatis] cam2imu time-offset: " << estimator.Tdiff_;
    }
}

void pubOdometry(const VinsEstimator &estimator, const std_msgs::Header &header) {
    if (estimator.flag_solver_type_ == VinsEstimator::SolverFlag::NON_LINEAR) {
        nav_msgs::Odometry odometry;
        // odometry.header = header;
        odometry.header.stamp = ros::Time(estimator.curr_sys_state_time_);
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        // pose_stamped.header = header;
        pose_stamped.header.stamp = ros::Time(estimator.curr_sys_state_time_);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        // ros_path.header = header;
        ros_path.header.stamp = ros::Time(estimator.curr_sys_state_time_);
        ros_path.header.frame_id = "world";
        ros_path.poses.push_back(pose_stamped);
        pub_path.publish(ros_path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(5);
        foutC << estimator.curr_sys_state_time_ << " "
              << estimator.Ps[WINDOW_SIZE].x() << " "
              << estimator.Ps[WINDOW_SIZE].y() << " "
              << estimator.Ps[WINDOW_SIZE].z() << " "
              << tmp_Q.x() << " "
              << tmp_Q.y() << " "
              << tmp_Q.z() << " "
              << tmp_Q.w() << endl;
            //   << estimator.Vs[WINDOW_SIZE].x() << ","
            //   << estimator.Vs[WINDOW_SIZE].y() << ","
            //   << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        // printf("[printStatis] time: %f, t: %f %f %f q: %f %f %f %f \n", estimator.curr_sys_state_time_, 
        //     tmp_T.x(), tmp_T.y(), tmp_T.z(), tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    }
}

void pubKeyPoses(const VinsEstimator &estimator, const std_msgs::Header &header) {
    if (estimator.key_poses_.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    // key_poses.header = header;
    key_poses.header.stamp = ros::Time(estimator.curr_sys_state_time_);
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++) {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses_[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const VinsEstimator &estimator, const std_msgs::Header &header) {
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.flag_solver_type_ == VinsEstimator::SolverFlag::NON_LINEAR) {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.T_IC_[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.R_IC_[0]);

        nav_msgs::Odometry odometry;
        // odometry.header = header;
        odometry.header.stamp = ros::Time(estimator.curr_sys_state_time_);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cam_model_viz.reset();
        cam_model_viz.add_pose(P, R);
        if(STEREO) {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.T_IC_[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.R_IC_[1]);
            cam_model_viz.add_pose(P, R);
        }
        cam_model_viz.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(const VinsEstimator &estimator, const std_msgs::Header &header) {
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    // point_cloud.header = header;
    // loop_point_cloud.header = header;

    point_cloud.header.stamp = ros::Time(estimator.curr_sys_state_time_);
    point_cloud.header.frame_id = "world";

    loop_point_cloud.header.stamp = ros::Time(estimator.curr_sys_state_time_);
    loop_point_cloud.header.frame_id = "world";

    for (auto &it_per_id : estimator.f_manager_.features_) {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.R_IC_[0] * pts_i + estimator.T_IC_[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    // margin_cloud.header = header;
    margin_cloud.header.stamp = ros::Time(estimator.curr_sys_state_time_);
    margin_cloud.header.frame_id = "world";

    for (auto &it_per_id : estimator.f_manager_.features_) {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 ) {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.R_IC_[0] * pts_i + estimator.T_IC_[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

void pubKeyframe(const VinsEstimator &estimator) {
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.flag_solver_type_ == VinsEstimator::SolverFlag::NON_LINEAR && 
        estimator.flag_marglize_type_ == VinsEstimator::MarginalizationFlag::MARGIN_OLD) {

        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.T_IC_[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f \n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager_.features_) {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && 
                it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 
                && it_per_id.solve_flag == 1) {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.R_IC_[0] * pts_i + estimator.T_IC_[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}

void pubTF(const VinsEstimator &estimator, const std_msgs::Header &header) {
    if( estimator.flag_solver_type_ != VinsEstimator::SolverFlag::NON_LINEAR) {
        return;
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(
        correct_t(0), correct_t(1), correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.T_IC_[0].x(),
                                    estimator.T_IC_[0].y(),
                                    estimator.T_IC_[0].z()));
    q.setW(Quaterniond(estimator.R_IC_[0]).w());
    q.setX(Quaterniond(estimator.R_IC_[0]).x());
    q.setY(Quaterniond(estimator.R_IC_[0]).y());
    q.setZ(Quaterniond(estimator.R_IC_[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    nav_msgs::Odometry odometry;
    // odometry.header = header;
    odometry.header.stamp = ros::Time(estimator.curr_sys_state_time_);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.T_IC_[0].x();
    odometry.pose.pose.position.y = estimator.T_IC_[0].y();
    odometry.pose.pose.position.z = estimator.T_IC_[0].z();
    Quaterniond tmp_q{estimator.R_IC_[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);

}



