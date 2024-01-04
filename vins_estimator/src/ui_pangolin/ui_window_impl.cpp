/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#include "ui_window_impl.h"

#include <cstdlib>

// namespace VinsNS {

// *********************************** 初始化 *********************************** //

bool UiWindowImpl::Init(const std::string& win_name) {
    std::cout << "[ Ui Window Impl ] init Ui ... " << std::endl;
    logging_rater.reset(new LogRater(false, 1000));

    // create a window and bind its context to the main thread
    /// 创建一个视窗对象用于后续的显示，指定其id(name)，以便render线程可以通过id来retrive这个视窗
    window_name_ = win_name;
    pangolin::CreateWindowAndBind(window_name_, window_width_, window_height_);

    // 3D mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // opengl buffer (一些gltext对象)
    AllocateBuffer();

    // 初始化一个灰色图像，赋给texture
    std::array<uint8_t, 3 * UI_IMAGE_ROWS * 
        UI_IMAGE_COLS> range_data, intensity_data;
    range_data.fill((unsigned char)100); 
    intensity_data.fill((unsigned char)100); 
    const int size = 3 * UI_IMAGE_ROWS * UI_IMAGE_COLS;
    for (int cnt = 0; cnt < UI_IMAGE_ROWS * UI_IMAGE_COLS; cnt++) {
        int i = rand() % size;
        range_data[i] = rand() % int(UI_IMAGE_RANGE_LIMIT);
        intensity_data[i] = rand() % int(255);
    }
    gltexture_curr_image_.Upload(range_data.data(), GL_RGB, GL_UNSIGNED_BYTE);
    gltexture_intensity_image_.Upload(intensity_data.data(), GL_RGB, GL_UNSIGNED_BYTE);

    // unset the current context from the main thread
    // 窗口在主线程中创建，init完成后，使用RemoveCurrent()将其解绑，以便其它线程使用
    pangolin::GetBoundWindow()->RemoveCurrent();

    // // 轨迹
    // traj_dr_ui_.reset(new ui::UiTrajectory(color_dr_));                 // 白色
    // traj_gins_ui_.reset(new ui::UiTrajectory(color_gins_));             // 黄色
    // traj_lidarloc_ui_.reset(new ui::UiTrajectory(color_lidarloc_));     // 红色
    // traj_laneline_ui_.reset(new ui::UiTrajectory(color_laneline_));     // 天蓝色
    // traj_pgoloc_ui_.reset(new ui::UiTrajectory(color_pgo_));            // 紫色
    // traj_smootherloc_ui_.reset(new ui::UiTrajectory(color_smoother_));  // 绿色
    // traj_bag_vehicle_state_ui_.reset(new ui::UiTrajectory(color_bag_vehicle_state_));  // 橘色

    // current_scan_.reset(new PointCloudXYZI());

    /// data log
    dlog_wheel_vel_.SetLabels(std::vector<std::string>{"Wheel: vel0", "vel1", "vel2", "vel3"});
    dlog_imu_acc_.SetLabels(std::vector<std::string>{"IMU: acc_x", "acc_y", "acc_z"});
    dlog_imu_gyr_.SetLabels(std::vector<std::string>{"IMU: gyr_x", "gyr_y", "gyr_z"});

    dlog_lio_rp_.SetLabels(std::vector<std::string>{"roll(deg)", "pitch(deg)"});
    dlog_lio_yaw_.SetLabels(std::vector<std::string>{"yaw(deg)"});

    dlog_lio_pos_.SetLabels(std::vector<std::string>{"pos_x", "pos_y", "pos_z"});
    dlog_lio_vel_.SetLabels(std::vector<std::string>{"wheel_vel", "lio_vel"});
    dlog_bias_acc_.SetLabels(std::vector<std::string>{"ba_x", "ba_y", "ba_z"});
    dlog_bias_gyr_.SetLabels(std::vector<std::string>{"bg_x", "bg_y", "bg_z"});
    dlog_state_grav_.SetLabels(std::vector<std::string>{"grav_x", "grav_y", "grav_z"});
    dlog_lio_confi_.SetLabels(std::vector<std::string>{"match_confidence"});

    dlog_time_usage_.SetLabels(std::vector<std::string>{"lkTrack(ms)", "procMeas(ms)", "Visualz(ms)", "Sum(ms)"});

    return true;

}

bool UiWindowImpl::UnInit() {
    std::cout << "[ Ui Window Impl ] uninit Ui ... " << std::endl;
    //
    return true;
}

void UiWindowImpl::AllocateBuffer() {

    // (重新)初始化glTexture容器用于读取图像
    gltexture_curr_image_.Reinitialise(UI_IMAGE_ROWS, UI_IMAGE_COLS, 
        GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    gltexture_intensity_image_.Reinitialise(UI_IMAGE_ROWS, UI_IMAGE_COLS, 
        GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    // // clang-format off
    // auto& font = pangolin::default_font();
    // gltext_label_global_        = font.Text("Welcome to VINS-Understood. All right reserved.");
    // gltext_label_gps_           = font.Text("Traj GINS            ---");
    // gltext_label_dr_loc_        = font.Text("Traj DR Loc          ---");
    // gltext_label_lidar_loc_     = font.Text("Traj Lidar Loc       ---");
    // gltext_label_laneline_      = font.Text("Traj LANEMATCH Loc   ---");
    // gltext_label_pgo_loc_       = font.Text("Traj PGO Loc         ---");
    // gltext_label_smoother_loc_  = font.Text("Traj Smoother Loc    ---");
    // gltext_bag_vehicle_state_   = font.Text("Traj Bag Loc         ---");

    // gltext_gins_                = font.Text("STATUS: [3 in 1] gins                ");
    // gltext_lidar_               = font.Text("STATUS: [3 in 1] lidar               ");
    // gltext_laneline_            = font.Text("STATUS: [3 in 1] laneline            ");
    // gltext_lidar_gins_          = font.Text("STATUS: [4 in 1] gins,lidar          ");
    // gltext_laneline_gins_       = font.Text("STATUS: [4 in 1] gins,laneline       ");
    // gltext_lidar_laneline_      = font.Text("STATUS: [4 in 1] lidar,laneline      ");
    // gltext_lidar_gins_laneline_ = font.Text("STATUS: [5 in 1] gins,lidar,laneline ");
    // gltext_other_               = font.Text("STATUS: [0 in 0] unlocked / other    ");

    // gltext_pgo_status_title_    = font.Text("PGO Sliding-Window Status ");
    // gltext_pgo_time_data_       = font.Text("Delayed: x.xxx s ");
    // gltext_pgo_gins_status_     = font.Text("GINS Overall Weight: x.x, Inliers: x/x");
    // gltext_pgo_lidar_status_    = font.Text("LidarLoc RecentAve x.x, HReliable xx, xx");
    // gltext_pgo_laneline_status_ = font.Text("LaneLoc Status: not set ");
    // gltext_pgo_overall_status_  = font.Text("IsInMap: xxx, IsInHdMap: xxx");
    // pgo_time_data_fmt_      = boost::format("Delayed: %.3f s");
    // pgo_gins_status_fmt_    = boost::format("GINS Overall Weight: %.1f, Inliers: x/x");
    // pgo_lidar_status_fmt_   = boost::format("LidarLoc RecentAve %.1f, HReliable %d, %d");
    // pgo_overall_status_fmt_ = boost::format("IsInMap: %d, IsInHdMap: %d ");

    // gltext_full_split_line_     = font.Text("-----------------------------------------");
    // gltext_short_split_line_    = font.Text("----------- ");

    // gltext_system_title_        = font.Text("Tfusion System Status");
    // gltext_system_duration_     = font.Text("System Runing Duration: xxh xxm xxs ");
    // gltext_system_kilometers_   = font.Text("System Runing Distance: x.xxx km ");
    // system_duration_fmt_    = boost::format("System Runing Duration: %dh %dm %ds ");
    // system_kilometers_fmt_  = boost::format("System Runing Distance: %.3f km ");

    // gltext_time_statistics_     = font.Text("Modules Time Consumption             ");
    // gltext_laneloc_time_        = font.Text("Lane Loc : Mean Time x.xx ms");
    // laneloc_time_fmt_       = boost::format("Lane Loc : Mean Time %.2f ms");

    // gltext_gins_status_         = font.Text("GINS Status: not set                 ");
    // gltext_gnss_rtk_status_     = font.Text("GNSS/RTK Status: not set             ");

    // gltext_gins_yaw_status_     = font.Text("GINS Yaw Online Calib Info           ");
    // gltext_gins_calib_time_     = font.Text("GINS Yaw Calib Time : not set        ");
    // gltext_gins_calib_value_    = font.Text("GINS Yaw Calib Value: not set        ");
    // gins_yaw_time_fmt_      = boost::format("GINS Yaw Calib Time : %.2f min ago   ");
    // gins_yaw_value_fmt_     = boost::format("GINS Yaw Calib Value: %.2f degree    ");

    // gltext_lidarloc_title_      = font.Text("Lidar Loc Status, Method -- Unknown  ");
    // gltext_lidarloc_time_data_  = font.Text("Duration: xxx ms, Delayed: xx.xx s   ");
    // gltext_lidarloc_status_     = font.Text("Status: success / failed / out of map");
    // gltext_lidarloc_scores_     = font.Text("Scores: Ave xx.x, Inliers Ave xx.x   ");
    // gltext_lidarloc_inliers_    = font.Text("Inliers: Num xxxx / xxxx, Ratio xx.x%");
    // gltext_lidarloc_poses1_     = font.Text("Alignment: dX x.xx, dY x.xx, dZ x.xx ");
    // gltext_lidarloc_poses2_     = font.Text("Alignment: dR x.xx, dP x.xx, dY x.xx ");
    // gltext_lidarloc_chunkid_    = font.Text("ChunkId: [xxxxx, xxxxx], N13: xx     ");
    // gltext_lidarloc_jacobian_   = font.Text("Jacobi: x.x, x.x, x.x, x.x, x.x, x.x ");
    // gltext_lidarloc_hessian1_   = font.Text("HesseD: x.x, x.x, x.x, x.x, x.x, x.x ");
    // gltext_lidarloc_hessian2_   = font.Text("HesseE: x.x, x.x, x.x, x.x, x.x, x.x ");

    // lidarloc_time_data_fmt_ = boost::format("Duration: %d ms, Delayed: %.3f s");
    // lidarloc_status_fmt_    = boost::format("Status: in / out of map scope");
    // lidarloc_scores_fmt_    = boost::format("Scores: Ave %.2f, Inliers Ave %.2f");
    // lidarloc_inliers_fmt_   = boost::format("Inliers: Num %d / %d, Ratio %.1f%% ");
    // lidarloc_poses1_fmt_    = boost::format("Alignment: dX %.2f, dY %.2f, dZ %.2f");
    // lidarloc_poses2_fmt_    = boost::format("Alignment: dR %.2f, dP %.2f, dY %.2f");
    // lidarloc_chunkid_fmt_   = boost::format("ChunkId: [%.1f, %.1f], N13: %d");
    // lidarloc_jacobian_fmt_  = boost::format("Jacobi: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f");
    // lidarloc_hessian1_fmt_  = boost::format("HesseD: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f");
    // lidarloc_hessian2_fmt_  = boost::format("HesseE: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f");
    // lidarloc_status_color_ = Vec3i(205, 250, 255);

    // gltext_lidarloc_solution_title_    = font.Text("Lidar Loc, Try Other Solution, xx / xx");
    // gltext_lidarloc_solution_status1_  = font.Text("Result: xxx better, Scores: x.x vs x.x");
    // gltext_lidarloc_solution_status2_  = font.Text("Pose Diff: x.xx, x.xx, x.xx, x.xx deg");
    // lidarloc_solution_title_fmt_   = boost::format("Lidar Loc, Try Other Solution, %d / %d");
    // lidarloc_solution_status1_fmt_ = boost::format("Result: %s, Scores: %.1f vs %.1f");
    // lidarloc_solution_status2_fmt_ = boost::format("Pose Diff: %.2f, %.2f, %.2f, %.1f deg");

    // gltext_lidarloc_vs_gins_error_     = font.Text("vs GINS Error: x.xx, x.xx, x.xx, x.x");
    // lidarloc_vs_gins_error_fmt_    = boost::format("vs GINS Error: %.2f, %.2f, %.2f, %.1f");

    // gltext_laneline_distance_     = font.Text("distance to left and right: x.xx, x.xx");
    // laneline_distance_fmt_    = boost::format("distance to left and right: %.2f, %.2f");

    // gltext_lidarloc_reset_status_      = font.Text("Lidar Loc, Reset Last Pose, Counts 0");


    // clang-format on
}

void UiWindowImpl::ReleaseBuffer() {}

void UiWindowImpl::CreateDisplayLayout() {

    /** NOTE: 在当前线程中创建窗口布局 */

    // define camera render object (for view / scene browsing)
    /// 给定内参矩阵ProjectionMatrix，以便对摄像机进行交互操作时，Pangolin会自动根据内参矩阵完成对应的透视变换
    /// 内参矩阵的含义就是常规相机内参，分别为图像的高度、宽度、4个内参以及最近和最远视距，含义是清晰的
    /// 还需指定当前摄像机所处的位置、视点位置（即摄像机的光轴朝向哪一个点）、以及摄像机本身哪一个轴朝上，含义很清晰
    auto cam3d_proj_mat = pangolin::ProjectionMatrix(window_width_, window_width_, 
        cam_focus_, cam_focus_, window_width_ / 2, window_width_ / 2, cam_z_near_, cam_z_far_);
    auto cam3d_view_point = pangolin::ModelViewLookAt(reset_3d_view_look_from_[0], 
        reset_3d_view_look_from_[1], reset_3d_view_look_from_[2], 0, 0, 0, pangolin::AxisY);
    s_cam3d_observer_ = pangolin::OpenGlRenderState(std::move(cam3d_proj_mat), std::move(cam3d_view_point));

    // Add named OpenGL viewport to window and provide 3D Handler
    /// 创建一个交互式视图（view）用于显示上一步摄像机所“拍摄”到的内容，这一步类似于OpenGL中的viewport处理
    /// setBounds()函数指定视图在视窗中的范围（下、上、左、右），可以采用相对坐标（0~1）以及绝对坐标（使用Attach对象）
    pangolin::View &view_cam3d = pangolin::Display(dis_cam3d_name_)
                                    .SetBounds(0.0, 1.0, 0.0, 1.0)
                                    .SetHandler(new pangolin::Handler3D(s_cam3d_observer_));

    pangolin::View &d_cam3d_main = pangolin::Display(dis_cam3d_main_name_)
                                    .SetBounds(0.0, sensor_bottom, 0.0, plotter_left)
                                    .SetLayout(pangolin::LayoutOverlay)
                                    .AddDisplay(view_cam3d);

    // OpenGL 'view' of data, these plotters are very useful for tuning! [data，左右下上，横刻度尺，纵刻度尺]
    const Vec3f medidark = Vec3f(0.15, 0.15, 0.15);
    const Vec3f deepgrey = Vec3f(0.2, 0.2, 0.2);
    const Vec3f medigrey = Vec3f(0.3, 0.3, 0.3);
    const Vec3f lightgrey = Vec3f(0.4, 0.4, 0.4);
    plotter_wheel_vel_ = boost::make_unique<pangolin::Plotter>(&dlog_wheel_vel_, -10, 1000, -10, 10, 100, 5.0); //std::make_unique<>要到C++14才支持
    plotter_wheel_vel_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_wheel_vel_->Track("$i");
    plotter_imu_acc_ = boost::make_unique<pangolin::Plotter>(&dlog_imu_acc_, -10, 1000, -16.0, 16.0, 100, 0.1);
    plotter_imu_acc_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_imu_acc_->Track("$i");
    plotter_imu_gyr_ = boost::make_unique<pangolin::Plotter>(&dlog_imu_gyr_, -10, 1000, -0.8, 0.8, 100, 0.1);
    plotter_imu_gyr_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_imu_gyr_->Track("$i");

    plotter_lio_rp_ = boost::make_unique<pangolin::Plotter>(&dlog_lio_rp_, -10, 1000, -M_PI, M_PI, 75, 0.1);
    plotter_lio_rp_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_lio_rp_->Track("$i");
    plotter_lio_yaw_ = boost::make_unique<pangolin::Plotter>(&dlog_lio_yaw_, -10, 1000, -M_PI, M_PI, 75, 0.1);
    plotter_lio_yaw_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_lio_yaw_->Track("$i");
    plotter_lio_pos_ = boost::make_unique<pangolin::Plotter>(&dlog_lio_pos_, -10, 1000, -100, 100, 75, 10);
    plotter_lio_pos_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_lio_pos_->Track("$i");
    plotter_lio_vel_ = boost::make_unique<pangolin::Plotter>(&dlog_lio_vel_, -10, 1000, -10, 10, 75, 10);
    plotter_lio_vel_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_lio_vel_->Track("$i");

    plotter_bias_acc_ = boost::make_unique<pangolin::Plotter>(&dlog_bias_acc_, -10, 1000, -2.0, 2.0, 75, 0.1);
    plotter_bias_acc_->SetBackgroundColour(pangolin::Colour(medidark[0], medidark[1], medidark[2]));
    plotter_bias_acc_->SetAxisColour(pangolin::Colour(lightgrey[0], lightgrey[1], lightgrey[2]));
    plotter_bias_acc_->SetTickColour(pangolin::Colour(medigrey[0], medigrey[1], medigrey[2]));
    plotter_bias_acc_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_bias_acc_->Track("$i");

    plotter_bias_gyr_ = boost::make_unique<pangolin::Plotter>(&dlog_bias_gyr_, -10, 1000, -0.2, 0.2, 75, 0.01);
    plotter_bias_gyr_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_bias_gyr_->Track("$i");

    plotter_state_grav_ = boost::make_unique<pangolin::Plotter>(&dlog_state_grav_, -10, 1000, -5, 10, 75, 1.0);
    plotter_state_grav_->SetBackgroundColour(pangolin::Colour(medidark[0], medidark[1], medidark[2]));
    plotter_state_grav_->SetAxisColour(pangolin::Colour(lightgrey[0], lightgrey[1], lightgrey[2]));
    plotter_state_grav_->SetTickColour(pangolin::Colour(medigrey[0], medigrey[1], medigrey[2]));
    plotter_state_grav_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_state_grav_->Track("$i");

    plotter_lio_confi_ = boost::make_unique<pangolin::Plotter>(&dlog_lio_confi_, -10, 1000, 0, 10.0, 75, 2.0);
    plotter_lio_confi_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_lio_confi_->Track("$i");

    plotter_time_usage_ = boost::make_unique<pangolin::Plotter>(&dlog_time_usage_, -10, 1000, 0, 100.0, 75, 20);
    plotter_time_usage_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_time_usage_->Track("$i");

    pangolin::View &d_plot_sensor = pangolin::Display(dis_plot_sensor_name_)
                                .SetBounds(sensor_bottom, sensor_top, 0.0, plotter_left)
                                .SetLayout(pangolin::LayoutEqualVertical)
                                .SetLock(pangolin::LockLeft, pangolin::LockTop)
                                .AddDisplay(*plotter_imu_acc_)
                                .AddDisplay(*plotter_imu_gyr_);

    pangolin::View &d_plot_state = pangolin::Display(dis_plot_state_name_)
                                .SetBounds(0.0, 1.0, plotter_left, plotter_right)
                                .SetLayout(pangolin::LayoutEqualVertical)
                                .AddDisplay(*plotter_lio_vel_)
                                // .AddDisplay(*plotter_lio_rp_)
                                .AddDisplay(*plotter_bias_acc_)
                                .AddDisplay(*plotter_bias_gyr_)
                                .AddDisplay(*plotter_state_grav_)
                                // .AddDisplay(*plotter_lio_confi_)
                                .AddDisplay(*plotter_time_usage_);


    // pangolin::View &view_range_img = pangolin::Display(dis_range_img_name)
    //                             .SetBounds(pangolin::Attach::Pix(0), pangolin::Attach::Pix(UI_IMAGE_ROWS), 
    //                                         pangolin::Attach::Pix(0), pangolin::Attach::Pix(UI_IMAGE_COLS))
    //                             // .SetAspect(1.0 * UI_IMAGE_COLS / UI_IMAGE_ROWS);
    //                             .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // pangolin::View &view_intensity_img = pangolin::Display(dis_intens_img_name)
    //                             .SetBounds(pangolin::Attach::Pix(UI_IMAGE_ROWS+5), pangolin::Attach::Pix(UI_IMAGE_ROWS*2+5), 
    //                                         pangolin::Attach::Pix(0), pangolin::Attach::Pix(UI_IMAGE_COLS))
    //                             // .SetAspect(1.0 * UI_IMAGE_COLS / UI_IMAGE_ROWS)
    //                             .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // 在主窗口中绘制哪些ui组件
    pangolin::Display(dis_main_name_)
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(menu_panel_width_), 1.0)
        .AddDisplay(d_plot_state)
        .AddDisplay(d_plot_sensor)
        // .AddDisplay(view_range_img)
        // .AddDisplay(view_intensity_img)
        .AddDisplay(d_cam3d_main);

    auto widthhh = pangolin::Display(dis_main_name_).v.w;
    auto heightt = pangolin::Display(dis_main_name_).v.h;
}

void UiWindowImpl::RenderWindow() {

    /** NOTE: 注意这个函数是在单独线程中运行的 */

    // fetch the context and bind it to this thread
    /// 在当前线程中重新获取视窗对象
    pangolin::BindToContext(window_name_);

    // Issue specific OpenGl we might need
    glEnable(GL_DEPTH_TEST);/*启动深度测试，以便pangolin只绘制镜头前方的像素点，避免后方像素点混淆界面，3D绘制都应当启用此选项*/
    glEnable(GL_BLEND);     /*启用颜色混合，也即视线上的颜色是该视线上所有颜色的混合，3D绘制都应当启用此选项*/
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  /*指定颜色混合的计算方法*/

    // menu
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(menu_panel_width_));
    pangolin::Var<bool> menu_ui_smaller_point("menu.Small Point Size", true, true);
    pangolin::Var<bool> menu_view_from_far("menu.Look From Faraway", false, true);
    pangolin::Var<bool> menu_follow_loc("menu.Follow", true, true);
    pangolin::Var<bool> menu_ui_pause("menu.Ui Pause", false, true);
    pangolin::Var<bool> menu_reset_3d_view("menu.set TopDown View", false, false);
    pangolin::Var<bool> menu_reset_front_view("menu.Set Front View", false, false);

    pangolin::Var<bool> menu_show_lio_traj("menu.Show VIO Traj", true, true);
    pangolin::Var<bool> menu_show_dr_traj("menu.Show DR Traj", false, true);
    pangolin::Var<bool> menu_reset_dr_traj("menu.Reset DR Traj", false, false);
    pangolin::Var<bool> menu_show_glob_traj("menu.Show Glob Traj", true, true);
    pangolin::Var<bool> menu_show_image("menu.Show Range Image", false, true);
    pangolin::Var<bool> menu_enhance_image("menu.Image Enhancement", false, true);

    // pangolin::Var<bool> menu_debug_pause("menu.Debug Pause", false, false);
    pangolin::Var<bool> menu_debug_next("menu.Debug VIO Next", false, false);

    pangolin::Var<double> menu_speedX_value("menu.speed x: ", 0, -30.0, 30.0);
    pangolin::Var<bool> menu_set_speed_values("menu.Set Speed Values", false, false);
    pangolin::Var<bool> menu_reset_speed_values("menu.Reset Speed Values", false, false);

    pangolin::Var<double> menu_transl_Tx_value("menu.transl x: ", 0, -30.0, 30.0);
    pangolin::Var<double> menu_orient_Roll_value("menu.orient roll: ", 0, -180.0, 180.0);
    pangolin::Var<bool> menu_set_pose_values("menu.Set Pose Values", false, false);
    pangolin::Var<bool> menu_reset_pose_values("menu.Reset Pose Values", false, false);


    // display layout
    CreateDisplayLayout();

    while (!pangolin::ShouldQuit() && !flag_ui_exit_.load()) {

        // fetch the context and bind it to this thread
        pangolin::BindToContext(window_name_);

        // 清空颜色缓存和深度缓存，如果不清空就会保留上一帧的图形
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // menu control

        // const bool ui_pause = menu_ui_pause;
        // pangolin_ui_pause_.store(ui_pause);
        // following_loc_ = menu_follow_loc;
        // show_dr_traj_ = menu_show_dr_traj;
        // show_gins_traj_ = menu_show_glob_traj;
        // show_lidarloc_traj_ = menu_show_lio_traj;
        // debug_lidarloc_ = menu_show_lio_details;

        

        const bool view_from_faraway = false;
        if (view_from_faraway) {
            // 景深显示:近似平行投影
            cam_focus_ = 5000;
            cam_z_near_ = 1.0;
            reset_3d_view_look_from_ = Vec3f(0, 0, 300);
            reset_front_view_look_from_ = Vec3f(-10, 0, 10);
            reset_front_view_look_at_ = Vec3f(50, 0, 10);
        } else {
            // 景深显示:近似透视投影
            cam_focus_ = 1000;
            cam_z_near_ = 0.5;
            reset_3d_view_look_from_ = Vec3f(0, 0, 50);
            reset_front_view_look_from_ = Vec3f(-10, 0, 10);
            reset_front_view_look_at_ = Vec3f(50, 0, 10);
        }

        // const bool ui_small_pt_size = menu_ui_smaller_point;
        // if (ui_small_pt_size) {
        //     map_pt_size_ui_ = 1;
        // } else {
        //     map_pt_size_ui_ = 2;
        // }

        if (menu_reset_3d_view) {
            s_cam3d_observer_.SetModelViewMatrix(pangolin::ModelViewLookAt(reset_3d_view_look_from_[0], 
                reset_3d_view_look_from_[1], reset_3d_view_look_from_[2], 0, 0, 0, pangolin::AxisY));
            menu_reset_3d_view = false;
        }
        if (menu_reset_front_view) {
            s_cam3d_observer_.SetModelViewMatrix(pangolin::ModelViewLookAt(
                reset_front_view_look_from_[0], reset_front_view_look_from_[1], reset_front_view_look_from_[2], 
                reset_front_view_look_at_[0], reset_front_view_look_at_[1], reset_front_view_look_at_[2], pangolin::AxisZ));
            menu_reset_front_view = false;
        }

        static bool last_choice = false;
        const bool choice = menu_show_image;

        // 在这里执行所有的更新
        UpdateUiWheel();
        UpdateUiIMU();
        // UpdateTrackedImages();
        UpdateTrackedFeatures();
        UpdateUiVioState();
        UpdateUiSysRunStatus();

        // 在这里执行所有的渲染
        // RenderRangeImage();
        // RenderPlotterLayout();

        // Swap frames and Process Events (对前边设置的所有显示元素进行一次显示)
        pangolin::FinishFrame();
    }

    std::cout << "[ Ui Window Impl ] rendering loop terminated." << std::endl;

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();

}


// *********************************** 模块级更新 *********************************** //

bool UiWindowImpl::UpdateUiWheel() {
    if (!wheel_need_update_.load()) {
        return false;
    }

    // 读写访问中间变量，然后释放锁
    {
        std::lock_guard<std::mutex> lock(mtx_raw_wheel_);
        const auto& speeds = curr_wheel_.wheel_speed;
        dlog_wheel_vel_.Log(speeds[0], speeds[1], speeds[2], speeds[3]);
        current_wheel_vel_ = speeds.mean();
        if (!vio_state_received_) {
            dlog_lio_vel_.Log(current_wheel_vel_, current_lio_vel_);
        }
        wheel_need_update_.store(false);
    }

    return true;
}

bool UiWindowImpl::UpdateUiIMU() {
    if (!imu_need_update_.load()) {
        return false;
    }

    // 读写访问中间变量，然后释放锁
    {
        std::lock_guard<std::mutex> lock(mtx_raw_imu_);
        dlog_imu_acc_.Log(curr_imu_.linear_acceleration[0], 
            curr_imu_.linear_acceleration[1], curr_imu_.linear_acceleration[2]);
        dlog_imu_gyr_.Log(curr_imu_.angular_velocity[0], 
            curr_imu_.angular_velocity[1], curr_imu_.angular_velocity[2]);
        imu_need_update_.store(false);
    }

    return true;
}

bool UiWindowImpl::UpdateTrackedImages() {
    if (!curr_image_need_update_.load()) {
        return false;
    }

    // 读写访问中间变量，然后释放锁
    cv::Mat image_deep_copy;
    {
        std::lock_guard<std::mutex> lock(mtx_curr_image_);
        image_deep_copy = curr_tracked_image_.clone();
        curr_image_need_update_.store(false);
    }

    // // 渲染pangolin格式的texture图像
    // std::array<uint8_t, 3 * UI_IMAGE_ROWS * UI_IMAGE_COLS> range_data, intensity_data;
    // range_data.fill((unsigned char)UI_IMAGE_RANGE_LIMIT);
    // intensity_data.fill((unsigned char)0);
    // if (image_deep_copy.type() == CV_32FC3) {
    //     for (int row = 0; row < image_deep_copy.rows; row++) {
    //         const int start = 3 * UI_IMAGE_COLS * row;
    //         for (int col = 0; col < image_deep_copy.cols; col++) {
    //             const float pt_intensity = image_deep_copy.at<cv::Vec3f>(row, col)[0];
    //             const float pt_range = image_deep_copy.at<cv::Vec3f>(row, col)[1];
    //             const float pt_time = image_deep_copy.at<cv::Vec3f>(row, col)[2];
    //             const int head = start + 3 * col;
    //             auto range_rgb = RangeValueColorMap(pt_range);
    //             range_data[head+0] = range_rgb[0];
    //             range_data[head+1] = range_rgb[1];
    //             range_data[head+2] = range_rgb[2];
    //             auto intensity_rgb = IntensityValueColorMap(pt_intensity);
    //             intensity_data[head+0] = intensity_rgb[0];
    //             intensity_data[head+1] = intensity_rgb[1];
    //             intensity_data[head+2] = intensity_rgb[2];
    //         }
    //     }
    //     // 向GPU装载图像
    //     gltexture_curr_image_.Upload(range_data.data(), GL_RGB, GL_UNSIGNED_BYTE);
    //     gltexture_intensity_image_.Upload(intensity_data.data(), GL_RGB, GL_UNSIGNED_BYTE);
    //     static int update_imgs_counts = 0;
    //     std::cout << "[ Ui Window Impl ] updated range image, counts=" << update_imgs_counts++ << std::endl;

    // } else {
    //     std::cout << "[ Ui Window Impl ] range image not CV_32FC3, fatal error." << std::endl;
    // }

    return true;
}

bool UiWindowImpl::UpdateTrackedFeatures() {
    //
    return true;
}

bool UiWindowImpl::UpdateUiVioState() {
    if (!vio_state_need_update_.load()) {
        return false;
    }

    // 读写访问中间变量，然后释放锁
    VioState state_show;
    {
        std::lock_guard<std::mutex> lock(mtx_vio_state_);
        state_show = curr_vio_state_;
        vio_state_need_update_.store(false);
        vio_state_received_ = true;
    }

    double visualz_usage = 
        state_show.visualz_imgLK_usage_ + 
        state_show.visualz_state_usage_;
    state_show.latest_whole_usage_ = 
        state_show.latest_tracking_usage_ +
        state_show.latest_procmeas_usage_ + 
        state_show.visualz_imgLK_usage_ + 
        state_show.visualz_state_usage_;
    current_lio_vel_ = state_show.velocity_.norm();
    dlog_lio_vel_.Log(current_wheel_vel_, current_lio_vel_);
    const auto& biasAcc = state_show.bias_acc_;
    dlog_bias_acc_.Log(biasAcc[0], biasAcc[1], biasAcc[2]);
    const auto& biasGyr = state_show.bias_gyr_;
    dlog_bias_gyr_.Log(biasGyr[0], biasGyr[1], biasGyr[2]);
    const auto gravVec = state_show.grav_;
    dlog_state_grav_.Log(gravVec[0], gravVec[1], gravVec[2]);
    dlog_lio_confi_.Log(0);
    dlog_time_usage_.Log(state_show.latest_tracking_usage_, 
        state_show.latest_procmeas_usage_, visualz_usage, 
        state_show.latest_whole_usage_);

    return true;
}

bool UiWindowImpl::UpdateUiSysRunStatus() {
    if (!run_status_need_update_.load()) {
        return false;
    }

    // 读写访问中间变量，然后释放锁
    {
        std::lock_guard<std::mutex> lock(mtx_run_status_);
        const double all_dura = (curr_run_status_.lidar_preproc_ + curr_run_status_.lio_pipeline_) * 1000;
        // dlog_time_usage_.Log(all_dura, curr_run_status_.lio_pipeline_ * 1000, curr_run_status_.ieskf_sum_ * 1000);
        run_status_need_update_.store(false);
    }

    return true;
}

void UiWindowImpl::RenderRangeImage() {
    pangolin::Display(dis_range_img_name).Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    gltexture_curr_image_.RenderToViewport();

    pangolin::Display(dis_intens_img_name).Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    gltexture_intensity_image_.RenderToViewport();
}

void UiWindowImpl::RenderPlotterLayout() {
    //
}


// } // namespace VinsNS