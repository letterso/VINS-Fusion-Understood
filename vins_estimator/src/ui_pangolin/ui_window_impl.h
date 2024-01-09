/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef VINS_UI_WINDOW_IMPL_H_
#define VINS_UI_WINDOW_IMPL_H_

// pangolin先于pcl, 消除HAVE_OPENNI的编译警告
#include <pangolin/pangolin.h>

#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <boost/format.hpp>
#include <boost/make_unique.hpp>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "../utility/common.h"
#include "../utility/sensor_type.h"


// namespace VinsNS {

#define UI_IMAGE_ROWS (128)
#define UI_IMAGE_COLS (1400)
#define UI_IMAGE_RANGE_LIMIT (200.f)

/// @brief 注意，ui作为工具属性，我们要求其在满足多线程的同时，
/// 要尽己所能避免任何阻塞，所以严禁所有的数据共用同一个数据锁！
class UiWindowImpl {
   public: /* ********* 公共接口 ********* */
    
    using Vec3f = Eigen::Vector3f;

    UiWindowImpl() = default;
    ~UiWindowImpl() = default;

    /// 初始化，创建各种点云、小车实体
    bool Init(const std::string& win_name = "VINS.UI");

    /// 注销ui
    bool UnInit();

    /// 创建一个循环，渲染所有信息
    void RenderWindow();


   public: /* ********* 公共成员，供外层类直接写入，多线程保护 ********* */
    /// 后台渲染线程
    std::thread render_thread_;

    /// 读写锁和原子量
    std::mutex mtx_raw_imu_;
    std::mutex mtx_raw_wheel_;
    std::mutex mtx_curr_image_;
    std::mutex mtx_scan_in_lidar_;
    std::mutex mtx_scan_in_world_;
    std::mutex mtx_vio_state_;
    std::mutex mtx_run_status_;

    std::atomic<bool> flag_ui_exit_;
    std::atomic<bool> imu_need_update_;
    std::atomic<bool> wheel_need_update_;
    std::atomic<bool> curr_image_need_update_;
    std::atomic<bool> lidar_scan_need_update_;
    std::atomic<bool> vio_state_need_update_;
    std::atomic<bool> run_status_need_update_;

    /// 多线程读写变量，读写保护
    ImuData curr_imu_;
    WheelData curr_wheel_;
    cv::Mat curr_tracked_image_;
    std::vector<Vec3f> curr_tracked_feats_;
    VioStatus curr_vio_state_;
    RunningStatus curr_run_status_;


   private: /* ********* 私有函数 ********* */

    /// 创建OpenGL Buffers
    void AllocateBuffer();
    void ReleaseBuffer();

    /// 创建ui布局
    void CreateDisplayLayout();

    /// 渲染各个模块的信息
    bool RenderWheelMsgs();
    bool RenderIMUMsgs();
    bool RenderTrackedImages();
    bool RenderTrackedFeatures();
    bool RenderVioStatus();
    bool RenderRunningStatus();


   private: /* ********* 私有成员，无需多线程保护 ********* */

    void RenderRangeImage();
    void RenderPlotterLayout();

    /// Ui内部数据,无需加锁
    bool vio_state_received_ = false;
    double curr_wheel_vel_ = 0;
    Eigen::Vector3d curr_vio_vel_ = Eigen::Vector3d::Zero();

    /// 内部设施
    std::shared_ptr<LogRater> logging_rater;


    /// 窗口layout相关 ************************* //
    std::string window_name_ = "VINS.UI";
    int window_width_ = 1200;
    int window_height_ = 900;
    int menu_panel_width_ = 200;
    // int plotter_panel_width_ = 400;
    // int sensors_panel_height_ = 250;
    const float sensor_top = 1.0;
    const float sensor_bottom = 0.5;
    const float plotter_left = 0.5;
    const float plotter_right = 1.0;

    /// 观察相机参数
    float cam_focus_ = 1000;
    float cam_z_near_ = 0.5;
    float cam_z_far_ = 1e8;
    Vec3f reset_3d_view_look_from_ = Vec3f(0, 0, 50);
    Vec3f reset_front_view_look_from_ = Vec3f(-10, 0, 10);
    Vec3f reset_front_view_look_at_ = Vec3f(50, 0, 10);

    const std::string dis_main_name_ = "Window Main";  //除了按钮面板外的整个window部分
    const std::string dis_cam3d_name_ = "Cam 3D View";
    const std::string dis_cam3d_main_name_ = "Cam 3D Main";
    const std::string dis_plot_sensor_name_ = "Plot Sensor";
    const std::string dis_plot_state_name_ = "Plot State";
    const std::string dis_range_img_name = "Show Range Image";
    const std::string dis_intens_img_name = "Show Intensity Image";

    /// camera, 3D窗口的观察相机
    pangolin::OpenGlRenderState s_cam3d_observer_;

    /// image textures
    pangolin::GlTexture gltexture_curr_image_;
    pangolin::GlTexture gltexture_intensity_image_;
    // pangolin::GlTexture gltexture_camera_image_;

    // static constexpr int score_img_resolution_ = 10;  // resolution不能是奇数，可能有些内存padding的问题
    // static constexpr int lidarloc_search_step_ = 10;
    // using LidarLocScoreImg = UiImageRgb<score_img_resolution_, lidarloc_search_step_, lidarloc_search_step_>;
    // LidarLocScoreImg lidarloc_score_img_;

    // text, 按需取用
    pangolin::GlText gltext_label_global_;
    pangolin::GlText gltext_sensor_timeout_;
    pangolin::GlText gltext_bag_name_;

    // 统一管理所有颜色
    Vec3f color_dr_ = Vec3f(1.0, 1.0, 1.0);             // 白色
    Vec3f color_gins_ = Vec3f(1.0, 1.0, 51.0 / 255.0);  // 黄色
    Vec3f color_lidarloc_ = Vec3f(1.0, 0.0, 0.0);       // 红色
    Vec3f color_laneline_ = Vec3f(0.0, 0.7, 1.0);       // 天蓝色
    Vec3f color_pgo_ = Vec3f(1.0, 0.0, 1.0);            // 紫色
    Vec3f color_smoother_ = Vec3f(0.0, 1.0, 0.0);       // 绿色
    Vec3f color_other_ = Vec3f(0.3, 0.3, 0.3);          // 暗灰色
    Vec3f color_bag_loc_ = Vec3f(1.0, 0.5, 0.0);        // 橘色

    // 滤波器状态相关 Data logger object
    pangolin::DataLog dlog_wheel_vel_;      // 轮速计的速度
    pangolin::DataLog dlog_imu_acc_;        // 
    pangolin::DataLog dlog_imu_gyr_;        // 
    pangolin::DataLog dlog_lio_rp_;         // odom
    pangolin::DataLog dlog_lio_yaw_;        // 航向角
    pangolin::DataLog dlog_lio_pos_;        // odom
    pangolin::DataLog dlog_lio_vel_;        // frame下的速度
    pangolin::DataLog dlog_bias_acc_;       //
    pangolin::DataLog dlog_bias_gyr_;       //
    pangolin::DataLog dlog_state_grav_;     // odom
    pangolin::DataLog dlog_lio_confi_;      // 
    pangolin::DataLog dlog_time_usage_;     // 

    std::unique_ptr<pangolin::Plotter> plotter_wheel_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_imu_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_imu_gyr_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_lio_rp_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_lio_yaw_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_lio_pos_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_lio_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_gyr_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_state_grav_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_lio_confi_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_time_usage_ = nullptr;

};


// } // namespace VinsNS

#endif // VINS_UI_WINDOW_IMPL_H_