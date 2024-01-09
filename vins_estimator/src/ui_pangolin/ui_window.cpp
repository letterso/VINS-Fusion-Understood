/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#include "ui_window.h"

// namespace VinsNS {

UiWindow::UiWindow() {
    impl_ = std::make_shared<UiWindowImpl>();
    std::cout << "[ Ui Window ] constructed. " << std::endl;
    
}

UiWindow::~UiWindow() {
    UnInit();
    std::cout << "[ Ui Window ] destructed." << std::endl;

}

bool UiWindow::Init() {
    std::cout << "[ Ui Window ] init Ui ... " << std::endl;

    impl_->flag_ui_exit_.store(false);
    impl_->imu_need_update_.store(false);
    impl_->wheel_need_update_.store(false);
    impl_->curr_image_need_update_.store(false);
    impl_->lidar_scan_need_update_.store(false);
    impl_->vio_state_need_update_.store(false);
    impl_->run_status_need_update_.store(false);

    bool inited = impl_->Init();
    if (inited) {
        impl_->render_thread_ = std::thread([this]() { impl_->RenderWindow(); });
    }
    return inited;
}

void UiWindow::UnInit() {
    std::cout << "[ Ui Window ] uninit Ui ... " << std::endl;
    if (impl_->render_thread_.joinable()) {
        impl_->flag_ui_exit_.store(true);
        impl_->render_thread_.join();
    }
    impl_->UnInit();
}

void UiWindow::UpdateRawImu(const ImuData& imu_data) {
    std::lock_guard<std::mutex> lock(impl_->mtx_raw_imu_);
    impl_->curr_imu_ = imu_data;
    impl_->imu_need_update_.store(true);
}

void UiWindow::UpdateRawWheel(const WheelData& wheel_data) {
    std::lock_guard<std::mutex> lock(impl_->mtx_raw_wheel_);
    impl_->curr_wheel_ = wheel_data;
    impl_->wheel_need_update_.store(true);
}

void UiWindow::UpdateCurrentImage(const cv::Mat& curr_image) {
    std::lock_guard<std::mutex> lock(impl_->mtx_curr_image_);
    impl_->curr_tracked_image_ = curr_image.clone();
    impl_->curr_image_need_update_.store(true);
}

void UiWindow::UpdateCurrentFeats(const std::vector<Eigen::Vector3f>& curr_feats) {
    //
}

void UiWindow::UpdateVioStatus(const VioStatus& vio_state) {
    std::lock_guard<std::mutex> lock(impl_->mtx_vio_state_);
    impl_->curr_vio_state_ = vio_state;
    impl_->vio_state_need_update_.store(true);
}

void UiWindow::UpdateRunningStatus(const RunningStatus& status) {
    std::lock_guard<std::mutex> lock(impl_->mtx_run_status_);
    impl_->curr_run_status_ = status;
    impl_->run_status_need_update_.store(true);
}

void UiWindow::UpdateVioStatus(const VinsEstimator& estimator) {
    // 放入 vio state
    std::lock_guard<std::mutex> lock(impl_->mtx_vio_state_);
    impl_->curr_vio_state_.timestamp_ = estimator.latest_time_;
    impl_->curr_vio_state_.position_ = estimator.latest_P_;
    impl_->curr_vio_state_.orientation_ = estimator.latest_Q_;
    impl_->curr_vio_state_.velocity_ = estimator.latest_V_;
    impl_->curr_vio_state_.bias_acc_ = estimator.latest_Ba_;
    impl_->curr_vio_state_.bias_gyr_ = estimator.latest_Bg_;
    impl_->curr_vio_state_.grav_ = estimator.Grav_;
    impl_->curr_vio_state_.latest_tracking_usage_ = estimator.curr_tracking_usage_.load();
    impl_->curr_vio_state_.latest_procmeas_usage_ = estimator.curr_procmeas_usage_.load();
    impl_->curr_vio_state_.visualz_imgLK_usage_ = estimator.ui_show_imgLK_usage_.load();
    impl_->curr_vio_state_.visualz_state_usage_ = estimator.ui_show_state_usage_.load();
    impl_->vio_state_need_update_.store(true);
}




// } // namespace VinsNS