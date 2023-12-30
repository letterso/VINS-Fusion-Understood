/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef VINS_UI_WINDOW_H_
#define VINS_UI_WINDOW_H_

#include <memory>
#include <fstream>
#include <iomanip>

#include "ui_window_impl.h"
#include "../estimator/estimator.h"

// namespace VinsNS {

class UiWindow {
   public:
    UiWindow();
    ~UiWindow();

    bool Init();    // 初始化ui资源，开启显示线程
    void UnInit();  // 等待显示线程结束，并释放资源

    /// 更新各种信息
    void UpdateRawImu(const ImuData& imu_data);
    void UpdateRawWheel(const WheelData& wheel_data);
    void UpdateCurrentImage(const cv::Mat& curr_image);
    void UpdateCurrentFeats(const std::vector<Eigen::Vector3f>& curr_feats);
    void UpdateVioState(const VioState& vio_state);
    void UpdateRunningStatus(const RunningStatus& status);

    /// @brief AllIn1方式更新ui信息
    void UpdateVinsStatus(const VinsEstimator& estimator);

   private:
    double xxx;
    std::shared_ptr<UiWindowImpl> impl_;
};







// } // namespace VinsNS

#endif // VINS_UI_WINDOW_H_