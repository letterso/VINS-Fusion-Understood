/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <pthread.h>

#include <cstdlib>
#include <unordered_map>

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;  //多线程执行边缘化因子的线性化


/// @brief 边缘化操作中的一个因子，我们需要对因子计算残差和雅可比，这些事情用这个结构体来维护
struct ResidualBlockInfo {
    ResidualBlockInfo(ceres::CostFunction* _cost_function, ceres::LossFunction* _loss_function, 
                        std::vector<double*> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function)
        , parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    //在当前线性化点计算一次残差、和雅可比，结果保存在成员变量中，边缘化会用到
    void Evaluate();

    ceres::CostFunction* cost_function;     //常规costFunc
    ceres::LossFunction* loss_function;     //常规lossFunc
    std::vector<double*> parameter_blocks;  //因子关联的参数块（地址指向VinsEstimator类中的ceres参数块）
    std::vector<int> drop_set;              //需要被边缘化掉的参数块的索引

    double** raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;

    //参数块在李代数上的size
    int localSize(int size) {
        return size == 7 ? 6 : size;
    }
};


/// @brief 用于在多个线程中执行线性化任务 //重命名为'ThreadTask'可能更好
struct ThreadsStruct {
    std::vector<ResidualBlockInfo*> sub_factors;
    Eigen::MatrixXd A; //Ax=b，是各factor的A的和
    Eigen::VectorXd b; //Ax=b，是各factor的b的和
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};


/// @brief 边缘化操作的核心类，收集被边缘化帧及其关联的帧、特征、因子，执行边缘化操作，获得新的边缘化因子。
/// NOTE: 这个类在T.Qin原版中的名字叫'MarginalizationInfo'，但我觉得Manager这个名字显然更贴切。
class MarginalizationManager {
   public:
    MarginalizationManager(){valid = true;}

    // 别忘了在析构中显示地释放掉一些指针，以防内存泄漏
    ~MarginalizationManager();

    int localSize(int size) const;
    int globalSize(int size) const;

    // 添加一个与被边缘化帧相关联的因子
    void addResidualBlockInfo(ResidualBlockInfo* residual_block_info);

    // 对所有因子，在当前线性化点计算残差和雅可比
    void preMarginalize();

    // 计算Ax=b和舒尔补等
    void marginalize();

    //
    std::vector<double*> getParameterBlocks(std::unordered_map<long, double*> &addr_shift);

    std::vector<ResidualBlockInfo*> factors_;

    // 所有的参数块都要放到一起来管理
    std::unordered_map<long, int> parameter_block_sizes_;   //参数块地址+参数块size（global size）
    std::unordered_map<long, int> parameter_block_idx_;     //参数块地址+全局索引（local size）
    std::unordered_map<long, double*> parameter_block_data_;//似乎是拷贝的参数块？

    int m;  /*被边缘化参数块的size*/
    int n;  /*保留参数块的size*/
    

    std::vector<int> remaining_block_sizes_; //global size
    std::vector<int> remaining_block_idx_;  //local size
    std::vector<double*> remaining_block_data_;

    int sum_block_size_; /*边缘化后，剩下的参数块的size之和*/

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
    bool valid = true;

};


/// @brief 边缘化因子，没毛病
class MarginalizationFactor : public ceres::CostFunction {
   public:
    // 将边缘化的结果转化为边缘化因子
    MarginalizationFactor(MarginalizationManager* _marginalization_info);

    //
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    MarginalizationManager* marginalization_info;
};
