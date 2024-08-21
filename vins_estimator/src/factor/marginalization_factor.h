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

const int NUM_THREADS = 16;  //计算hessian时的并发加速多线程数量


/// @brief 边缘化操作中的一个因子，我们需要对因子计算残差和雅可比，这些事情用这个结构体来维护
struct ResidualBlockInfo {
    ResidualBlockInfo(ceres::CostFunction* _cost_function, ceres::LossFunction* _loss_function, 
                        std::vector<double*> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function)
        , parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    //在当前线性化点计算一次残差、和雅可比，结果保存在成员变量中，边缘化会用到
    void Evaluate();

    ceres::CostFunction* cost_function;     //ceres中的costFunc
    ceres::LossFunction* loss_function;     //ceres中的lossFunc
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
    Eigen::MatrixXd A; //是各subfactor的A的和，对应边缘化中的Ax=b
    Eigen::VectorXd b; //是各subfactor的b的和，对应边缘化中的Ax=b
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};


/// @brief 边缘化操作的核心类，收集被边缘化帧及其关联的帧、特征、因子，执行边缘化操作，获得新的边缘化因子。
/// NOTE: 这个类在原版VINS中的叫'MarginalizationInfo'，但我觉得Manager这个名字显然更贴切。
class MarginalizationManager {
   public:
    MarginalizationManager(){valid = true;}

    /// 别忘了在析构中显示地释放掉一些指针，以防内存泄漏
    ~MarginalizationManager();

    int localSize(int size) const;
    int globalSize(int size) const;

    /// 添加一个与被边缘化帧相关联的因子
    void addResidualBlockInfo(ResidualBlockInfo* residual_block_info);

    /// 对所有因子，在当前线性化点计算残差和雅可比
    void preMarginalize();

    /// 计算Ax=b和舒尔补等，也即边缘化
    void marginalize();

    /// xxx
    std::vector<double*> getParameterBlocks(std::unordered_map<long, double*> &addr_shift);

    std::vector<ResidualBlockInfo*> factors_;               //所有要被边缘化掉的约束

    /*边缘化相关的参数块都放在这里管理*/ 
    std::unordered_map<long, int> parameter_block_sizes_;   //参数块地址+参数块size（global size）
    std::unordered_map<long, int> parameter_block_idx_;     //参数块地址+参数块索引（local size）
    std::unordered_map<long, double*> parameter_block_data_;//深复制的参数块

    int m;  //被边缘化参数块的size
    int n;  //保留参数块的size

    std::vector<int> remaining_block_sizes_; //global size
    std::vector<int> remaining_block_idx_;  //local size
    std::vector<double*> remaining_block_data_;

    int sum_block_size_; //边缘化后，剩下的参数块的size之和

    /*边缘化的最终结果就是下边这两个玩意儿*/ 
    Eigen::MatrixXd linearized_jacobians_; //边缘化后的先验因子的雅可比（线性化点已经固定了，但作者认为影响不大）
    Eigen::VectorXd linearized_residuals_; //边缘化后的先验因子对当前状态量的残差

    const double eps_ = 1e-8; //防止数值不稳定的阈值
    bool valid = true;
};


/// @brief 边缘化因子，没毛病
class MarginalizationFactor : public ceres::CostFunction {
   public:
    // 将边缘化的结果转化为边缘化因子
    MarginalizationFactor(MarginalizationManager* _marginalization_info);

    // ceres接口，计算残差和雅可比，提供给ceres做优化
    virtual bool Evaluate(double const* const* _Parameters, double* _Residuals, double** _Jacobians) const;

    MarginalizationManager* marginalization_info_;
};
