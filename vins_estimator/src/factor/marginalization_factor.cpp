/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "marginalization_factor.h"

#include <tbb/tbb.h>

namespace {

void* ThreadsConstructA(void* threadsstruct) {
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    for (auto it : p->sub_factors) {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            if (size_i == 7)
                size_i = 6;
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

} // namespace

void ResidualBlockInfo::Evaluate() {
    // 重置残差块和jacobian块的size为正确的大小
    residuals.resize(cost_function->num_residuals());
    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians = new double*[block_sizes.size()];
    jacobians.resize(block_sizes.size());
    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }

    // 计算出【在当前线性化点位置】的残差和雅可比
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

    // 如果有鲁邦核函数，则根据核函数来修正残差和雅可比
    if (loss_function) {
        double residual_scaling_, alpha_sq_norm_;
        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

// ========================== split line ========================== //

MarginalizationManager::~MarginalizationManager() {
    // 释放new出来的数据，防内存泄漏
    for (auto it = parameter_block_data_.begin(); it != parameter_block_data_.end(); ++it) {
        delete it->second;
    }
    for (int i = 0; i < (int)factors_.size(); i++) {
        delete[] factors_[i]->raw_jacobians;
        delete factors_[i]->cost_function;
        delete factors_[i];
    }
}

void MarginalizationManager::addResidualBlockInfo(ResidualBlockInfo* residual_block_info) {
    factors_.emplace_back(residual_block_info);

    std::vector<double*>& factor_param_blocks = residual_block_info->parameter_blocks;
    std::vector<int> factor_param_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

    // 更保存（边缘化相关）参数块的size
    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
        double* addr = factor_param_blocks[i];
        int size = factor_param_block_sizes[i];
        parameter_block_sizes_[reinterpret_cast<long>(addr)] = size;
    }

    // 对于要被边缘化掉的参数块，地址记录到容器，index初始化为0
    /// NOTE: parameter_block_idx_保存的是localsize，且被边缘化的状态量在前，因此这里要先把被边缘化的状态量放进来，index给个0
    /// 在后续真正边缘化时，会把被边缘化的和保留的状态量，都放进来，且赋予正确的index。
    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++) {
        double* addr = factor_param_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx_[reinterpret_cast<long>(addr)] = 0;
    }
}

void MarginalizationManager::preMarginalize() {
    // 并行计算
    auto preparation_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            const auto &factor = factors_[k];
            //在当前线性化点计算残差和jacobian
            factor->Evaluate();
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, factors_.size()), preparation_function);

    // 涉及修改 parameter_block_data_
    for (const auto &factor : factors_) {
        std::vector<int> block_sizes = factor->cost_function->parameter_block_sizes();
        for (size_t i = 0; i < block_sizes.size(); i++) {
            long addr = reinterpret_cast<long>(factor->parameter_blocks[i]);
            int size = block_sizes[i];

            // 拷贝参数块数据
            if (parameter_block_data_.find(addr) == parameter_block_data_.end()) {
                double *data = new double[size];
                memcpy(data, factor->parameter_blocks[i], sizeof(double) * size);
                parameter_block_data_[addr] = data;
            }
        }
    }
}

int MarginalizationManager::localSize(int size) const {
    return size == 7 ? 6 : size;
}

int MarginalizationManager::globalSize(int size) const {
    return size == 6 ? 7 : size;
}

void MarginalizationManager::marginalize() {
    // 将所有参数块聚合到起来，被边缘化的参数块先排列
    int posi = 0;
    for (auto &it : parameter_block_idx_) {
        it.second = posi;
        posi += localSize(parameter_block_sizes_[it.first]);
    }
    m = posi; /*被边缘化的状态量的维度*/

    // 其余参数块，依次排列
    for (const auto &it : parameter_block_sizes_) {
        if (parameter_block_idx_.find(it.first) == parameter_block_idx_.end()) {
            parameter_block_idx_[it.first] = posi;
            posi += localSize(it.second);
        }
    }
    n = posi - m; /*保留的状态量的维度*/

    //printf("marginalization, posi: %d, m: %d, n: %d, size: %d", posi, m, n, (int)parameter_block_idx_.size());
    // LOG(INFO) << "marginalization dimensions: all size " << posi << ", margnlz size " << m 
    //     << ", remainning size " << n << ", block num " << (int)parameter_block_idx_.size();

    // 如果m为零，意味着没有要被边缘化参数块，结束
    if(m == 0) {
        valid = false;
        LOG(WARNING) << "(no param to marginalize) unstable tracking ... ";
        return;
    }

    Eigen::MatrixXd A(posi, posi);
    Eigen::VectorXd b(posi);
    A.setZero();
    b.setZero();

    if (factors_.size() < NUM_THREADS) {
        // ########## single-thread ##########
        for (auto it : factors_) {
            for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
                int idx_i = parameter_block_idx_[reinterpret_cast<long>(it->parameter_blocks[i])];
                int size_i = localSize(parameter_block_sizes_[reinterpret_cast<long>(it->parameter_blocks[i])]);
                Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
                for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
                    int idx_j = parameter_block_idx_[reinterpret_cast<long>(it->parameter_blocks[j])];
                    int size_j = localSize(parameter_block_sizes_[reinterpret_cast<long>(it->parameter_blocks[j])]);
                    Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                    if (i == j)
                        A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    else
                    {
                        A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                        A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                    }
                }
                b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
            }
        }
    } else {
      // ########## multi-thread ##########
      TicToc t_thread_summing;
      pthread_t tids[NUM_THREADS];
      ThreadsStruct threaded_tasks[NUM_THREADS];
      int i = 0;
      for (auto it : factors_) {
        threaded_tasks[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
      }
      for (int i = 0; i < NUM_THREADS; i++) {
        TicToc zero_matrix;
        //把A和b的size设置对了，后边各个线程中计算时，只管往正确的位置填结果即可
        threaded_tasks[i].A = Eigen::MatrixXd::Zero(posi, posi);
        threaded_tasks[i].b = Eigen::VectorXd::Zero(posi);
        threaded_tasks[i].parameter_block_size = parameter_block_sizes_;
        threaded_tasks[i].parameter_block_idx = parameter_block_idx_;
        int ret = pthread_create(&tids[i], NULL, ThreadsConstructA,
                                 (void *)&(threaded_tasks[i]));
        if (ret != 0) {
          LOG(ERROR) << ("pthread_create error");
          assert(0);
        }
      }

      // ########## sum up all A&b ##########
      for (int i = NUM_THREADS - 1; i >= 0; i--) {
        //等待线程执行完毕，获取结果
        pthread_join(tids[i], NULL);
        A += threaded_tasks[i].A;
        b += threaded_tasks[i].b;
      }
      // printf("thread summing up costs %f ms", t_thread_summing.toc());
      // printf("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());
    }

    // ########## compute Schur Complement ##########

    // 计算Amm块（A矩阵左上块）的逆，注意要解决一下数值不稳定问题
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm); //自伴矩阵(实对称就是)特征值分解
    /// 如下，作者似乎是强制特征值不能太小（>1e-4），后来作者注释掉了，我猜是通过下边求逆部分的数值处理弥补了这个要求。
    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());

    /// NOTE: 如下，这里的求逆并不是常规的矩阵inverse，我猜是因为这种操作数值不稳定，比如对接近于0的值敏感；
    /// 作者采用的等价方法是，对特征值求倒数，再和特征向量乘在一起，恢复为矩阵，作为矩阵的逆；
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * 
        Eigen::VectorXd((saes.eigenvalues().array() > eps_).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * 
        saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    // 计算舒尔补，获得边缘化后的A阵和b
    Eigen::MatrixXd Amr = A.block(0, m, m, n); //右上块
    Eigen::MatrixXd Arm = A.block(m, 0, n, m); //左下块
    Eigen::MatrixXd Arr = A.block(m, m, n, n); //右下块
    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr; //这就是舒尔补了，A作为hessian，包含了（线性近似后的）先验信息
    b = brr - Arm * Amm_inv * bmm; //等号右侧项，包含了“先验信息对当前状态量的残差”信息

    // ########## compute Marginalization factor (with fixed linearization position) ##########
    /** NOTE: 解释一下下边做的事情：从边缘化后的A和b中恢复出雅可比和残差，
     * 原理是H=JT*J, b=J*residual, 因此要对H做特征值分解，以求得J，再求得residual */ 

    // 同上，为防数值不稳定，要求特征值（及其倒数）不能接近于0，若接近于0则干脆置0
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S /*特征值*/ = Eigen::VectorXd((saes2.eigenvalues().array() > eps_).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv /*特征值的倒数*/ = Eigen::VectorXd((saes2.eigenvalues().array() > eps_).select(saes2.eigenvalues().array().inverse(), 0));

    // 求开方：也即对特征值元素逐个计算开方
    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    // 这里就得到边缘化想要的东西了
    linearized_jacobians_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose(); //恢复出雅可比，没毛病
    linearized_residuals_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b; //从b中去除雅可比（参见G-N），恢复出残差，没毛病

    //std::cout << A << std::endl << std::endl;
    //std::cout << linearized_jacobians_ << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians_.transpose() * linearized_jacobians_ - A).sum(),
    //      (linearized_jacobians_.transpose() * linearized_residuals_ - b).sum());

    // ########## all done ##########
}

std::vector<double*> MarginalizationManager::getParameterBlocks(std::unordered_map<long, double*> &addr_shift) {
    std::vector<double*> remaining_block_addr;
    remaining_block_sizes_.clear();
    remaining_block_idx_.clear();
    remaining_block_data_.clear();

    for (const auto &it : parameter_block_idx_) {
        if (it.second >= m) {
            remaining_block_sizes_.push_back(parameter_block_sizes_[it.first]);
            remaining_block_idx_.push_back(parameter_block_idx_[it.first]);
            remaining_block_data_.push_back(parameter_block_data_[it.first]);
            remaining_block_addr.push_back(addr_shift[it.first]);
        }
    }
    sum_block_size_ = std::accumulate(std::begin(remaining_block_sizes_), std::end(remaining_block_sizes_), 0);

    return remaining_block_addr;
}

// ========================== split line ========================== //

MarginalizationFactor::MarginalizationFactor(MarginalizationManager* _marginalization_info)
    : marginalization_info_(_marginalization_info) {
    //
    int cnt = 0;
    for (auto it : marginalization_info_->remaining_block_sizes_) {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info_->n);
};

bool MarginalizationFactor::Evaluate(double const* const* _Parameters, double* _Residuals, double** _Jacobians) const {
    //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    //for (int i = 0; i < static_cast<int>(remaining_block_sizes_.size()); i++)
    //{
    //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(_Parameters[i]));
    //    //printf("signed %x\n", reinterpret_cast<long>(_Parameters[i]));
    //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    //printf("residual %x\n", reinterpret_cast<long>(residuals));
    //}

    // step01: 计算当前状态量，对边缘化时的线性化点的差异量
    const int n = marginalization_info_->n;
    const int m = marginalization_info_->m;
    Eigen::VectorXd dx(n);
    for (int i = 0; i < static_cast<int>(marginalization_info_->remaining_block_sizes_.size()); i++) {
        int size = marginalization_info_->remaining_block_sizes_[i];
        int idx = marginalization_info_->remaining_block_idx_[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(_Parameters[i], size); //当前状态值
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info_->remaining_block_data_[i], size); //边缘化线性化点
        if (size != 7) {
            dx.segment(idx, size) = x - x0;
        }
        else {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0)) {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }

    // step02: 更新边缘化因子的残差
    Eigen::Map<Eigen::VectorXd>(_Residuals, n) = marginalization_info_->linearized_residuals_ + marginalization_info_->linearized_jacobians_ * dx;

    // step03: 更新边缘化因子的雅可比（直接从相应位置拷贝雅可比块）
    if (_Jacobians) {
        for (int i = 0; i < static_cast<int>(marginalization_info_->remaining_block_sizes_.size()); i++) {
            if (_Jacobians[i]) {
                int size = marginalization_info_->remaining_block_sizes_[i], local_size = marginalization_info_->localSize(size);
                int idx = marginalization_info_->remaining_block_idx_[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(_Jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info_->linearized_jacobians_.middleCols(idx, local_size);
            }
        }
    }
    return true;
}
