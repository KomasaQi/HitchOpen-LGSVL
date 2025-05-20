// MFASMC_Controller.cpp

#include "MFASMC_Controller.hpp"
#include <random>
#include <algorithm>
#include <iostream>

using namespace Eigen;

MFASMC_Controller::MFASMC_Controller(const Params& params) 
    : params_(params) 
{
    // 初始化随机数生成器
    std::default_random_engine generator(0);
    std::normal_distribution<double> distribution(0.0, 0.1);

    // 初始化存储向量
    u = VectorXd::Zero(params_.Nu);
    y = VectorXd::Zero(params_.Ny);
    dH = VectorXd(params_.Ly * params_.Ny + params_.Lu * params_.Nu);
    
    // 使用正态分布初始化dH
    for (int i = 0; i < dH.size(); ++i) {
        dH(i) = distribution(generator);
    }

    // 初始化Phi矩阵
    Phi = MatrixXd::Zero(params_.Ny, params_.Ly * params_.Ny + params_.Lu * params_.Nu);
    Phi.block(0, params_.Ly * params_.Ny, params_.Ny, params_.Nu) = params_.phi_Lyp1;

    // 初始化其他状态变量
    ydes_k_1 = VectorXd::Zero(params_.Ny);
    ek_1 = VectorXd::Zero(params_.Ny);
    hk_1 = VectorXd::Zero(params_.Ny);
    fk_1 = VectorXd::Zero(params_.Ny);
    A = (MatrixXd::Identity(params_.Ny, params_.Ny) / params_.Ts + params_.Lambda);
    
    checkValidity();
}

void MFASMC_Controller::updatePhiEstimation(const VectorXd& yk) {
    VectorXd yk_1 = y;  // 保存旧值
    y = yk;             // 更新当前观测值

    // 计算伪梯度更新
    double ldHl2 = dH.squaredNorm();
    MatrixXd deltaPhi = (params_.eta * (yk - yk_1 - Phi * dH) * dH.transpose()) / (params_.mu + ldHl2);
    Phi += deltaPhi;

    // 伪梯度重置逻辑
    for (int n = 0; n < params_.Lu; ++n) {
        int colStart = params_.Ly * params_.Ny + n * params_.Nu;
        MatrixXd block = Phi.block(0, colStart, params_.Ny, params_.Nu);
        
        for (int i = 0; i < params_.Ny; ++i) {
            for (int j = 0; j < params_.Nu; ++j) {
                if (i == j) {
                    if (std::abs(block(i,j)) < params_.epsilon || 
                        std::abs(block(i,j)) > params_.uplim) {
                        block(i,j) = params_.phi_Lyp1(i,j);
                    }
                } else {
                    if (std::abs(block(i,j)) > params_.uplim) {
                        block(i,j) = params_.phi_Lyp1(i,j);
                    }
                }
            }
        }
        Phi.block(0, colStart, params_.Ny, params_.Nu) = block;
    }

    // 更新dH中的dyk部分
    VectorXd deltaY = yk - yk_1;
    dH.segment(0, params_.Ny) = deltaY;
    dH.segment(params_.Ny, (params_.Ly-1)*params_.Ny) = 
        dH.segment(params_.Ny, (params_.Ly-1)*params_.Ny);
}

VectorXd MFASMC_Controller::calculateControlValueSMC(const VectorXd& yk, const VectorXd& ydes) {
    VectorXd fk = VectorXd::Zero(params_.Ny);
    
    // 计算fk的前Ly项
    for (int i = 0; i < params_.Ly; ++i) {
        int colStart = i * params_.Ny;
        MatrixXd phiBlock = Phi.block(0, colStart, params_.Ny, params_.Ny);
        VectorXd dyBlock = dH.segment(colStart, params_.Ny);
        fk += params_.rho[i] * (phiBlock * dyBlock);
    }

    // 计算fk的Lu项
    for (int i = 1; i < params_.Lu; ++i) {
        int colStart = params_.Ly * params_.Ny + (i-1) * params_.Nu;
        MatrixXd phiBlock = Phi.block(0, colStart, params_.Ny, params_.Nu);
        VectorXd duBlock = dH.segment(colStart, params_.Nu);
        fk += params_.rho[params_.Ly + i] * (phiBlock * duBlock);
    }

    // 计算滑模控制项
    VectorXd skp1 = smcApproach(yk);
    MatrixXd phi_Lyp1 = Phi.block(0, params_.Ly * params_.Ny, params_.Ny, params_.Nu);
    
    // 构建求解方程
    MatrixXd leftMatrix = params_.lambda * MatrixXd::Identity(params_.Nu, params_.Nu) + 
                          phi_Lyp1.transpose() * phi_Lyp1;
    VectorXd rightVector = phi_Lyp1.transpose() * 
        (params_.rho[params_.Ly] * (ydes - yk) - fk - 
         A.inverse() * (fk_1 / params_.Ts - hk_1 / params_.Ts + skp1));

    // 求解线性方程组
    VectorXd duk = leftMatrix.llt().solve(rightVector);

    // 控制量限幅
    if (!params_.du_cons.isZero()) {
        for (int i = 0; i < params_.Nu; ++i) {
            duk(i) = std::clamp(duk(i), params_.du_cons(i,0), params_.du_cons(i,1));
        }
    }

    VectorXd uk = u + duk;
    
    if (!params_.u_cons.isZero()) {
        for (int i = 0; i < params_.Nu; ++i) {
            uk(i) = std::clamp(uk(i), params_.u_cons(i,0), params_.u_cons(i,1));
        }
    }

    // 更新存储变量
    u = uk;
    dH.segment(params_.Ly * params_.Ny, params_.Lu * params_.Nu) = 
        (uk - u).eval();  // 注意：需要更新整个du历史
    
    ydes_k_1 = ydes;
    fk_1 = fk;
    hk_1 = phi_Lyp1 * duk;

    return uk;
}

VectorXd MFASMC_Controller::smcApproach(const VectorXd& yk) {
    VectorXd ek = ydes_k_1 - yk;
    VectorXd sk = A * ek - ek_1 / params_.Ts;
    VectorXd skp1 = (MatrixXd::Identity(params_.Ny, params_.Ny) - params_.Ts * params_.K) * sk 
                   - params_.Ts * params_.E * sk.unaryExpr([](double x){return x>0?1:-1;});
    ek_1 = ek;
    return skp1;
}

void MFASMC_Controller::validateObservation(const VectorXd& yk, const VectorXd& ydes) const {
    if (yk.size() != params_.Ny) {
        throw std::invalid_argument("Invalid observation vector size");
    }
    if (ydes.size() != params_.Ny) {
        throw std::invalid_argument("Invalid desired vector size");
    }
}

void MFASMC_Controller::checkValidity() {
    if (params_.rho.size() != (params_.Ly + params_.Lu)) {
        if (params_.rho.size() == 1) {
            params_.rho = std::vector<double>(params_.Ly + params_.Lu, params_.rho[0]);
        } else if (params_.rho.size() == 2) {
            params_.rho = std::vector<double>(params_.Ly, params_.rho[0]);
            params_.rho.insert(params_.rho.end(), params_.Lu, params_.rho[1]);
        } else {
            throw std::invalid_argument("Invalid rho vector size");
        }
    }

    if (params_.mode == "mfasmc") {
        if (params_.K.rows() != params_.Ny || params_.K.cols() != params_.Ny) {
            params_.K = 0.5 * MatrixXd::Identity(params_.Ny, params_.Ny);
        }
        if (params_.E.rows() != params_.Ny || params_.E.cols() != params_.Ny) {
            params_.E = 0.1 * MatrixXd::Identity(params_.Ny, params_.Ny);
        }
    }
}
