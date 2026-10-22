#pragma once

#include "lqr_config.hpp"

// P and K use scaled coordinates: v = -K z.
struct LqrResult {
    MatrixA P{}; //离散 Riccati 方程的解，大小是 6×6
    VectorB K{}; //反馈增益，顺序对应六个状态

    int iterations = 0; //实际迭代次数
    double final_change = 0.0; // P 的相对迭代变化量
    double riccati_residual = 0.0; // 归一化 Riccati 残差
    double symmetry_error = 0.0;        // P 的归一化对称误差
    double min_p_eigenvalue = 0.0;      // P 的最小特征值
    double closed_loop_radius = 0.0;   // 闭环谱半径 
};

// Input A/B must be in physical coordinates; scaling is applied internally.
LqrResult solve_discrete_lqr(
    const MatrixA& A,
    const VectorB& B,
    const LqrConfig& config
);

// 输入实际状态，返回实际推力，单位 N；不在这里限幅。
double compute_lqr_force(
    const State& state,
    const LqrResult& lqr,
    const LqrConfig& config
);