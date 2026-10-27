#pragma once

#include "model/linearize.hpp"

struct LqrConfig{
    Scaling scaling{
        {1.0,1.0,0.5,2.0,0.5,2.0}, //归一化尺度
        20.0  //参考力
    };

    StateVector q_diagonal{
        10.0,1.0,100.0,1.0,100.0,1.0
        //状态代价矩阵 Q 的对角元素,p、dp、q1、dq1、q2、dq2
    };

    double r = 1.0;

    // 离散 Riccati 迭代参数
    int max_iterations = 20000;
    double convergence_tolerance = 1e-12;
};

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
