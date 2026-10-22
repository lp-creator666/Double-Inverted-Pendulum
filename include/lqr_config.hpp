#pragma once

#include "scaling.hpp"

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