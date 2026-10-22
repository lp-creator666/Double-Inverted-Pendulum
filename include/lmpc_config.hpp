#pragma once

#include "scaling.hpp"

struct LmpcConfig {
    static constexpr int horizon = 20;  //统一预测时域为 20 步

    Scaling scaling{{1.0, 1.0, 0.5, 2.0, 0.5, 2.0}, 20.0};  //定义输入、轨道位置、速度约束

    // v=u/20, so these correspond to +/-20 N.
    double input_min = -1.0;
    double input_max = 1.0;

    // Bounds below use physical units.
    double cart_position_min = -3.9;
    double cart_position_max = 3.9;
    double cart_speed_min = -10.0;
    double cart_speed_max = 10.0;
    double joint_speed_min = -50.0;
    double joint_speed_max = 50.0;
};
