#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <functional>
#include "types.hpp"

struct RunOptions {
    double requested_force = 0.0;
    double duration = 0.0; // <= 0 means run until the window is closed.
    std::string log_path = "logs/run.csv";
    // Empty: constant force. Otherwise evaluate before every physical step.
    std::function<double(const State&)> feedback;
};

// 借用 model 和 data，不负责释放；运行期间会推进 data 的仿真状态。
// duration <= 0 时运行到窗口关闭，否则到达指定仿真时间自动退出。
int run_viewer(
    const mjModel* model,
    mjData* data,
    const int* qpos_addresses,
    const int* qvel_addresses,
    const RunOptions& options
);

// 无窗口运行同一套仿真、日志和安全检查。
int run_headless(
    const mjModel* model,
    mjData* data,
    const int* qpos_addresses,
    const int* qvel_addresses,
    const RunOptions& options
);
