#pragma once

#include <mujoco/mujoco.h>

// 借用 model 和 data，不负责释放；运行期间会推进 data 的仿真状态。
// 关闭窗口后返回 0，初始化失败或仿真异常时返回 1。
int run_viewer(const mjModel* model, mjData* data);
