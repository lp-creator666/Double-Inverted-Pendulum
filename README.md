# 二级倒立摆：MuJoCo + C++ 控制工程

在 MuJoCo 中完成二级小车倒立摆的局部平衡、约束平衡和大角度摆起。参考一阶倒立摆工程的学习方式：你按主计划逐项实现，我负责解释、拆解任务、检查代码与指导调参。

## 从哪里开始

**只按 [工程计划.md](工程计划.md) 推进。** 每次继续项目时看其中的“当前任务”，不需要写版本记录、阶段报告或开发日志。

当前已实现模型加载、关节地址查询、六维状态、独立 C++ 可视化窗口、CSV 日志、离散预测、数值线性化和可控性检查。长时域可控性、PBH 可稳定性、平衡点与小扰动预测检查已通过，离散 LQR 求解及 P、K 数值检查也已完成，下一步将反馈力接入 MuJoCo；之后按计划推进 LMPC 和 SQP。当前仍为开环仿真。

Windows 在项目根目录的 PowerShell 中运行：

```powershell
.\run.bat
```

也可直接双击项目根目录的 `run.bat`。脚本自动编译并启动窗口，使用本机 MinGW 中的 GLFW 和 `build/deps/mujoco`。初始第一关节偏转 0.02 rad；固定视角，按 Esc 或关闭窗口退出，目前不含鼠标相机操作。脚本只对当前子进程会话设置执行策略。

无窗口开环实验可直接传参数：

```powershell
.\run.bat --headless --duration 2 --force 1 --q1 0.02 --output logs/force1.csv
```

`--force` 会经过 XML 的输入范围限幅；每次实验覆盖指定 CSV。

## 控制路线

```text
模型与仿真 → 数值线性化 → LQR 平衡 → LMPC 约束平衡
                                      ↓
                         SQP 非线性约束控制 → 模式切换与整体调试
                                      ↓
                         根据实时性需求优化 SQP/RTI 配置
```

输入限幅和异常检查从仿真循环开始加入。LQR 只在验证过的局部区域内作为备用控制器。

## 文件入口

| 文件 / 目录 | 用途 |
|---|---|
| [工程计划.md](工程计划.md) | 唯一任务清单、当前任务和完成标准 |
| [docs/工程说明.md](docs/工程说明.md) | 状态约定、模块职责和程序数据流 |
| [MPC说明.md](MPC说明.md) | MPC、LMPC、SQP、NMPC 知识讲解 |
| [完整控制路线参考](二级倒立摆_iLQR_NMPC_LMPC_LQR完整路线.md) | 控制架构、代价、切换和终端集的扩展阅读 |
| [Windows与Ubuntu协同开发注意事项.md](Windows与Ubuntu协同开发注意事项.md) | 跨设备环境和 Git 使用说明 |
| `model/` | MuJoCo XML，物理参数的来源 |
| `include/`、`src/` | C++ 接口和实现，按任务逐步增加文件 |
| `config/` | 待接入的运行与控制参数 |
| `scripts/` | 后续的运行、画图和辅助工具 |
| `tests/` | 关键数值检查和控制效果验证 |
| `logs/`、`results/` | 最近一次实验的 CSV、曲线或视频 |

## CMake 构建

需 C++17 编译器、GLFW、OpenGL 和 MuJoCo。可控性诊断已使用 Eigen 3.4 的直接 SVD。Windows 脚本默认读取 `build/deps/eigen-3.4.0`，也可通过环境变量 `EIGEN3_INCLUDE_DIR` 指定包含 `Eigen/` 的目录。CMake 优先使用系统 Eigen3 包，否则查找工程内该目录或 `-DEIGEN3_INCLUDE_DIR=...`。

```bash
cmake -S . -B build/native -DMUJOCO_ROOT=/你的/MuJoCo/目录
cmake --build build/native
```

Windows MinGW 可使用 `-G "MinGW Makefiles"`，默认 MuJoCo 目录为项目内 `build/deps/mujoco`。本机也可使用 `build/deps/cmake/data/bin/cmake.exe`。Ubuntu 需另外安装 GLFW/OpenGL 开发依赖并指定本机 MuJoCo 目录；当前兼容未版本化 MuJoCo 库或 `libmujoco.so.3.4.0`。Linux 构建尚未实测。

可执行程序必须从项目根目录运行，以读取 `model/double_cartpole.xml`。Windows 首选上方脚本，它会同时配置 MuJoCo 与编译器运行库搜索路径。

数值诊断位于 `src/model_diagnostics.cpp`，启动时自动执行；无需额外运行命令，不打印可控性矩阵。CMake 构建后可用 `ctest --test-dir build/native --output-on-failure` 运行 SVD/PBH 回归测试与 MuJoCo 局部模型检查。后一项测试从项目根目录加载模型，并覆盖 `logs/model_check.csv`。Windows MinGW 已加入 Eigen 大目标文件所需的 `-Wa,-mbig-obj` 选项。

LQR 求解位于 `src/lqr.cpp`，参数位于 `include/lqr_config.hpp`。程序启动时打印收敛次数、Riccati 残差、P 的性质、闭环谱半径与缩放坐标下的 K。`lqr_solver` 测试包含解析解对照、缩放一致性和失败处理。
