# 二级倒立摆：MuJoCo + C++

工程包含 LQR 局部平衡、LMPC 约束平衡、SQP 离线起摆规划、时变 LQR 轨迹反馈、以实测状态重求的在线 SQP，以及捕获切换、故障处理和自动验收。详细范围与实测限制见唯一执行计划 [工程计划.md](工程计划.md)。知识文档继续保留。

## 运行

在项目根目录 PowerShell 中，推荐先运行带反馈的起摆：

```powershell
.\run.bat --track-plan config/plans/sqp_swingup.csv --duration 10
```

两杆从完全下垂开始，前 4 秒跟踪 SQP 生成的轨迹，随后检查实际捕获状态并切换平衡 LQR。运行时每 0.002 s 根据实际偏差修正输入；不会在线重做轨迹优化。关闭窗口或按 Esc 退出。源码和头文件未变化时，脚本复用已编译程序。

其他入口：

```powershell
.\run.bat --lqr --q1-deg 1 --q2-deg 0
.\run.bat --lmpc --q1-deg 1 --q2-deg 0
.\run.bat --online-plan config/plans/sqp_swingup.csv --duration 10
.\run.bat --track-plan config/scenarios/down_q2_90.csv --duration 10
```

`--online-plan` 每 0.08 s 从当前实测状态重做剩余时域的 SQP，再配合段内状态反馈，最后切到 LQR；它使用已有轨迹作初始种子。当前长时域求解多次超过 80 ms，允许仿真慢于真实时间，不能当作硬实时控制器。默认软预算 0.5 s，可用 `--solve-budget` 调整；推荐日常演示使用 `--track-plan`。

`--plan FILE` 保留纯前馈重放对照，`--sqp` 是不带起摆轨迹种子的短时域局部优化实验，`--swingup` 是尚未成功通过大角度验收的能量启发式对照。它们不能互相替代或混用。

角度与状态：

- `--q1-deg/--q2-deg` 使用度；直接 exe 的 `--q1/--q2` 使用 rad。
- q1 相对竖直向上，q2 相对第一杆；第二杆绝对角为 q1+q2。
- 计划入口从 CSV 获取完整初态，不能同时给 `--q1` 等初态参数。
- `--headless --duration 10` 使用同一控制和保护逻辑进行无窗口仿真。
- `--output logs/example.csv` 指定日志路径，重复运行覆盖文件。

## 偏差和扰动

给真实初态增加偏差，参考轨迹保持不变：

```powershell
.\run.bat --track-plan config/plans/sqp_swingup.csv --duration 10 --plan-q1-offset-deg 2
```

支持 `--plan-q2-offset-deg`、`--plan-p-offset`、`--plan-dp-offset`、`--plan-dq1-offset`、`--plan-dq2-offset`，位置/速度参数使用物理单位。

第 1 秒向小车施加 1 N、持续 0.1 s 的外力：

```powershell
.\run.bat --track-plan config/plans/sqp_swingup.csv --duration 10 --push-force 1 --push-start 1 --push-duration 0.1
```

外力通过 `qfrc_applied` 施加，与执行器力分开，因此不会被 ±20 N 的执行器限幅吞掉。

## 自动检查

也可以直接运行统一测试入口：`python tests/run_all.py`，加 `--quick` 跳过综合验收。


```powershell
.\check.bat
.\check.bat -Quick
```

完整检查会用 Release 配置构建，再运行数值测试、闭环对照和综合验收；`-Quick` 跳过耗时的综合验收。需要 Python 3 运行批量脚本。综合结果覆盖到 `logs/acceptance/report.json`，检查位置、速度、力、末段稳定误差、实际外力冲量、在线优化采用次数与超时/故障处理。测试通过代表明示场景通过，不代表全局稳定。

轨迹库在 `config/scenarios.json`，包含完全下垂、反方向对称姿态及 q1=180°、相对 q2=20°/90°。大角度范围依赖具体双杆组合，不能从单一角度推断。模型修改后必须重新生成并验收轨迹。

## 日志与曲线

默认日志 `logs/run.csv` 包含六维状态、第二杆绝对角、请求力、实际执行器力、外力和数值阶段代码。状态时间为物理步推进后的时间，力和阶段表示该物理步使用的值。

| stage | 含义 |
|---|---|
| 0 | 开环/初始日志 |
| 1 | 纯前馈轨迹 |
| 2 | SQP |
| 3 | 平衡 LQR |
| 4 | 在线失败后的可行暖启动或轨迹反馈兜底 |
| 5 | 时变 LQR 轨迹反馈 |
| 6 | LMPC |
| 7 | 能量启发式对照 |

在线运行另写 `<输出CSV路径>.solver.csv`，记录状态、可行性、接受步数、QP 迭代、优化前后代价、约束违反量、耗时、软预算和控制周期超时。`selected` 区分 `sqp`、`feasible_warm_start` 与 `trajectory_feedback`，不能把兜底次数算成 SQP 更新成功。

```powershell
python scripts/plot_response.py logs/acceptance/online_nominal.csv --title "Online SQP swing-up and LQR capture" --output results/online_response.png
python scripts/plot_solver.py logs/acceptance/online_nominal.csv.solver.csv
```

绘图需要 Matplotlib，脚本也可读取工程内 `build/deps/python_plot` 的安装。连续角度到 360° 表示再次直立；稳定验收使用周期误差。曲线覆盖到 `results/`，不逐次生成版本报告。

## 重新规划

规划工具是 CMake 目标 `plan_swingup`，源码在 `tools/plan_swingup.cpp`，不是需要随控制循环执行的测试文件。

```powershell
.\build\svd-check\plan_swingup.exe 3.141592653589793 config/seeds/sqp_swingup_seed.csv 20 0
python scripts/plan_scenarios.py --iterations 20
```

单次工具参数为 `q1_rad seed_csv iterations q2_rad dq1 dq2`，后两个速度默认零。默认 50 段，每段 0.08 s，时域 4 s。种子路径为 `-` 时探索零/正弦种子；完全下垂的零梯度不能作为起摆成功，非零种子和局部极值都会影响结果。

候选输入为 `logs/swingup_candidate.csv`。只有末端捕获及连续重放后 6 s 的稳定检查均通过，才写 `logs/swingup_verified.csv` 并返回成功；每次规划开始清除旧 verified 文件。批量工具把本次通过的结果存到 `logs/planning/`，不自动覆盖已交付轨迹库。实际参数在命令行、C++ 配置结构、XML 与 CSV；保留的 `docs/examples/default.yaml` 不被程序读取。

## 构建依赖

需要 C++17、MuJoCo、Eigen 3.4、GLFW、OpenGL。本机脚本使用 MinGW，默认依赖位于 `build/deps/mujoco`、`build/deps/eigen-3.4.0`；Eigen 可通过 `EIGEN3_INCLUDE_DIR` 指定。编译器 `bin` 与 MuJoCo DLL 目录必须在运行时 PATH 内，`run.bat` / `check.bat` 会处理当前会话。

通用 CMake：

```bash
cmake -S . -B build/native -DCMAKE_BUILD_TYPE=Release -DMUJOCO_ROOT=/你的/MuJoCo/目录
cmake --build build/native -j 2
ctest --test-dir build/native --output-on-failure
```

Windows MinGW 可用 `-G "MinGW Makefiles"`；本机检查脚本复用 `build/svd-check`。无系统 CMake 时可使用 `build/deps/cmake/data/bin/cmake.exe`。Ubuntu 需本机 GLFW/OpenGL 与 MuJoCo；跨系统构建入口保留，Linux 尚未在本轮实测。程序从工程根目录运行。

## 文件怎么找

```text
src/、include/
  app/             启动、仿真窗口、日志
  model/           状态、预测、线性化、诊断
  control/         LQR、LMPC、轨迹反馈、在线控制与切换
  optimization/    非线性模型、代价、SQP、通用 QP
research/       能量法与多重射击辅助代码
tests/             模型、控制、优化、实验测试及闭环验收
tools/             离线规划程序
scripts/           运行包装、检查与绘图
model/             MuJoCo XML
config/            运行轨迹；scenarios/ 为其他姿态，seeds/ 为规划种子
docs/              工程说明与全部知识材料
build/             依赖和编译结果
logs/、results/    运行数据与图片
```

`src/main.cpp` 只负责启动，参数解析与控制器组合在 `src/app/application.cpp`。LQR/LMPC 的参数和接口各自集中在同一个头文件。查阅算法时，从同名的头文件和实现文件开始；详细职责与阅读顺序见工程说明。

## 学习入口

- [工程说明](docs/工程说明.md)：模块、状态、轨迹反馈与控制数据流。
- [MPC 说明](docs/MPC说明.md)：原理材料。
- [控制架构参考](docs/二级倒立摆_iLQR_NMPC_LMPC_LQR完整路线.md)：扩展知识，工程实际选择 SQP。
- [Windows 与 Ubuntu 协作](docs/Windows与Ubuntu协同开发注意事项.md)：环境与 Git。

默认推荐方式是 SQP 离线规划 + TVLQR + LQR；在线 SQP 已接通并测量计算成本。全局起摆、严格鲁棒约束和硬实时不是当前已证明的能力，具体限制见工程计划。