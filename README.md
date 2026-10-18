# Double Inverted Pendulum

二级小车倒立摆控制项目。

## 控制架构

```text
iLQR / NMPC：大角度摆起
        ↓
LMPC：接近平衡时处理约束
        ↓
LQR：局部快速稳定和故障兜底
        ↓
安全滤波：输入限幅、轨道保护、异常检查
```

## 当前阶段

当前只建立工程框架，暂不实现控制算法代码。

## 目录说明

- `model/`：MuJoCo XML 模型；
- `include/`：后续放置 C++ 头文件；
- `src/`：后续放置 C++ 源文件；
- `tests/`：单元测试和控制器测试；
- `config/`：模型参数和控制器参数；
- `docs/`：设计文档和实验记录；
- `scripts/`：编译、运行和数据处理脚本；
- `logs/`：仿真日志和结果；
- `results/`：曲线、视频和实验汇总。

## 推荐实现顺序

```text
MuJoCo 接口 → 动力学 → 线性化 → LQR → LMPC → iLQR → NMPC → 模式切换 → 安全滤波
```
