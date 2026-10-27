# Windows 与 Ubuntu 协同开发注意事项

本项目可以同时在 Windows 和 Ubuntu 上开发。Git 负责同步项目文件，但不会同步编译器、MuJoCo、Eigen 和其他运行环境，因此两台设备需要分别配置依赖。

## 1. 第一次使用 Ubuntu

```bash
git clone https://github.com/你的用户名/Double-Inverted-Pendulum.git
cd Double-Inverted-Pendulum
```

建议安装基础工具：

```bash
sudo apt update
sudo apt install git cmake g++ libeigen3-dev
```

MuJoCo、QP 求解器以及其他动态库需要在 Ubuntu 上单独安装和配置。

## 2. 不要提交构建目录

Windows 和 Ubuntu 的编译结果不同，不要把构建结果上传到 GitHub。

Ubuntu 上可以重新生成自己的构建目录：

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

项目中的 `.gitignore` 已经忽略 `build/`、日志和实验结果目录。

## 3. 使用相对路径

代码中不要写 Windows 专用的绝对路径：

```text
C:\Users\wlp\Desktop\Double Inverted Pendulum\model\double_cartpole.xml
```

应该使用项目相对路径：

```text
model/double_cartpole.xml
```

这样 Windows 和 Ubuntu 都能读取同一个模型文件。

## 4. 两台设备分别配置依赖

Git 只同步源代码、XML、配置和文档，不会同步以下内容：

- C++ 编译器；
- CMake；
- Eigen；
- MuJoCo；
- QP 求解器；
- Python 环境；
- 动态库和环境变量。

每台设备都需要单独安装这些工具，并确认版本兼容。

## 5. Git 同步顺序

开始工作前先获取其他设备的修改：

```bash
git pull
```

完成修改后提交并上传：

```bash
git add .
git commit -m "描述本次修改"
git push
```

如果两台设备同时修改了同一个文件，可能产生冲突。遇到冲突时，先打开 Git 标记出的文件，保留正确内容，然后执行：

```bash
git add 冲突文件
git commit -m "解决合并冲突"
git push
```

## 6. 换行符和编码

Windows 和 Ubuntu 的换行符可能不同。建议在 Ubuntu 上设置：

```bash
git config --global core.autocrlf input
```

源代码、XML 和 Markdown 文件统一使用 UTF-8 编码。不要使用只在 Windows 上可用的本地编码保存中文文件。

## 7. CMake 和路径写法

CMake 中优先使用正斜杠和相对路径：

```cmake
model/double_cartpole.xml
```

不要把 `C:\...`、盘符或 Windows 用户名写进 `CMakeLists.txt`、源代码和配置文件。

## 8. MuJoCo 模型检查

在两台设备上都确认：

- XML 可以被 MuJoCo 加载；
- `qpos` 和 `qvel` 的顺序一致；
- 执行器名称和控制范围一致；
- 模型中的相对资源路径正确；
- MuJoCo 版本没有明显差异。

如果两台设备使用不同 MuJoCo 版本，仿真细节和数值结果可能有轻微差异。

## 9. 推荐的日常工作流程

```text
切换设备
    ↓
git pull
    ↓
修改代码或文档
    ↓
本地编译和测试
    ↓
git add .
    ↓
git commit
    ↓
git push
```

不要直接把另一台设备的整个项目文件夹覆盖当前项目，优先使用 Git 同步，这样可以保留提交历史并减少文件丢失风险。

## 10. 本项目的核心原则

```text
GitHub 同步项目文件
每台设备单独安装依赖
构建目录不提交
代码使用相对路径
开始工作前先 pull
完成工作后再 commit 和 push
```
