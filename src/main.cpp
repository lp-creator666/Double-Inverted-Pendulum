#include <iostream>
#include <cmath>
#include <string>
#include <stdexcept>

#include <mujoco/mujoco.h>

#include "viewer.hpp"
#include "types.hpp"
#include "mujoco_interface.hpp"
#include "dynamics.hpp"
#include "linearize.hpp"
#include "model_diagnostics.hpp"
#include "lqr.hpp"

namespace {
double parse_number(const std::string& text) {
    std::size_t end = 0;
    const double value = std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument("无效数字: " + text);
    }
    return value;
}
}

int main(int argc, char** argv) {
    RunOptions options;
    bool headless = false;
    bool use_lqr = false;
    double initial_q1 = 0.02;
    double initial_q2 = 0.0;
    double initial_dq1 = 0.0;
    double initial_dq2 = 0.0;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "用法: double_pendulum [选项]\n"
                             "  --headless          无窗口运行\n"
                             "  --lqr               LQR 局部平衡控制\n"
                             "  --force N           恒定请求力（N）\n"
                             "  --duration S        仿真时长；无窗口模式必填\n"
                             "  --q1 RAD            第一关节初始角\n"
                             "  --q2 RAD            第二关节初始相对角\n"
                             "  --dq1 RAD_S         第一关节初始角速度\n"
                             "  --dq2 RAD_S         第二关节初始相对角速度\n"
                             "  --output FILE       CSV 输出路径\n";
                return 0;
            }
            if (arg == "--headless") {
                headless = true;
                continue;
            }
            if (arg == "--lqr") {
                use_lqr = true;
                continue;
            }
            if (i + 1 >= argc) throw std::invalid_argument("缺少参数值: " + arg);
            const std::string value = argv[++i];
            if (arg == "--force") {
                options.requested_force = parse_number(value);
            } else if (arg == "--duration") {
                options.duration = parse_number(value);
            } else if (arg == "--q1") {
                initial_q1 = parse_number(value);
            } else if (arg == "--q2") {
                initial_q2 = parse_number(value);
            } else if (arg == "--dq1") {
                initial_dq1 = parse_number(value);
            } else if (arg == "--dq2") {
                initial_dq2 = parse_number(value);
            } else if (arg == "--output") {
                options.log_path = value;
            } else {
                throw std::invalid_argument("未知参数: " + arg);
            }
        }
        if (use_lqr && options.requested_force != 0.0) {
            throw std::invalid_argument("--lqr cannot be combined with a nonzero --force");
        }
        if (headless && options.duration <= 0) {
            throw std::invalid_argument("--headless 必须同时指定正的 --duration");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n使用 --help 查看用法。\n";
        return 1;
    }

    const char* xml_path = "model/double_cartpole.xml";

    char error[1024] = "";

    mjModel* model = mj_loadXML(
        xml_path,
        nullptr,
        error,
        sizeof(error)
    );

    if (model == nullptr) {
        std::cerr <<"MuJoCo模型加载失败：\n"
                  << error << std::endl;
        return 1;
    }
    
    mjData* data = mj_makeData(model);

    if (data == nullptr) {
        std::cerr << "MuJoCo 数据创建失败" << std::endl;
        mj_deleteModel(model);
        return 1;
    }

    mjData* prediction_data = mj_makeData(model);
    if (prediction_data == nullptr) {
        std::cerr << "预测数据创建失败\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    std::cout << "MuJoCo 模型加载成功\n";
    std::cout << "时间步长: " << model->opt.timestep << " s\n";
    std::cout << "qpos 数量 nq: " << model->nq << "\n";
    std::cout << "qvel 数量 nv: " << model->nv << "\n";
    std::cout << "执行器数量 nu: " << model->nu << "\n";
    std::cout << "仿真时间: " << data->time << " s\n";

    const char* joint_names[3] = {
        "cart_slide",
        "pole_1_hinge",
        "pole_2_hinge"
    };

    int joint_ids[3];
    int qpos_addresses[3];
    int qvel_addresses[3];

    for (int i = 0; i < 3; ++i) {
        joint_ids[i] = mj_name2id(model, mjOBJ_JOINT, joint_names[i]);

        if (joint_ids[i] < 0) {
            std::cerr << "找不到关节: " << joint_names[i] << std::endl;
            mj_deleteData(prediction_data);
            mj_deleteData(data);
            mj_deleteModel(model);
            return 1;
        }

        qpos_addresses[i] = model->jnt_qposadr[joint_ids[i]];
        qvel_addresses[i] = model->jnt_dofadr[joint_ids[i]];

        std::cout << "关节: " << joint_names[i]
                  << ", ID: " << joint_ids[i]
                  << ", qpos 地址: " << qpos_addresses[i]
                  << ", qvel 地址: " << qvel_addresses[i] << "\n";
    }

    const State state = read_state(
        data,
        qpos_addresses,
        qvel_addresses
    );

    std::cout << "\n六维状态 [p, dp, q1, dq1, q2, dq2]:\n";

    std::cout << "["
              << state.p << ", "
              << state.dp << ", "
              << state.q1 << ", "
              << state.dq1 << ", "
              << state.q2 << ", "
              << state.dq2 << "]\n";

    std::cout << "第二杆绝对角: "
          << state.q1 + state.q2 << " rad\n";
          
    // 设置本次实验的初始关节角。
    data->qpos[qpos_addresses[1]] = initial_q1;
    data->qpos[qpos_addresses[2]] = initial_q2;
    data->qvel[qvel_addresses[1]] = initial_dq1;
    data->qvel[qvel_addresses[2]] = initial_dq2;
    mj_forward(model, data);

    const State current_state = read_state(
        data,
        qpos_addresses,
        qvel_addresses
    );
    const State predicted_state = predict_state(
        model,
        prediction_data,
        current_state,
        qpos_addresses,
        qvel_addresses,
        options.requested_force,
        10
    );

    // 在直立平衡点附近计算单步离散线性模型。
    const State equilibrium_state{};
    const int linearization_steps = 1;
    const double state_epsilon = 1e-6;
    const double input_epsilon = 1e-6;
    const Linearization linearization = linearize_discrete(
        model,
        prediction_data,
        equilibrium_state,
        0.0,
        qpos_addresses,
        qvel_addresses,
        linearization_steps,
        state_epsilon,
        input_epsilon
    );

    std::cout << "预测 10 个物理步后的状态（约 0.02 s）：\n"
              << "[" << predicted_state.p << ", "
              << predicted_state.dp << ", "
              << predicted_state.q1 << ", "
              << predicted_state.dq1 << ", "
              << predicted_state.q2 << ", "
              << predicted_state.dq2 << "]\n";

    std::cout << "\n离散线性化矩阵 A:\n";
    for (int row = 0; row < kStateDimension; ++row) {
        std::cout << "[ ";
        for (int column = 0; column < kStateDimension; ++column) {
            std::cout << linearization.A[row * kStateDimension + column];
            if (column + 1 < kStateDimension) {
                std::cout << ", ";
            }
        }
        std::cout << " ]\n";
    }

    std::cout << "离散线性化矩阵 B:\n[ ";
    for (int row = 0; row < kStateDimension; ++row) {
        std::cout << linearization.B[row];
        if (row + 1 < kStateDimension) {
            std::cout << ", ";
        }
    }
    std::cout << " ]\n";

    try {
        run_model_diagnostics(model, prediction_data, qpos_addresses,
                              qvel_addresses, linearization);
        const LqrConfig config;
        const LqrResult lqr = solve_discrete_lqr(
            linearization.A, linearization.B, config);
        if (use_lqr) {
            // Capture owned copies: the simulation runs after this scope ends.
            options.feedback = [lqr, config](const State& state) {
                return compute_lqr_force(state, lqr, config);
            };
        }
        std::cout << "Control mode: " << (use_lqr ? "LQR" : "constant force") << "\n";

        std::cout << "\nLQR numerical verification:\n"
                  << "iterations: " << lqr.iterations << "\n"
                  << "relative change: " << lqr.final_change << "\n"
                  << "Riccati residual: " << lqr.riccati_residual << "\n"
                  << "P symmetry error: " << lqr.symmetry_error << "\n"
                  << "P minimum eigenvalue: " << lqr.min_p_eigenvalue << "\n"
                  << "closed-loop spectral radius: " << lqr.closed_loop_radius << "\n"
                  << "K in scaled coordinates: [ ";
        for (double value : lqr.K) std::cout << value << " ";
        std::cout << "]\n";

    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        mj_deleteData(prediction_data);
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    const int run_result = headless
        ? run_headless(model, data, qpos_addresses, qvel_addresses, options)
        : run_viewer(model, data, qpos_addresses, qvel_addresses, options);

    mj_deleteData(prediction_data);
    mj_deleteData(data);
    mj_deleteModel(model);

    return run_result;
}
