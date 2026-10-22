#include "model_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "controllability.hpp"
#include "dynamics.hpp"
#include "scaling.hpp"

namespace {
StateVector values(const State& x) {
    return {x.p, x.dp, x.q1, x.dq1, x.q2, x.dq2};
}
}

void run_model_diagnostics(const mjModel* model, mjData* prediction_data,
    const int* qpos_addresses, const int* qvel_addresses,
    const Linearization& linearization) {
    const Scaling scaling{{1.0, 1.0, 0.5, 2.0, 0.5, 2.0}, 20.0};
    const auto scaled = scale_linearization(linearization.A, linearization.B, scaling);
    std::cout << "\n长时域可控性（相对容差 1e-10，控制周期不变）:\n";
    for (int steps : {6, 50, 250}) {
        const auto raw = analyze_controllability_horizon(
            linearization.A, linearization.B, steps);
        const auto normalized = analyze_controllability_horizon(scaled.A, scaled.B, steps);
        std::cout << "  " << steps << " 步 / " << steps * linearization.timestep
                  << " s: rank=" << raw.rank << "/6, condition=" << raw.condition_number
                  << "; scaled rank=" << normalized.rank
                  << "/6, condition=" << normalized.condition_number << "\n";
    }
    std::cout << "六步矩阵病态不能单独作为物理系统不可控的证据。\n";
    const auto pbh = analyze_stabilizability(scaled.A, scaled.B);
    std::cout << "离散 PBH 检查（缩放坐标，相对容差 1e-10）:\n";
    for (const auto& mode : pbh.modes) {
        std::cout << "  lambda=" << mode.eigenvalue << ", |lambda|="
                  << std::abs(mode.eigenvalue) << ", rank=" << mode.rank
                  << "/6, sigma_min/max=" << mode.singular_value_ratio
                  << (mode.requires_control ? " [需要控制]" : " [稳定模态]") << "\n";
    }
    std::cout << "局部线性模型数值可稳定性: "
              << (pbh.stabilizable ? "通过" : "未通过") << "\n";

    const auto equilibrium = values(predict_state(model, prediction_data, State{},
        qpos_addresses, qvel_addresses, 0.0, linearization.prediction_steps));
    double residual = 0.0;
    for (int i = 0; i < kStateDimension; ++i)
        residual = std::max(residual, std::abs(equilibrium[i]) / scaling.state_scale[i]);
    std::cout << "直立零输入平衡点残差（归一化无穷范数）: " << residual << "\n";

    double difference_change = 0.0;
    double reference_size = 0.0;
    for (double v : linearization.A) reference_size = std::max(reference_size, std::abs(v));
    for (double v : linearization.B) reference_size = std::max(reference_size, std::abs(v));
    for (double epsilon : {1e-4, 1e-5, 1e-7}) {
        const auto other = linearize_discrete(model, prediction_data, State{}, 0.0,
            qpos_addresses, qvel_addresses, linearization.prediction_steps, epsilon, epsilon);
        double change = 0.0;
        for (std::size_t i = 0; i < other.A.size(); ++i)
            change = std::max(change, std::abs(other.A[i] - linearization.A[i]));
        for (std::size_t i = 0; i < other.B.size(); ++i)
            change = std::max(change, std::abs(other.B[i] - linearization.B[i]));
        difference_change = std::max(difference_change, change / reference_size);
    }
    std::cout << "差分步长 1e-4/1e-5/1e-7：A、B 最大元素变化 / 基准最大元素 = "
              << difference_change << "\n";

    double worst_relative_error = 0.0;
    std::cout << "小扰动预测对照（所有状态及正负输入，最大归一化误差）:\n";
    for (int steps : {1, 50}) {
        double max_error = 0.0;
        double max_relative_error = 0.0;
        for (int component = 0; component <= kStateDimension; ++component) {
            for (double sign : {-1.0, 1.0}) {
                StateVector x{};
                if (component < kStateDimension)
                    x[component] = sign * 1e-4 * scaling.state_scale[component];
                const State initial{x[0], x[1], x[2], x[3], x[4], x[5]};
                const double force = component == kStateDimension ? sign * 1e-3 : 0.0;
                const auto nonlinear = values(predict_state(model, prediction_data, initial,
                    qpos_addresses, qvel_addresses, force, steps * linearization.prediction_steps));
                for (int k = 0; k < steps; ++k) {
                    StateVector next{};
                    for (int row = 0; row < kStateDimension; ++row) {
                        next[row] = linearization.B[row] * force;
                        for (int col = 0; col < kStateDimension; ++col)
                            next[row] += linearization.A[row * kStateDimension + col] * x[col];
                    }
                    x = next;
                }
                double error = 0.0, size = 0.0;
                for (int i = 0; i < kStateDimension; ++i) {
                    error = std::max(error, std::abs(x[i] - nonlinear[i]) / scaling.state_scale[i]);
                    size = std::max(size, std::abs(nonlinear[i]) / scaling.state_scale[i]);
                }
                max_error = std::max(max_error, error);
                max_relative_error = std::max(max_relative_error, error / std::max(size, 1e-15));
            }
        }
        worst_relative_error = std::max(worst_relative_error, max_relative_error);
        std::cout << "  " << steps * linearization.timestep << " s: error=" << max_error
                  << ", relative=" << max_relative_error << "\n";
    }
    // Regression limits for these deliberately small, local perturbations only.
    if (!pbh.stabilizable || residual > 1e-10 || difference_change > 1e-5 ||
        worst_relative_error > 1e-3)
        throw std::runtime_error("局部模型诊断未通过，请检查输出");
    std::cout << "局部模型诊断通过；下一步可设计 LQR，仍需验证闭环与输入限幅。\n";
}
