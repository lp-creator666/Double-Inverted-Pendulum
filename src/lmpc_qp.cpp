//计算H,g,constant_cost

#include "lmpc_qp.hpp"

#include <cmath>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace {

constexpr int nx = kStateDimension;
constexpr int nu = LmpcConfig::horizon;

using Matrix6 = Eigen::Matrix<double, nx, nx>;
using Vector6 = Eigen::Matrix<double, nx, 1>;
using MatrixG = Eigen::Matrix<double, nx, nu>;
using MatrixH = Eigen::Matrix<double, nu, nu>;
using VectorU = Eigen::Matrix<double, nu, 1>;
using RowMajor6 = Eigen::Matrix<double, nx, nx, Eigen::RowMajor>;

} // namespace

LmpcQpProblem build_lmpc_qp(
    const MatrixA& As,
    const VectorB& Bs,
    const StateVector& z0,
    const LmpcConfig& config,
    const LqrConfig& weights,
    const MatrixA& terminal_cost
) {
    const Matrix6 A = Eigen::Map<const RowMajor6>(As.data());
    const Vector6 B = Eigen::Map<const Vector6>(Bs.data());
    const Vector6 initial = Eigen::Map<const Vector6>(z0.data());
    const Matrix6 P = Eigen::Map<const RowMajor6>(terminal_cost.data());

    if(!A.allFinite() || !B.allFinite() || !initial.allFinite() || !P.allFinite()){
        throw std::invalid_argument("Non-finite LMPC QP data");
    }

    if (!std::isfinite(config.input_min) ||
        !std::isfinite(config.input_max) ||
        config.input_min > config.input_max) {
        throw std::invalid_argument("Invalid LMPC input bounds");
    }

    if (!std::isfinite(weights.r) || weights.r <= 0.0) {
        throw std::invalid_argument("Invalid LMPC R");
    }

    Matrix6 Q = Matrix6::Zero();

    for (int i = 0; i < nx; ++i) {
        const double weight = weights.q_diagonal[i];

        if (!std::isfinite(weight) || weight <= 0.0) {
            throw std::invalid_argument("Invalid LMPC Q");
        }

        Q(i, i) = weight;
    }

    const double p_scale = std::max(1.0, P.cwiseAbs().maxCoeff());

    if ((P - P.transpose()).cwiseAbs().maxCoeff() > 1e-12 * p_scale) {
        throw std::invalid_argument("Terminal P is not symmetric");
    }

    // 当前使用 LQR 的正定终端矩阵。
    Eigen::LLT<Matrix6> p_check(P);

    if (p_check.info() != Eigen::Success) {
        throw std::invalid_argument("Terminal P is not positive definite");
    }

    // 输入代价 sum(R * v[k]^2)。
    MatrixH H = 2.0 * weights.r * MatrixH::Identity();
    VectorU g = VectorU::Zero();
    double constant_cost = 0.0;

    //核心 z0 = f + G*V，初始状态不由未来输入决定。
    Vector6 f = initial;
    MatrixG G = MatrixG::Zero();

    for (int k = 0; k <= nu; ++k) {
        const Matrix6& W = (k == nu) ? P : Q;

        H += 2.0 * G.transpose() * W * G;
        g += 2.0 * G.transpose() * W * f;
        constant_cost += f.dot(W * f);

        if (k < nu) {
            // 从 z_k 推进到 z_{k+1}。
            f = (A * f).eval();
            G = (A * G).eval();

            // 当前输入 v_k 通过 B 影响下一步状态。
            G.col(k) += B;
        }
    }

    H = (0.5 * (H + H.transpose())).eval();

    if (!H.allFinite() || !g.allFinite() ||
        !std::isfinite(constant_cost)) {
        throw std::runtime_error("LMPC QP construction overflow");
    }

    Eigen::LLT<MatrixH> h_check(H);

    if (h_check.info() != Eigen::Success) {
        throw std::runtime_error("LMPC Hessian is not positive definite");
    }

        LmpcQpProblem problem;
    problem.constant_cost = constant_cost;

    for (int row = 0; row < nu; ++row) {
        problem.g[row] = g[row];
        problem.lower_bound[row] = config.input_min;
        problem.upper_bound[row] = config.input_max;

        for (int col = 0; col < nu; ++col) {
            problem.H[row * nu + col] = H(row, col);
        }
    }

    return problem;
}