#include "control/lmpc.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "optimization/dense_qp.hpp"

//计算H,g,constant_cost

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

    // Convert physical state limits into linear inequalities on V:
    // lower <= f_i + G_i V <= upper.
    LmpcQpProblem problem;
    constexpr int constrained_state_indices[] = {0, 1, 3, 5};
    const double physical_lower[] = {
        config.cart_position_min, config.cart_speed_min,
        config.joint_speed_min, config.joint_speed_min};
    const double physical_upper[] = {
        config.cart_position_max, config.cart_speed_max,
        config.joint_speed_max, config.joint_speed_max};
    for (int k = 0; k <= nu; ++k) {
        for (int c = 0; c < 4; ++c) {
            const int row = problem.constraint_count++;
            const int state_index = constrained_state_indices[c];
            const double scale = config.scaling.state_scale[state_index];
            if (!std::isfinite(scale) || scale <= 0.0)
                throw std::invalid_argument("Invalid LMPC state scale");
            const double lower = physical_lower[c] / scale - f[state_index];
            const double upper = physical_upper[c] / scale - f[state_index];
            problem.constraint_lower[row] = lower;
            problem.constraint_upper[row] = upper;
            for (int column = 0; column < nu; ++column)
                problem.constraint_matrix[row * nu + column] = G(state_index, column);
            if (lower > upper)
                throw std::invalid_argument("Invalid LMPC state constraint");
        }
        if (k < nu) {
            f = (A * f).eval();
            G = (A * G).eval();
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

namespace {
constexpr int n = LmpcQpProblem::variable_count;
void validate(const LmpcQpProblem& p, const LmpcQpSolverConfig& c) {
    if (c.max_iterations <= 0 || !std::isfinite(c.tolerance) || c.tolerance <= 0)
        throw std::invalid_argument("Invalid box QP solver configuration");
    if (p.constraint_count < 0 || p.constraint_count > LmpcQpProblem::max_constraints)
        throw std::invalid_argument("Invalid QP constraint count");
    for (int r=0;r<p.constraint_count;++r) {
        if (!std::isfinite(p.constraint_lower[r]) || !std::isfinite(p.constraint_upper[r]) ||
            p.constraint_lower[r] > p.constraint_upper[r])
            throw std::invalid_argument("Invalid QP state bounds");
        for (int j=0;j<n;++j)
            if (!std::isfinite(p.constraint_matrix[r*n+j]))
                throw std::invalid_argument("Non-finite QP constraint");
    }
    for (int i=0;i<n;++i) {
        if (!std::isfinite(p.g[i]) || !std::isfinite(p.lower_bound[i]) ||
            !std::isfinite(p.upper_bound[i]) || p.lower_bound[i] > p.upper_bound[i])
            throw std::invalid_argument("Invalid box QP bounds");
        for (int j=0;j<n;++j)
            if (!std::isfinite(p.H[i*n+j])) throw std::invalid_argument("Non-finite Hessian");
    }
}
double cost(const LmpcQpProblem& p,const std::array<double,n>& x) {
    double value=p.constant_cost;
    for(int i=0;i<n;++i) {
        value+=p.g[i]*x[i];
        for(int j=0;j<n;++j) value+=0.5*x[i]*p.H[i*n+j]*x[j];
    }
    return value;
}
}
// 将 LMPC 的约束转换为通用 QP，并保留最优性诊断。
LmpcQpSolution solve_box_qp(const LmpcQpProblem& p,const LmpcQpSolverConfig& c) {
    validate(p,c);
    using RowMatrix=Eigen::Matrix<double,n,n,Eigen::RowMajor>;
    const Eigen::MatrixXd H=Eigen::Map<const RowMatrix>(p.H.data());
    const Eigen::VectorXd g=Eigen::Map<const Eigen::Matrix<double,n,1>>(p.g.data());
    Eigen::MatrixXd C=Eigen::MatrixXd::Zero(2*(n+p.constraint_count),n);
    Eigen::VectorXd bounds(C.rows());
    C.topRows(n)=Eigen::MatrixXd::Identity(n,n);
    C.middleRows(n,n)=-Eigen::MatrixXd::Identity(n,n);
    for(int i=0;i<n;++i) {bounds[i]=p.upper_bound[i];bounds[n+i]=-p.lower_bound[i];}
    for(int r=0;r<p.constraint_count;++r) {
        for(int j=0;j<n;++j) {
            C(2*n+2*r,j)=p.constraint_matrix[r*n+j];
            C(2*n+2*r+1,j)=-p.constraint_matrix[r*n+j];
        }
        bounds[2*n+2*r]=p.constraint_upper[r];
        bounds[2*n+2*r+1]=-p.constraint_lower[r];
    }
    const auto qp=solve_dense_qp(H,g,C,bounds,c.max_iterations,c.tolerance);
    LmpcQpSolution result;
    result.solved=qp.solved;result.iterations=qp.iterations;
    result.primal_residual=qp.primal_residual;
    result.stationarity_residual=qp.stationarity_residual;
    result.complementarity_residual=qp.complementarity_residual;
    for(int i=0;i<n;++i) result.variables[i]=qp.step[i];
    result.objective=cost(p,result.variables);
    return result;
}
