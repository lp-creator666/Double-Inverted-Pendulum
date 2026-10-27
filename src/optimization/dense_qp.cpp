#include "optimization/dense_qp.hpp"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

DenseQpResult solve_dense_qp(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
    const Eigen::MatrixXd& C, const Eigen::VectorXd& bounds,
    int max_iterations, double tolerance, double max_seconds) {
    const auto started=std::chrono::steady_clock::now();
    auto expired=[&]{return max_seconds>0 && std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>=max_seconds;};
    if (H.rows() == 0 || H.rows() != H.cols() || H.rows() != g.size() ||
        C.cols() != H.cols() || C.rows() != bounds.size() ||
        !H.allFinite() || !g.allFinite() || !C.allFinite() || !bounds.allFinite() ||
        max_iterations <= 0 || !std::isfinite(tolerance) || tolerance <= 0 || !std::isfinite(max_seconds) || max_seconds<0 ||
        (H-H.transpose()).cwiseAbs().maxCoeff() > 1e-9)
        throw std::invalid_argument("Invalid dense QP");
    Eigen::LLT<Eigen::MatrixXd> factor(H);
    if (factor.info() != Eigen::Success) throw std::invalid_argument("QP Hessian is not SPD");
    DenseQpResult result;
    // A feasible SQP iterate gives a feasible zero step. Use a primal active
    // set so thousands of inactive trajectory bounds need only a matrix scan.
    if (bounds.size() == 0 || bounds.minCoeff() >= 0.0) {
        result.step = Eigen::VectorXd::Zero(g.size());
        std::vector<int> active;
        for (int iteration=0; iteration<max_iterations; ++iteration) {
            if(expired()) {result.timed_out=true;return result;}
            const Eigen::VectorXd gradient = H*result.step+g;
            Eigen::VectorXd direction = factor.solve(-gradient);
            Eigen::VectorXd lambda(active.size());
            Eigen::MatrixXd A(active.size(),g.size());
            for (int i=0;i<A.rows();++i) A.row(i)=C.row(active[i]);
            if (!active.empty()) {
                const Eigen::MatrixXd D=factor.solve(A.transpose());
                const Eigen::MatrixXd S=A*D;
                lambda=S.completeOrthogonalDecomposition().solve(A*direction);
                direction-=D*lambda;
            }
            result.iterations=iteration+1;
            if (direction.lpNorm<Eigen::Infinity>() <= tolerance) {
                Eigen::Index remove=0;
                if (lambda.size() && lambda.minCoeff(&remove)<-tolerance) {
                    active.erase(active.begin()+remove);
                    continue;
                }
                const Eigen::VectorXd slack=C*result.step-bounds;
                result.primal_residual=slack.size()?std::max(0.0,slack.maxCoeff()):0;
                result.stationarity_residual=(gradient+A.transpose()*lambda).lpNorm<Eigen::Infinity>();
                result.complementarity_residual=0;
                for(int i=0;i<lambda.size();++i)
                    result.complementarity_residual=std::max(result.complementarity_residual,
                        std::abs(lambda[i]*slack[active[i]]));
                const double dual_scale=1+g.lpNorm<Eigen::Infinity>()+(H*result.step).lpNorm<Eigen::Infinity>()+
                    (A.transpose()*lambda).lpNorm<Eigen::Infinity>();
                const double cost_scale=1+std::abs(0.5*result.step.dot(H*result.step)+g.dot(result.step));
                result.solved=result.primal_residual<=tolerance &&
                    result.stationarity_residual<=tolerance*dual_scale && result.complementarity_residual<=tolerance*cost_scale;
                return result;
            }
            const Eigen::VectorXd slope=C*direction;
            const Eigen::VectorXd slack=bounds-C*result.step;
            double alpha=1; int blocking=-1;
            for(int i=0;i<C.rows();++i) {
                if(slope[i]<=1e-12 || std::find(active.begin(),active.end(),i)!=active.end()) continue;
                const double candidate=std::max(0.0,slack[i])/slope[i];
                if(candidate<alpha) {alpha=candidate;blocking=i;}
            }
            result.step+=alpha*direction;
            if(blocking>=0) active.push_back(blocking);
            if(!result.step.allFinite()) return result;
        }
        return result;
    }
    result.step = factor.solve(-g);
    const Eigen::MatrixXd directions = factor.solve(C.transpose());
    Eigen::VectorXd multipliers = Eigen::VectorXd::Zero(C.rows());
    for (int iteration=0; iteration<max_iterations; ++iteration) {
        if(expired()) {result.timed_out=true;return result;}
        for (int i=0; i<C.rows(); ++i) {
            if(i%128==0 && expired()) {result.timed_out=true;return result;}
            const double diagonal = C.row(i).dot(directions.col(i));
            const double violation = C.row(i).dot(result.step) - bounds[i];
            if (diagonal < 1e-20) {
                if (violation > tolerance) return result;
                continue;
            }
            const double next = std::max(0.0, multipliers[i] + violation/diagonal);
            result.step -= (next-multipliers[i])*directions.col(i);
            multipliers[i] = next;
        }
        result.iterations = iteration+1;
        const Eigen::VectorXd violation = C*result.step-bounds;
        result.primal_residual = violation.size() ? std::max(0.0,violation.maxCoeff()) : 0;
        result.stationarity_residual = (H*result.step+g+C.transpose()*multipliers).lpNorm<Eigen::Infinity>();
        result.complementarity_residual = violation.size() ?
            (multipliers.array()*violation.array()).abs().maxCoeff() : 0;
        const double dual_scale=1+g.lpNorm<Eigen::Infinity>()+(H*result.step).lpNorm<Eigen::Infinity>()+
            (C.transpose()*multipliers).lpNorm<Eigen::Infinity>();
        const double cost_scale=1+std::abs(0.5*result.step.dot(H*result.step)+g.dot(result.step));
        if (result.primal_residual <= tolerance && result.stationarity_residual <= tolerance*dual_scale &&
            result.complementarity_residual <= tolerance*cost_scale) {
            result.solved = true;
            break;
        }
        if (!result.step.allFinite() || !multipliers.allFinite()) break;
    }
    return result;
}
