#include "optimization/dense_qp.hpp"
#include <iostream>
#include <stdexcept>
void require(bool ok) { if (!ok) throw std::runtime_error("Dense QP check failed"); }
int main() try {
    Eigen::MatrixXd H=Eigen::MatrixXd::Identity(2,2), C(3,2);
    Eigen::VectorXd g(2), b(3);
    g << -2,-3; C << 1,1,-1,0,0,-1; b << 1,0,0;
    const auto result=solve_dense_qp(H,g,C,b);
    require(result.solved && std::abs(result.step[0])<1e-6 && std::abs(result.step[1]-1)<1e-6);
    const auto scaled=solve_dense_qp(1e8*H,1e8*g,C,b);
    require(scaled.solved && (scaled.step-result.step).norm()<1e-6);
    C.resize(0,2); b.resize(0);
    const auto free=solve_dense_qp(H,g,C,b);
    require(free.solved && (free.step+g).norm()<1e-10);
    C.resize(2,2); b.resize(2); C << 1,0,-1,0; b << -1,-1;
    require(!solve_dense_qp(H,g,C,b,100).solved);
    // Many redundant inactive bounds must not change the active-set solution.
    C.resize(204,2); b.resize(204);
    C.topRows(4) << -1,0,0,-1,1,0,0,1;
    b.head(4) << 0,0,1,1;
    for(int i=4;i<204;++i) {C.row(i)<<1,1;b[i]=2+(i%7);}
    g << -2,1;
    const auto box=solve_dense_qp(H,g,C,b);
    require(box.solved && (box.step-Eigen::Vector2d(1,0)).norm()<1e-6);
    require(box.primal_residual<1e-7 && box.stationarity_residual<1e-7);
    // Constraints through the initial feasible point, including duplicates.
    C.resize(5,2); b.resize(5);
    C << -1,0,0,-1,1,1,2,2,0,1; b << 0,0,1,2,1;
    g << 1,-2;
    const auto degenerate=solve_dense_qp(H,g,C,b);
    require(degenerate.solved && (degenerate.step-Eigen::Vector2d(0,1)).norm()<1e-6);
    const auto timeout=solve_dense_qp(H,g,C,b,2000,1e-7,1e-12);
    require(timeout.timed_out && !timeout.solved);
    std::cout << "Dense QP analytical, coupled constraint and infeasibility checks passed\n";
    return 0;
} catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
