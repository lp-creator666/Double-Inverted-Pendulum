#include "control/lmpc.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool ok) { if (!ok) throw std::runtime_error("LMPC prediction test failed"); }

int main() {
    MatrixA A{};
    for (int i = 0; i < kStateDimension; ++i) A[i * kStateDimension + i] = 1.0;
    VectorB B{}; B[0] = 1.0;
    StateVector x{}; x[0] = 0.5;
    std::array<double, LmpcConfig::horizon> u{}; u[0] = 0.25;
    const auto prediction = predict_linear_horizon(A, B, x, u, 3);
    require(prediction.steps == 3);
    require(std::abs(prediction.states[0] - 0.5) < 1e-12);
    require(std::abs(prediction.states[6] - 0.75) < 1e-12);
    require(std::abs(prediction.states[12] - 0.75) < 1e-12);
    LmpcConfig config;
    require(check_lmpc_constraints(prediction, config));
    u[0] = 1.5;
    const auto invalid = predict_linear_horizon(A, B, x, u, 3);
    require(!check_lmpc_constraints(invalid, config));
    bool rejected = false;
    try { predict_linear_horizon(A, B, x, u, LmpcConfig::horizon + 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
    std::cout << "LMPC prediction and constraint checks passed\n";
}
