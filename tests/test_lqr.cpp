#include "lqr.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition) {
    if (!condition) throw std::runtime_error("LQR regression failed");
}
}

int main() {
    // Independent scalar unstable system plus five stable, unactuated states.
    MatrixA A{};
    A[0] = 1.1;
    for (int i = 1; i < kStateDimension; ++i) A[i * kStateDimension + i] = 0.5;
    VectorB B{};
    B[0] = 1.0;
    LqrConfig config;
    config.scaling = Scaling{{1, 1, 1, 1, 1, 1}, 1};
    config.q_diagonal.fill(1.0);
    const auto result = solve_discrete_lqr(A, B, config);
    // Scalar DARE: P^2 - a^2 P - 1 = 0 for Q=R=B=1.
    const double a = A[0];
    const double exact_p = (a*a + std::sqrt(a*a*a*a + 4.0)) / 2.0;
    const double exact_k = exact_p * a / (1.0 + exact_p);
    require(std::abs(result.P[0] - exact_p) < 1e-9);
    require(std::abs(result.K[0] - exact_k) < 1e-9);
    require(result.closed_loop_radius < 1.0);
    require(result.riccati_residual < 1e-10);
    for (int i = 1; i < kStateDimension; ++i) {
        require(std::abs(result.P[i * kStateDimension + i] - 4.0/3.0) < 1e-9);
        require(std::abs(result.K[i]) < 1e-12);
    }

    // Coordinate scaling must preserve the feedback in physical units.
    config.scaling.state_scale = {2, 3, 4, 5, 6, 7};
    config.scaling.input_scale = 3;
    for (int i = 0; i < kStateDimension; ++i)
        config.q_diagonal[i] = config.scaling.state_scale[i] * config.scaling.state_scale[i];
    config.r = 9;
    const auto scaled = solve_discrete_lqr(A, B, config);
    require(std::abs(scaled.K[0] * 3.0 / 2.0 - exact_k) < 1e-9);

    State state{};
    state.p = 0.1;
    require(std::abs(compute_lqr_force(state, scaled, config) + exact_k * 0.1) < 1e-9);
    require(compute_lqr_force(State{}, scaled, config) == 0.0);
    state.p = std::numeric_limits<double>::quiet_NaN();
    bool bad_state = false;
    try { compute_lqr_force(state, scaled, config); }
    catch (const std::invalid_argument&) { bad_state = true; }
    require(bad_state);

    config.r = 0;
    bool rejected = false;
    try { solve_discrete_lqr(A, B, config); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
    config.r = 9;
    config.max_iterations = 1;
    rejected = false;
    try { solve_discrete_lqr(A, B, config); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected);
    std::cout << "LQR analytic, scaling and failure checks passed\n";
}
