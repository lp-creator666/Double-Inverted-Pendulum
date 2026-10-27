#include "model/diagnostics.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition) {
    if (!condition) throw std::runtime_error("SVD regression check failed");
}
int main() {
    MatrixA chain{};
    for (int i = 1; i < kStateDimension; ++i)
        chain[i * kStateDimension + i - 1] = 1.0;
    VectorB input{};
    input[0] = 1.0;
    const auto full = analyze_controllability(chain, input);
    require(full.rank == 6);
    require(std::abs(full.condition_number - 1.0) < 1e-12);
    for (double value : full.singular_values)
        require(std::abs(value - 1.0) < 1e-12);

    const auto zero = analyze_controllability(chain, VectorB{});
    require(zero.rank == 0 && std::isinf(zero.condition_number));
    const auto partial = analyze_controllability(MatrixA{}, input);
    require(partial.rank == 1 && std::isinf(partial.condition_number));

    // C is diagonal with known small singular values: no C^T C rounding.
    chain[1 * kStateDimension] = 1e-3;
    chain[2 * kStateDimension + 1] = 1e-3;
    chain[3 * kStateDimension + 2] = 1e-3;
    chain[4 * kStateDimension + 3] = 1e-3;
    chain[5 * kStateDimension + 4] = 1e-3;
    const auto tiny = analyze_controllability(chain, input, 1e-16);
    require(tiny.rank == 6);
    require(std::abs(tiny.singular_values.back() / 1e-15 - 1.0) < 1e-12);
    require(analyze_controllability(chain, input, 1e-10).rank == 4);

    const auto horizon = analyze_controllability_horizon(chain, input, 6, 1e-16);
    require(horizon.rank == tiny.rank);
    for (int i = 0; i < kStateDimension; ++i)
        require(std::abs(horizon.singular_values[i] - tiny.singular_values[i]) < 1e-16);
    require(analyze_controllability_horizon(chain, input, 50, 1e-16).rank == 6);
    bool invalid_horizon = false;
    try { analyze_controllability_horizon(chain, input, 5); }
    catch (const std::invalid_argument&) { invalid_horizon = true; }
    require(invalid_horizon);

    MatrixA diagonal{};
    for (int i = 0; i < kStateDimension; ++i) diagonal[i * kStateDimension + i] = 0.5;
    require(analyze_stabilizability(diagonal, VectorB{}).stabilizable);
    diagonal[0] = 1.1;
    require(!analyze_stabilizability(diagonal, VectorB{}).stabilizable);
    require(analyze_stabilizability(diagonal, input).stabilizable);
    diagonal[0] = 1.0;
    require(!analyze_stabilizability(diagonal, VectorB{}).stabilizable);
    // Complex unstable pair, controlled through its first coordinate.
    diagonal[0] = 1.05;
    diagonal[1] = -0.2;
    diagonal[kStateDimension] = 0.2;
    diagonal[kStateDimension + 1] = 1.05;
    require(analyze_stabilizability(diagonal, input).stabilizable);
    require(!analyze_stabilizability(diagonal, VectorB{}).stabilizable);

    input[0] = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try { analyze_controllability(chain, input); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
    std::cout << "Controllability SVD regression checks passed\n";
}
