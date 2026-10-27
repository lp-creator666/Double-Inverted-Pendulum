#include "optimization/nmpc_cost.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template <typename F>
void rejects(F function) {
    bool rejected = false;
    try { function(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid trajectory input was accepted");
}
}

int main() try {
    NmpcCostConfig state_config;
    require(compute_nmpc_state_cost({},state_config)==0,"Upright cost must be zero");
    const State moving{0.2,-0.3,0.4,0.5,-0.7,-0.2};
    State turned=moving;
    turned.q1+=2*std::acos(-1.0); turned.q2-=4*std::acos(-1.0);
    require(std::abs(compute_nmpc_state_cost(moving,state_config)-
        compute_nmpc_state_cost(turned,state_config))<1e-10,"Angle periodicity was lost");
    state_config.state_weights={0,0,0,0,1,1};
    State relative{}; relative.q1=0.4;relative.q2=-0.4;
    relative.dq1=2;relative.dq2=-2;
    require(compute_nmpc_state_cost(relative,state_config)==0,
        "Second pole cost must use absolute orientation and speed");
    relative.q2=0;relative.dq2=0;
    const double expected=4*std::pow(std::sin(0.2),2)+4;
    require(std::abs(compute_nmpc_state_cost(relative,state_config)-expected)<1e-12,
        "Absolute state weight mismatch");
    state_config.state_weights[0]=-1;
    rejects([&]{compute_nmpc_state_cost({},state_config);});
    state_config.state_weights[0]=0;
    relative.q1=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{compute_nmpc_state_cost(relative,state_config);});
    NmpcPrediction prediction;
    prediction.states = {State{}, State{}, State{}};
    const std::vector<double> forces{2.0, -1.0};

    NmpcCostConfig config;
    config.state_weights.fill(0.0);
    config.input_weight = 0.01;
    config.input_change_weight = 0.1;
    config.terminal_weight = 5.0;

    // 0.01*(2^2+(-1)^2) + 0.1*((2-0.5)^2+(-1-2)^2) = 1.175.
    const double input_cost = compute_nmpc_trajectory_cost(
        prediction, forces, config, 0.5);
    require(std::abs(input_cost - 1.175) < 1e-12,
            "input and slew cost mismatch");

    config.input_weight = 0.0;
    config.input_change_weight = 0.0;
    config.terminal_weight = 2.0;
    config.state_weights[0] = 10.0;
    prediction.states.back().p = 1.0;
    const double terminal_cost = compute_nmpc_trajectory_cost(
        prediction, {0.0, 0.0}, config);
    require(std::abs(terminal_cost - 20.0) < 1e-12,
        "terminal cost mismatch");

    prediction.states = {State{}, State{}};
    rejects([&] { compute_nmpc_trajectory_cost(prediction, {}, config); });
    rejects([&] { compute_nmpc_trajectory_cost(
        prediction, {std::numeric_limits<double>::quiet_NaN()}, config); });
    config.input_weight = -1.0;
    rejects([&] { compute_nmpc_trajectory_cost(prediction, {0.0}, config); });
    config.input_weight = 0.0;
    rejects([&] { compute_nmpc_trajectory_cost(
        prediction, {0.0}, config, std::numeric_limits<double>::infinity()); });
    std::cout << "NMPC trajectory cost checks passed\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
