#include "optimization/nonlinear_model.hpp"

#include "model/dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
double state_norm(const State& state) {
    return std::max({std::abs(state.p), std::abs(state.dp),
                     std::abs(state.q1), std::abs(state.dq1),
                     std::abs(state.q2), std::abs(state.dq2)});
}
State from_array(const std::array<double, 6>& values) {
    return {values[0], values[1], values[2], values[3], values[4], values[5]};
}
}

int main() try {
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML("model/double_cartpole.xml", nullptr, error, sizeof(error)), mj_deleteModel);
    require(bool(model), error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    require(bool(data), "Data allocation failed");
    const char* names[] = {"cart_slide", "pole_1_hinge", "pole_2_hinge"};
    int qpos[3]{}, qvel[3]{};
    for (int i = 0; i < 3; ++i) {
        const int id = mj_name2id(model.get(), mjOBJ_JOINT, names[i]);
        require(id >= 0, "Missing joint");
        qpos[i] = model->jnt_qposadr[id]; qvel[i] = model->jnt_dofadr[id];
    }
    const State initial{0.08, -0.03, 0.12, 0.02, -0.06, 0.01};
    const std::vector<double> forces{1.0, -0.5};
    const auto trajectory = predict_nonlinear_horizon(
        model.get(), data.get(), initial, qpos, qvel, forces, 4);
    const auto linearized = linearize_nmpc_trajectory(
        model.get(), data.get(), trajectory.states, forces,
        qpos, qvel, 4);
    require(linearized.segments.size() == forces.size(), "Wrong segment count");
    require(linearized.segments[0].A[0] > 0.99 &&
            linearized.segments[0].A[0] < 1.01, "Unexpected A position entry");

    // First-order perturbation must predict the nonlinear map to second order.
    const double eps = 1e-5;
    State perturbed = initial; perturbed.q1 += eps;
    const State actual = predict_state(model.get(), data.get(), perturbed,
                                       qpos, qvel, forces[0], 4);
    const auto& A = linearized.segments[0].A;
    const auto& base = linearized.segments[0].predicted_next;
    const double predicted_q1 = base.q1 + A[2 * kStateDimension + 2] * eps;
    require(std::abs(actual.q1 - predicted_q1) < 2e-8,
            "A linearization does not predict local perturbation");
    const double old_a = A[2 * kStateDimension + 2];
    const auto finer = linearize_nmpc_trajectory(
        model.get(), data.get(), trajectory.states, forces,
        qpos, qvel, 4, 5e-7, 5e-6);
    require(std::abs(old_a - finer.segments[0].A[2 * kStateDimension + 2]) < 1e-5,
            "Linearization is too sensitive to epsilon");
    const auto upper_trajectory = predict_nonlinear_horizon(
        model.get(), data.get(), initial,
        qpos, qvel, {model->actuator_ctrlrange[1]}, 4);
    const auto upper_linearization = linearize_nmpc_trajectory(
        model.get(), data.get(), upper_trajectory.states,
        {model->actuator_ctrlrange[1]}, qpos, qvel, 4);
    for (double value : upper_linearization.segments.front().B) {
        require(std::isfinite(value), "One-sided boundary B is not finite");
    }

    bool rejected = false;
    try { linearize_nmpc_trajectory(model.get(), data.get(), trajectory.states,
                                    forces, qpos, qvel, 0); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Invalid linearization period accepted");
    std::cout << "NMPC trajectory linearization checks passed\n";
    return 0;
} catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
}
