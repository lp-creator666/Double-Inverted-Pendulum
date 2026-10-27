#include "nmpc_shooting.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "model/dynamics.hpp"
#include "optimization/nonlinear_model.hpp"

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
double max_error(const State& state) {
    return std::max({std::abs(state.p), std::abs(state.dp),
                     std::abs(state.q1), std::abs(state.dq1),
                     std::abs(state.q2), std::abs(state.dq2)});
}
}

int main() try {
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML("model/double_cartpole.xml", nullptr, error, sizeof(error)),
        mj_deleteModel);
    require(bool(model), error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    require(bool(data), "Data allocation failed");

    const char* names[] = {"cart_slide", "pole_1_hinge", "pole_2_hinge"};
    int qpos[3]{}, qvel[3]{};
    for (int i = 0; i < 3; ++i) {
        const int id = mj_name2id(model.get(), mjOBJ_JOINT, names[i]);
        require(id >= 0, "Missing joint");
        qpos[i] = model->jnt_qposadr[id];
        qvel[i] = model->jnt_dofadr[id];
    }

    const State initial{0.05, -0.02, 0.1, 0.03, -0.04, 0.02};
    const std::vector<double> forces{1.0, -0.5, 0.0};
    const auto predicted = predict_nonlinear_horizon(
        model.get(), data.get(), initial, qpos, qvel, forces, 5);
    const auto valid = compute_nmpc_dynamics_defects(
        model.get(), data.get(), predicted.states, forces,
        qpos, qvel, 5);
    require(valid.defects.size() == forces.size(), "Wrong defect count");
    require(valid.infinity_norm < 1e-12, "Predicted trajectory has defects");
    for (const State& defect : valid.defects) {
        require(max_error(defect) < 1e-12, "Nonzero valid defect");
    }

    auto broken_states = predicted.states;
    broken_states[1].p += 0.01;
    const auto broken = compute_nmpc_dynamics_defects(
        model.get(), data.get(), broken_states, forces,
        qpos, qvel, 5);
    require(broken.infinity_norm > 1e-3, "Broken trajectory was accepted");

    bool rejected = false;
    try {
        compute_nmpc_dynamics_defects(
            model.get(), data.get(), {initial}, forces, qpos, qvel, 5);
    } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Mismatched trajectory was accepted");
    auto rejects = [&](const int* addresses, int steps, std::vector<double> inputs) {
        bool failed = false;
        try {
            compute_nmpc_dynamics_defects(model.get(), data.get(),
                predicted.states, inputs, addresses, qvel, steps);
        } catch (const std::invalid_argument&) { failed = true; }
        require(failed, "Invalid shooting input accepted");
    };
    rejects(nullptr, 5, forces);
    rejects(qpos, 0, forces);
    auto bad_forces = forces; bad_forces.back() = 21.0;
    rejects(qpos, 5, bad_forces);
    // Terminal-only perturbation must appear with the chosen positive sign.
    auto terminal = predicted.states; terminal.back().q2 += 0.02;
    const auto signed_defect = compute_nmpc_dynamics_defects(
        model.get(), data.get(), terminal, forces, qpos, qvel, 5);
    require(std::abs(signed_defect.defects.back().q2 - 0.02) < 1e-12,
        "Wrong defect sign or terminal indexing");
    require(max_error(signed_defect.defects.front()) < 1e-12,
        "Terminal perturbation changed earlier defect");

    std::cout << "NMPC multiple-shooting defect checks passed\n";
    return 0;
} catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
}
