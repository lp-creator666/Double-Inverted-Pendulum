#include "optimization/nonlinear_model.hpp"
#include "model/dynamics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
std::array<double, 6> values(const State& x) {
    return {x.p, x.dp, x.q1, x.dq1, x.q2, x.dq2};
}
double error(const State& a, const State& b) {
    const auto av = values(a), bv = values(b);
    double result = 0;
    for (int i = 0; i < 6; ++i) result = std::max(result, std::abs(av[i] - bv[i]));
    return result;
}
int main() {
    char message[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML("model/double_cartpole.xml", nullptr, message, sizeof(message)), mj_deleteModel);
    require(bool(model), message);
    auto make_data = [&] {
        return std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(model.get()), mj_deleteData);
    };
    auto prediction = make_data(), reference = make_data(), live = make_data();
    require(prediction && reference && live, "Data allocation failed");
    const char* names[] = {"cart_slide", "pole_1_hinge", "pole_2_hinge"};
    int qp[3], qv[3];
    for (int i = 0; i < 3; ++i) {
        const int id = mj_name2id(model.get(), mjOBJ_JOINT, names[i]);
        require(id >= 0, "Missing joint");
        qp[i] = model->jnt_qposadr[id]; qv[i] = model->jnt_dofadr[id];
    }
    auto predict = [&](State x, const std::vector<double>& forces, int steps) {
        return predict_nonlinear_horizon(model.get(), prediction.get(), x, qp, qv, forces, steps);
    };
    const auto zero = predict(State{}, {0,0,0}, 10);
    require(zero.states.size() == 4, "Wrong trajectory size");
    require(std::abs(zero.control_dt - 0.02) < 1e-14, "Wrong control period");
    for (const auto& x : zero.states) require(error(x, State{}) < 1e-12, "Equilibrium drift");
    State initial{0.1, -0.05, 0.3, 0.1, -0.2, -0.1};
    const auto one = predict(initial, {2.0}, 10);
    const auto expected = predict_state(model.get(), reference.get(), initial, qp, qv, 2.0, 10);
    require(error(one.states.back(), expected) < 1e-12, "Single segment mismatch");
    const auto iv = values(initial);
    for (int i = 0; i < 3; ++i) {
        reference->qpos[qp[i]] = iv[2*i]; reference->qvel[qv[i]] = iv[2*i+1];
    }
    reference->time = 0;
    mj_forward(model.get(), reference.get());
    live->qpos[qp[0]] = 0.37; live->time = 1.25; live->ctrl[0] = 3.0;
    const int spec = mjSTATE_INTEGRATION;
    std::vector<mjtNum> before(mj_stateSize(model.get(), spec)), after(before.size());
    mj_getState(model.get(), live.get(), before.data(), spec);
    const std::vector<double> forces{1, -2, 0, 3, -1};
    const auto trajectory = predict(initial, forces, 10);
    require(error(trajectory.states.front(), initial) == 0, "Initial state lost");
    double peak_error = 0;
    for (std::size_t k = 0; k < forces.size(); ++k) {
        for (int step = 0; step < 10; ++step) {
            reference->ctrl[0] = forces[k]; mj_step(model.get(), reference.get());
        }
        peak_error = std::max(peak_error, error(trajectory.states[k+1], read_state(reference.get(), qp, qv)));
    }
    require(peak_error < 1e-9, "Continuous simulation mismatch");
    mj_getState(model.get(), live.get(), after.data(), spec);
    require(before == after, "Live simulation changed");
    // Reusing the scratch data must not retain the previous rollout.
    const auto repeated = predict(initial, forces, 10);
    require(error(repeated.states.back(), trajectory.states.back()) < 1e-12, "Prediction retained stale state");
    auto reject = [&](const std::vector<double>& inputs, int steps, State x = {}) {
        bool rejected = false;
        try { predict(x, inputs, steps); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Invalid input accepted");
    };
    reject({}, 10); reject({0}, 0); reject({0}, -1);
    reject({0, model->actuator_ctrlrange[1] + 1}, 10);
    reject({model->actuator_ctrlrange[0] - 1}, 10);
    reject({std::numeric_limits<double>::quiet_NaN()}, 10);
    State invalid; invalid.q1 = std::numeric_limits<double>::infinity();
    reject({0}, 10, invalid);
    predict(State{}, {model->actuator_ctrlrange[0], model->actuator_ctrlrange[1]}, 1);
    bool rejected = false;
    try { predict_nonlinear_horizon(nullptr, prediction.get(), {}, qp, qv, {0}, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Null model accepted");
    const int bad[] = {model->nq, qp[1], qp[2]}; rejected = false;
    try { predict_nonlinear_horizon(model.get(), prediction.get(), {}, bad, qv, {0}, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Invalid address accepted");
    std::cout << "Nonlinear prediction checks passed; continuous rollout max error: " << peak_error << '\n';
}
