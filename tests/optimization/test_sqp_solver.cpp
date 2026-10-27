#include "optimization/sqp_solver.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
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
    SqpConfig config;
    config.max_iterations = 3;
    config.steps_per_control = 2;
    config.max_line_search = 4;
    const State initial{0.0, 0.0, 0.03, 0.0, 0.0, 0.0};
    const std::vector<double> seed(4, 0.0);
    NmpcCostConfig weights;
    const SqpResult result = solve_nmpc_sqp(
        model.get(), data.get(), initial, qpos, qvel, seed, weights, config);
    auto full_difference=config;full_difference.use_chain_derivatives=false;
    const auto reference=solve_nmpc_sqp(model.get(),data.get(),initial,qpos,qvel,seed,weights,full_difference);
    require(std::abs(result.final_cost-reference.final_cost)<1e-6,"Chain derivatives changed SQP cost");
    for(std::size_t k=0;k<seed.size();++k)
        require(std::abs(result.forces[k]-reference.forces[k])<1e-4,"Chain derivatives disagree with full rollout differences");
    require(result.feasible, "SQP result is infeasible");
    require(result.forces.size() == seed.size(), "SQP changed horizon size");
    require(result.final_cost <= result.initial_cost + 1e-10,
            "SQP did not reduce or preserve cost");
    for (double force : result.forces) {
        require(std::isfinite(force) && force >= -20.0 && force <= 20.0,
                "SQP force left actuator bounds");
    }
    const auto zero = solve_nmpc_sqp(model.get(), data.get(), {}, qpos, qvel,
        seed, weights, config);
    require(zero.solved && zero.feasible && zero.status == "converged",
        "Equilibrium must converge");
    State downward{};downward.q1=std::acos(-1.0);
    const auto stationary=solve_nmpc_sqp(model.get(),data.get(),downward,qpos,qvel,seed,weights,config);
    require(stationary.final_cost>100 && stationary.feasible,
        "Downward stationary point must not be confused with zero target cost");
    State outside{}; outside.p = 3.95;
    const auto infeasible = solve_nmpc_sqp(model.get(), data.get(), outside,
        qpos, qvel, seed, weights, config);
    require(!infeasible.feasible && !infeasible.solved && infeasible.constraint_violation>0,
        "Track constraint was not enforced");
    auto tiny_budget = config; tiny_budget.max_solve_seconds=1e-12;
    const auto timed_out = solve_nmpc_sqp(model.get(), data.get(), initial,
        qpos, qvel, seed, weights, tiny_budget);
    require(timed_out.status == "timeout" && !timed_out.solved,
        "Timeout was reported as convergence");
    bool rejected=false;
    try { solve_nmpc_sqp(model.get(), data.get(), initial, qpos, qvel,
        std::vector<double>(4,21.0),weights,config); }
    catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"Invalid seed was clipped instead of rejected");
    // Longer nonlinear rollout, including a saturated and a nearly saturated
    // input: local chain derivatives must agree with the slower global check.
    auto nonlinear_config=config;nonlinear_config.steps_per_control=20;
    nonlinear_config.max_iterations=1;nonlinear_config.trust_radius=1;
    const State nonlinear_state{0.1,0.1,1.4,0.1,-0.7,-0.1};
    const std::vector<double> nonlinear_seed{20,19.9999,-3,2,0,1};
    const auto chain=solve_nmpc_sqp(model.get(),data.get(),nonlinear_state,qpos,qvel,nonlinear_seed,weights,nonlinear_config);
    nonlinear_config.use_chain_derivatives=false;
    const auto full=solve_nmpc_sqp(model.get(),data.get(),nonlinear_state,qpos,qvel,nonlinear_seed,weights,nonlinear_config);
    require(chain.feasible==full.feasible && chain.accepted_steps==full.accepted_steps,
        "Nonlinear derivative methods changed feasibility or acceptance");
    require(std::abs(chain.final_cost-full.final_cost)<1e-5*(1+std::abs(full.final_cost)),
        "Nonlinear chain-rule cost mismatch");
    for(std::size_t k=0;k<nonlinear_seed.size();++k)
        require(std::abs(chain.forces[k]-full.forces[k])<1e-3,"Nonlinear chain-rule force mismatch");
    std::cout << "SQP baseline checks passed; cost " << result.initial_cost
              << " -> " << result.final_cost << '\n';
    return 0;
} catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
}
