#pragma once

#include "optimization/sqp_solver.hpp"
#include "control/trajectory.hpp"
#include <array>
#include <fstream>
#include <memory>

struct OnlineSqpConfig {
    SqpConfig solver;
    NmpcCostConfig cost;
    double solve_budget = 0.5; // Soft computation budget; simulation may run slower than real time.
    OnlineSqpConfig();
};

// Seeded shrinking-horizon NMPC: optimize the remaining swing-up at each node.
// A simulation may run slower than wall-clock; missed deadlines are reported.
class OnlineSqpController {
public:
    OnlineSqpController(const mjModel* model,const SwingupPlan& plan,
        const int* qpos,const int* qvel,const LqrResult& lqr,const LqrConfig& lqr_config,
        const std::string& diagnostics,OnlineSqpConfig config = {});
    double compute_force(double time,const State& measured);
    int stage_code() const { return balancing_?3:((fallback_ || reusing_)?4:2); }
private:
    const mjModel* model_;
    std::unique_ptr<mjData,decltype(&mj_deleteData)> scratch_;
    SwingupPlan plan_;
    std::array<int,3> qpos_,qvel_;
    LqrResult lqr_;
    LqrConfig lqr_config_;
    OnlineSqpConfig config_;
    std::unique_ptr<TrajectoryTracker> tracker_;
    std::vector<double> warm_;
    std::vector<State> local_reference_;
    std::ofstream log_;
    std::size_t node_=0,reference_start_=0;
    double selected_force_=0,previous_force_=0;
    bool balancing_=false,fallback_=false,reusing_=false;
};
