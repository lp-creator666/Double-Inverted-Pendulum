#pragma once

#include "types.hpp"
#include <string>
#include <vector>
#include "control/lqr.hpp"

// Offline SQP trajectory replay, followed by measured-state LQR capture.
// This is distinct from online receding-horizon NMPC.
struct SwingupPlan {
    std::vector<double> times;
    std::vector<State> states;
    std::vector<double> forces; // Last node has a zero placeholder force.
    double duration() const { return times.empty()?0.0:times.back(); }
    double force_at(double time) const;
};

bool terminal_capture_ready(const State& state);
SwingupPlan load_swingup_plan(const std::string& path,double physical_timestep,
    double lower_force,double upper_force);

// Finite-horizon, time-varying LQR around an offline dynamically feasible plan.
// References and gains use the physical simulation step; gains are prepared once.
// 参考轨迹上的时变 LQR 反馈。
class TrajectoryTracker {
public:
    TrajectoryTracker(const mjModel* model, mjData* scratch, const SwingupPlan& plan,
        const int* qpos, const int* qvel, const LqrResult& terminal_lqr,
        const LqrConfig& weights);
    double compute_force(double time, const State& measured) const;
    const State& reference(std::size_t step) const { return states_.at(step); }
    const VectorB& gain(std::size_t step) const { return gains_.at(step); }
    std::size_t steps() const { return forces_.size(); }
    double setup_seconds() const { return setup_seconds_; }
private:
    double timestep_ = 0;
    double setup_seconds_ = 0;
    std::vector<State> states_;
    std::vector<double> forces_;
    std::vector<VectorB> gains_; // Physical force / physical state.
};
