#include "control/online_sqp_controller.hpp"
#include "control/supervisor.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>

OnlineSqpConfig::OnlineSqpConfig() {
    solver.max_iterations=1;solver.steps_per_control=40;solver.trust_radius=0.5;
    solver.regularization=1;solver.qp_max_iterations=300;solver.qp_tolerance=1e-5;
    solver.max_line_search=12;
    cost.state_weights={1,1,10,1,10,1};cost.terminal_weight=10000;
    cost.input_weight=0.001;cost.input_change_weight=0.001;
}

OnlineSqpController::OnlineSqpController(const mjModel* model,const SwingupPlan& plan,
    const int* qpos,const int* qvel,const LqrResult& lqr,const LqrConfig& lqr_config,
    const std::string& diagnostics,OnlineSqpConfig config)
    : model_(model),scratch_(model?mj_makeData(model):nullptr,mj_deleteData),plan_(plan),
      lqr_(lqr),lqr_config_(lqr_config),config_(config) {
    if(!model_ || !scratch_ || !qpos || !qvel || !std::isfinite(config_.solve_budget) || config_.solve_budget<=0)
        throw std::invalid_argument("Invalid online SQP setup");
    for(int i=0;i<3;++i){qpos_[i]=qpos[i];qvel_[i]=qvel[i];}
    tracker_=std::make_unique<TrajectoryTracker>(model_,scratch_.get(),plan_,qpos_.data(),qvel_.data(),lqr_,lqr_config_);
    const double dt=model_->opt.timestep*config_.solver.steps_per_control;
    for(std::size_t i=0;i+1<plan_.times.size();++i)
        if(std::abs(plan_.times[i+1]-plan_.times[i]-dt)>1e-8)
            throw std::invalid_argument("Online SQP requires uniform plan segments matching solver period");
    warm_=std::vector<double>(plan_.forces.begin(),plan_.forces.end()-1);
    const std::filesystem::path path(diagnostics);
    if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    log_.open(path);
    if(!log_) throw std::runtime_error("Cannot open SQP diagnostics");
    log_<<std::setprecision(17)<<"time,horizon,status,feasible,accepted_steps,iterations,qp_iterations,initial_cost,final_cost,constraint_violation,solve_seconds,budget_seconds,control_period,deadline_missed,budget_exceeded,selected,force\n";
}

double OnlineSqpController::compute_force(double time,const State& measured) {
    if(!std::isfinite(time) || time<0 || !measured.finite()) throw std::invalid_argument("Invalid online SQP state/time");
    if(time+1e-10>=plan_.duration()) {
        if(!balancing_) {
            if(!terminal_capture_ready(measured)) throw std::runtime_error("Online SQP missed terminal capture region");
            balancing_=true;std::cout<<"Online SQP -> LQR at t="<<time<<" s\n";
        }
        return compute_lqr_force(local_balance_state(measured),lqr_,lqr_config_);
    }
    const auto step=static_cast<std::size_t>(std::llround(time/model_->opt.timestep));
    if(std::abs(time-step*model_->opt.timestep)>1e-8 || step>=tracker_->steps())
        throw std::invalid_argument("Online control time is not a physical step");
    if(step%config_.solver.steps_per_control==0) {
        if(node_>0 && !warm_.empty()) warm_.erase(warm_.begin());
        ++node_;
        if(warm_.empty()) throw std::runtime_error("Online SQP warm start exhausted");
        auto settings=config_.solver;settings.max_solve_seconds=config_.solve_budget;
        const auto result=solve_nmpc_sqp(model_,scratch_.get(),measured,qpos_.data(),qvel_.data(),warm_,config_.cost,settings,previous_force_);
        const double control_period=config_.solver.steps_per_control*model_->opt.timestep;
        const bool missed=result.solve_seconds>control_period;
        const bool exceeded=result.solve_seconds>config_.solve_budget;
        const bool usable=result.feasible && (result.solved || result.accepted_steps>0) && !exceeded;
        // If the QP fails, a newly checked feasible shifted sequence is a valid
        // backup. Retain its local reference instead of jumping to the original
        // offline trajectory, which may now be far from the optimized motion.
        reusing_=!usable && !exceeded && result.status!="timeout" && result.feasible;
        const bool use_sequence=usable || reusing_;
        fallback_=!use_sequence;
        if(use_sequence) {
            warm_=result.forces;selected_force_=warm_.front();reference_start_=step;
            const std::vector<double> controls(config_.solver.steps_per_control,selected_force_);
            local_reference_=predict_nonlinear_horizon(model_,scratch_.get(),measured,qpos_.data(),qvel_.data(),controls,1).states;
        }
        log_<<time<<','<<warm_.size()<<','<<result.status<<','<<result.feasible<<','<<result.accepted_steps<<','<<result.iterations<<','<<result.qp_iterations<<','
            <<result.initial_cost<<','<<result.final_cost<<','<<result.constraint_violation<<','<<result.solve_seconds<<','<<config_.solve_budget<<','<<control_period<<','<<missed<<','<<exceeded<<','
            <<(usable?"sqp":reusing_?"feasible_warm_start":"trajectory_feedback")<<','<<(use_sequence?selected_force_:tracker_->compute_force(time,measured))<<'\n';
        log_.flush();if(!log_) throw std::runtime_error("SQP diagnostics write failed");
    }
    double force;
    if(fallback_) force=tracker_->compute_force(time,measured);
    else {
        const auto& reference=local_reference_.at(step-reference_start_);
        const double period=2*std::acos(-1.0);
        const State error{measured.p-reference.p,measured.dp-reference.dp,
            std::remainder(measured.q1-reference.q1,period),measured.dq1-reference.dq1,
            std::remainder(measured.q2-reference.q2,period),measured.dq2-reference.dq2};
        const double values[]={error.p,error.dp,error.q1,error.dq1,error.q2,error.dq2};
        force=selected_force_;
        for(int j=0;j<6;++j) force-=tracker_->gain(step)[j]*values[j];
    }
    if(!std::isfinite(force)) throw std::runtime_error("Non-finite online SQP force");
    previous_force_=std::clamp(force,model_->actuator_ctrlrange[0],model_->actuator_ctrlrange[1]);
    return force;
}
