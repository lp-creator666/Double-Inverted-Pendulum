#include "control/trajectory.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "model/dynamics.hpp"
#include "optimization/nonlinear_model.hpp"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <chrono>

bool terminal_capture_ready(const State& x) {
    if(!x.finite() || !std::isfinite(x.q1+x.q2) || !std::isfinite(x.dq1+x.dq2)) return false;
    const double period=2*std::acos(-1.0);
    return std::abs(x.p)<0.3 && std::abs(x.dp)<0.2 &&
        std::abs(std::remainder(x.q1,period))<0.05 &&
        std::abs(std::remainder(x.q1+x.q2,period))<0.05 &&
        std::abs(x.dq1)<0.2 && std::abs(x.dq1+x.dq2)<0.2;
}

SwingupPlan load_swingup_plan(const std::string& path,double dt,double lower,double upper) {
    if(!std::isfinite(dt) || dt<=0 || !std::isfinite(lower) || !std::isfinite(upper) || lower>=upper)
        throw std::invalid_argument("Invalid plan model limits");
    std::ifstream file(path); std::string line;
    if(!file || !std::getline(file,line)) throw std::runtime_error("Cannot read swing-up plan: "+path);
    if(!line.empty() && line.back()=='\r') line.pop_back();
    if(line!="time,p,dp,q1,dq1,q2,dq2,force") throw std::invalid_argument("Invalid plan header");
    SwingupPlan plan;
    while(std::getline(file,line)) {
        if(!line.empty() && line.back()=='\r') line.pop_back();
        std::istringstream row(line); double v[8]{};
        for(int i=0;i<8;++i) {
            if(!(row>>v[i]) || !std::isfinite(v[i])) throw std::invalid_argument("Invalid plan number");
            if(i<7) {char comma; if(!(row>>comma) || comma!=',') throw std::invalid_argument("Invalid plan columns");}
        }
        char extra; if(row>>extra) throw std::invalid_argument("Extra plan column");
        if(v[7]<lower || v[7]>upper) throw std::invalid_argument("Plan force outside model bounds");
        if(plan.times.empty()) {
            if(v[0]!=0) throw std::invalid_argument("Plan must start at zero");
        } else {
            const double steps=(v[0]-plan.times.back())/dt;
            if(!std::isfinite(steps) || steps<1-1e-8 || std::abs(steps-std::round(steps))>1e-8)
                throw std::invalid_argument("Plan intervals must be positive integral physical steps");
        }
        plan.times.push_back(v[0]);
        plan.states.push_back({v[1],v[2],v[3],v[4],v[5],v[6]});
        plan.forces.push_back(v[7]);
    }
    if(plan.times.size()<2 || plan.forces.back()!=0 || !terminal_capture_ready(plan.states.back()))
        throw std::invalid_argument("Plan has no valid terminal capture state");
    return plan;
}

double SwingupPlan::force_at(double time) const {
    if(!std::isfinite(time) || time<0 || times.size()<2 || times.size()!=forces.size() || time>=duration())
        throw std::invalid_argument("Time outside feedforward plan");
    // Time is aligned to physical steps; tolerate floating accumulation at nodes.
    const auto next=std::upper_bound(times.begin(),times.end(),time+1e-10);
    const std::size_t index=static_cast<std::size_t>(next-times.begin()-1);
    return forces[std::min(index,forces.size()-2)];
}

namespace {
using Matrix6 = Eigen::Matrix<double,6,6>;
using RowMatrix6 = Eigen::Matrix<double,6,6,Eigen::RowMajor>;
using Vector6 = Eigen::Matrix<double,6,1>;
StateVector values(const State& x) { return {x.p,x.dp,x.q1,x.dq1,x.q2,x.dq2}; }
}

TrajectoryTracker::TrajectoryTracker(const mjModel* model,mjData* scratch,
    const SwingupPlan& plan,const int* qpos,const int* qvel,
    const LqrResult& terminal,const LqrConfig& weights) {
    const auto started=std::chrono::steady_clock::now();
    if(!model || !scratch || !qpos || !qvel || model->nu!=1 ||
        !model->actuator_ctrllimited[0] || !std::isfinite(model->opt.timestep) || model->opt.timestep<=0 ||
        plan.times.size()<2 || plan.states.size()!=plan.times.size() || plan.forces.size()!=plan.times.size() ||
        plan.times.front()!=0 || !std::isfinite(weights.r) || weights.r<=0)
        throw std::invalid_argument("Invalid trajectory tracker input");
    timestep_=model->opt.timestep;
    for(int i=0;i<3;++i) if(qpos[i]<0 || qpos[i]>=model->nq || qvel[i]<0 || qvel[i]>=model->nv)
        throw std::invalid_argument("Tracker joint address out of range");
    states_.push_back(plan.states.front());
    for(std::size_t node=0;node+1<plan.times.size();++node) {
        const double count=(plan.times[node+1]-plan.times[node])/timestep_;
        if(!std::isfinite(count) || count<1-1e-8 || count>100000 || std::abs(count-std::round(count))>1e-8 ||
            !std::isfinite(plan.forces[node]) || plan.forces[node]<model->actuator_ctrlrange[0] ||
            plan.forces[node]>model->actuator_ctrlrange[1])
            throw std::invalid_argument("Invalid tracker interval or force");
        for(int j=0;j<static_cast<int>(std::llround(count));++j) {
            forces_.push_back(plan.forces[node]);
            states_.push_back(predict_state(model,scratch,states_.back(),qpos,qvel,plan.forces[node],1));
        }
        const auto actual=values(states_.back()), recorded=values(plan.states[node+1]);
        for(int i=0;i<6;++i) if(!std::isfinite(recorded[i]) || std::abs(actual[i]-recorded[i])>1e-5)
            throw std::invalid_argument("Plan state is inconsistent with current model; regenerate the SQP trajectory");
    }
    if(!terminal_capture_ready(states_.back())) throw std::invalid_argument("Tracker reference misses capture region");
    Matrix6 Q=Matrix6::Zero();
    for(int i=0;i<6;++i) {
        if(!std::isfinite(weights.q_diagonal[i]) || weights.q_diagonal[i]<=0)
            throw std::invalid_argument("Invalid tracking state weight");
        Q(i,i)=weights.q_diagonal[i];
    }
    Matrix6 P=Eigen::Map<const RowMatrix6>(terminal.P.data());
    if(!P.allFinite() || (P-P.transpose()).cwiseAbs().maxCoeff()>1e-8 ||
        Eigen::LLT<Matrix6>(P).info()!=Eigen::Success)
        throw std::invalid_argument("Invalid tracker terminal cost");
    const auto linear=linearize_nmpc_trajectory(model,scratch,states_,forces_,qpos,qvel,1);
    gains_.resize(forces_.size());
    for(std::size_t k=forces_.size();k-->0;) {
        const auto scaled=scale_linearization(linear.segments[k].A,linear.segments[k].B,weights.scaling);
        const Matrix6 A=Eigen::Map<const RowMatrix6>(scaled.A.data());
        const Vector6 B=Eigen::Map<const Vector6>(scaled.B.data());
        const double denominator=weights.r+(B.transpose()*P*B)(0,0);
        if(!std::isfinite(denominator) || denominator<=0) throw std::runtime_error("Invalid tracking Riccati denominator");
        const Eigen::Matrix<double,1,6> K=(B.transpose()*P*A)/denominator;
        // This sum-of-squares form avoids subtracting nearly equal PSD matrices.
        const Matrix6 closed=A-B*K;
        P=Q+closed.transpose()*P*closed+weights.r*K.transpose()*K;
        P=(0.5*(P+P.transpose())).eval();
        if(!P.allFinite() || !K.allFinite()) throw std::runtime_error("Tracking Riccati recursion diverged");
        for(int i=0;i<6;++i) gains_[k][i]=weights.scaling.input_scale*K[i]/weights.scaling.state_scale[i];
    }
    setup_seconds_=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
}

double TrajectoryTracker::compute_force(double time,const State& measured) const {
    if(!std::isfinite(time) || time<0 || time>=forces_.size()*timestep_ || !measured.finite())
        throw std::invalid_argument("Invalid tracking time or measured state");
    const auto k=static_cast<std::size_t>(std::llround(time/timestep_));
    if(k>=forces_.size() || std::abs(time-k*timestep_)>1e-8)
        throw std::invalid_argument("Tracking time must align with physical steps");
    auto error=values(measured); const auto desired=values(states_[k]);
    for(int i=0;i<6;++i) error[i]-=desired[i];
    const double period=2*std::acos(-1.0);
    error[2]=std::remainder(error[2],period);error[4]=std::remainder(error[4],period);
    // Reject large departures: local TVLQR is not a recovery controller.
    const double limit[]={0.75,2.0,0.35,3.0,0.5,4.0};
    for(int i=0;i<6;++i) if(!std::isfinite(error[i]) || std::abs(error[i])>limit[i])
        throw std::runtime_error("State left trajectory feedback validity guard");
    double force=forces_[k];
    for(int i=0;i<6;++i) force-=gains_[k][i]*error[i];
    if(!std::isfinite(force)) throw std::runtime_error("Non-finite tracking force");
    return force; // Actuator saturation and state safety remain in the simulation loop.
}
