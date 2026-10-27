#include "optimization/sqp_solver.hpp"
#include "optimization/dense_qp.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
struct Evaluation { Vector residual; Vector constraints; double cost; NmpcPrediction trajectory; };
Evaluation evaluate(const mjModel* model, mjData* data, const State& initial,
    const int* qpos, const int* qvel, const std::vector<double>& forces,
    const NmpcCostConfig& weights, const SqpConfig& config, double previous,
    double track_lower, double track_upper) {
    Evaluation e;
    std::vector<double> physical_forces;
    physical_forces.reserve(forces.size()*config.steps_per_control);
    for(double force:forces) physical_forces.insert(physical_forces.end(),config.steps_per_control,force);
    const auto fine=predict_nonlinear_horizon(model,data,initial,qpos,qvel,physical_forces,1);
    e.trajectory.control_dt=model->opt.timestep*config.steps_per_control;
    for(std::size_t i=0;i<fine.states.size();i+=config.steps_per_control)
        e.trajectory.states.push_back(fine.states[i]);
    e.cost=compute_nmpc_trajectory_cost(e.trajectory,forces,weights,previous);
    const int n=static_cast<int>(forces.size());
    e.residual.resize(6*(n+1)+2*n);
    // State constraints are checked at every physical step, including between shooting nodes.
    e.constraints.resize(8*(n*config.steps_per_control+1));
    int r=0;
    for (int k=0;k<=n;++k) {
        const auto& x=e.trajectory.states[k];
        const double values[]={x.p,x.dp,2*std::sin(x.q1/2),x.dq1,
            2*std::sin((x.q1+x.q2)/2),x.dq1+x.dq2};
        const double multiplier=k==n?weights.terminal_weight:1.0;
        for(int j=0;j<6;++j) e.residual[r++]=std::sqrt(multiplier*weights.state_weights[j])*values[j];
    }
    for(int k=0;k<n;++k) {
        e.residual[r++]=std::sqrt(weights.input_weight)*forces[k];
        e.residual[r++]=std::sqrt(weights.input_change_weight)*(forces[k]-(k?forces[k-1]:previous));
    }
    int c=0;
    auto append=[&](const State& x) {
        e.constraints[c++]=x.p-track_upper; e.constraints[c++]=track_lower-x.p;
        e.constraints[c++]=x.dp-config.max_cart_speed; e.constraints[c++]=-x.dp-config.max_cart_speed;
        e.constraints[c++]=x.dq1-config.max_joint_speed; e.constraints[c++]=-x.dq1-config.max_joint_speed;
        e.constraints[c++]=x.dq2-config.max_joint_speed; e.constraints[c++]=-x.dq2-config.max_joint_speed;
    };
    for(const State& state:fine.states) append(state);
    if (!e.residual.allFinite() || !e.constraints.allFinite()) throw std::runtime_error("Non-finite SQP evaluation");
    return e;
}
double violation(const Evaluation& e) { return std::max(0.0,e.constraints.maxCoeff()); }
Vector state_vector(const State& x) {
    Vector v(6);v<<x.p,x.dp,x.q1,x.dq1,x.q2,x.dq2;return v;
}
State perturb(State x,int index,double amount) {
    double* fields[]={&x.p,&x.dp,&x.q1,&x.dq1,&x.q2,&x.dq2};
    *fields[index]+=amount;return x;
}
}

SqpResult solve_nmpc_sqp(const mjModel* model,mjData* data,const State& initial,
    const int* qpos,const int* qvel,std::vector<double> forces,
    const NmpcCostConfig& weights,const SqpConfig& config,double previous) {
    const auto started=std::chrono::steady_clock::now();
    auto elapsed=[&] { return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count(); };
    if(!model || !data || !qpos || !qvel || !initial.finite() || forces.empty() ||
        model->nu!=1 || !model->actuator_ctrllimited[0] || !std::isfinite(previous))
        throw std::invalid_argument("Invalid SQP input");
    if(config.max_iterations<=0 || config.steps_per_control<=0 || config.max_line_search<=0 ||
        config.qp_max_iterations<=0 || !std::isfinite(config.gradient_epsilon) || config.gradient_epsilon<=0 ||
        !std::isfinite(config.regularization) || config.regularization<=0 ||
        !std::isfinite(config.convergence_tolerance) || config.convergence_tolerance<=0 ||
        !std::isfinite(config.qp_tolerance) || config.qp_tolerance<=0 ||
        !std::isfinite(config.trust_radius) || config.trust_radius<=0 ||
        !std::isfinite(config.minimum_step) || config.minimum_step<=0 || config.minimum_step>1 ||
        !std::isfinite(config.track_margin) || config.track_margin<=0 ||
        !std::isfinite(config.max_cart_speed) || config.max_cart_speed<=0 ||
        !std::isfinite(config.max_joint_speed) || config.max_joint_speed<=0 ||
        !std::isfinite(config.max_solve_seconds) || config.max_solve_seconds<0)
        throw std::invalid_argument("Invalid SQP configuration");
    const int cart=mj_name2id(model,mjOBJ_JOINT,"cart_slide");
    if(cart<0 || !model->jnt_limited[cart]) throw std::invalid_argument("Missing track bounds");
    const double track_lower=model->jnt_range[2*cart]+config.track_margin;
    const double track_upper=model->jnt_range[2*cart+1]-config.track_margin;
    const double lower=model->actuator_ctrlrange[0],upper=model->actuator_ctrlrange[1];
    if(!std::isfinite(lower) || !std::isfinite(upper) || lower>=upper || track_lower>=track_upper)
        throw std::invalid_argument("Invalid SQP bounds");
    for(double f:forces) if(!std::isfinite(f) || f<lower || f>upper)
        throw std::invalid_argument("SQP seed outside actuator bounds");
    auto eval=[&](const std::vector<double>& u) {return evaluate(model,data,initial,qpos,qvel,u,weights,config,previous,track_lower,track_upper);};
    Evaluation current=eval(forces);
    SqpResult result;
    result.initial_cost=current.cost; result.status="iteration_limit";
    const int n=static_cast<int>(forces.size());
    for(int iteration=0;iteration<config.max_iterations;++iteration) {
        if(config.max_solve_seconds>0 && elapsed()>=config.max_solve_seconds) {result.status="timeout";break;}
        Matrix J(current.residual.size(),n), G(current.constraints.size(),n);
        bool timed_out=false;
        J.setZero();G.setZero();
        if(config.use_chain_derivatives) {
            Matrix sensitivity=Matrix::Zero(6,n);
            auto state_residual=[&](int k) {
                const auto& x=current.trajectory.states[k];
                Matrix D=Matrix::Identity(6,6);
                D(2,2)=std::cos(x.q1/2);
                D(4,4)=D(4,2)=std::cos((x.q1+x.q2)/2);
                D(5,3)=1;
                for(int j=0;j<6;++j) D.row(j)*=std::sqrt(weights.state_weights[j]*(k==n?weights.terminal_weight:1));
                J.middleRows(6*k,6)=D*sensitivity;
            };
            state_residual(0);
            const int samples=8*config.steps_per_control;
            for(int k=0;k<n;++k) {
                if(config.max_solve_seconds>0 && elapsed()>=config.max_solve_seconds) {timed_out=true;break;}
                Matrix A(6,6), Dx(samples,6); Vector B(6), Du(samples);
                const State& x=current.trajectory.states[k];
                for(int j=0;j<7;++j) {
                    if(config.max_solve_seconds>0 && elapsed()>=config.max_solve_seconds) {timed_out=true;break;}
                    const double h=j<6?1e-6:config.gradient_epsilon;
                    State xp=x,xm=x;double up=forces[k],um=forces[k];
                    if(j<6) {xp=perturb(x,j,h);xm=perturb(x,j,-h);}
                    else {
                        const double delta=std::min(h,(upper-lower)/2);
                        if(upper-up>=delta) up+=delta;
                        if(um-lower>=delta) um-=delta;
                    }
                    const auto p=evaluate(model,data,xp,qpos,qvel,{up},weights,config,previous,track_lower,track_upper);
                    const auto m=evaluate(model,data,xm,qpos,qvel,{um},weights,config,previous,track_lower,track_upper);
                    const double delta=j<6?2*h:up-um;
                    const Vector next=(state_vector(p.trajectory.states.back())-state_vector(m.trajectory.states.back()))/delta;
                    const Vector constraint=(p.constraints.tail(samples)-m.constraints.tail(samples))/delta;
                    if(j<6) {A.col(j)=next;Dx.col(j)=constraint;}
                    else {B=next;Du=constraint;}
                }
                if(timed_out) break;
                G.middleRows(8+k*samples,samples)=Dx*sensitivity;
                G.block(8+k*samples,k,samples,1)+=Du;
                sensitivity=(A*sensitivity).eval();sensitivity.col(k)+=B;
                state_residual(k+1);
                J(6*(n+1)+2*k,k)=std::sqrt(weights.input_weight);
                J(6*(n+1)+2*k+1,k)=std::sqrt(weights.input_change_weight);
                if(k) J(6*(n+1)+2*k+1,k-1)=-std::sqrt(weights.input_change_weight);
            }
        } else for(int k=0;k<n;++k) {
            if(config.max_solve_seconds>0 && elapsed()>=config.max_solve_seconds) {timed_out=true;break;}
            auto plus=forces,minus=forces;
            // Equal spacing in the interior, inward one-sided difference at bounds.
            const double h=std::min({config.gradient_epsilon,(upper-lower)/2});
            if(upper-forces[k]>=h) plus[k]+=h;
            if(forces[k]-lower>=h) minus[k]-=h;
            const Evaluation p=eval(plus),m=eval(minus);
            J.col(k)=(p.residual-m.residual)/(plus[k]-minus[k]);
            G.col(k)=(p.constraints-m.constraints)/(plus[k]-minus[k]);
        }
        if(timed_out) {result.status="timeout";break;}
        const Matrix H=2*J.transpose()*J+config.regularization*Matrix::Identity(n,n);
        const Vector gradient=2*J.transpose()*current.residual;
        Matrix C(G.rows()+2*n,n); Vector b(C.rows());
        C.topRows(G.rows())=G; b.head(G.rows())=-current.constraints;
        C.middleRows(G.rows(),n)=Matrix::Identity(n,n);
        C.bottomRows(n)=-Matrix::Identity(n,n);
        for(int k=0;k<n;++k) {
            b[G.rows()+k]=std::min(config.trust_radius,upper-forces[k]);
            b[G.rows()+n+k]=std::min(config.trust_radius,forces[k]-lower);
        }
        if(config.max_solve_seconds>0 && elapsed()>=config.max_solve_seconds) {result.status="timeout";break;}
        const double qp_budget=config.max_solve_seconds>0?std::max(1e-12,config.max_solve_seconds-elapsed()):0;
        const auto qp=solve_dense_qp(H,gradient,C,b,config.qp_max_iterations,config.qp_tolerance,qp_budget);
        result.iterations=iteration+1;
        result.qp_iterations=qp.iterations;
        result.qp_primal_residual=qp.primal_residual;
        result.qp_stationarity_residual=qp.stationarity_residual;
        result.qp_complementarity_residual=qp.complementarity_residual;
        if(!qp.solved) {result.status=qp.timed_out?"timeout":"qp_not_converged";break;}
        result.step_norm=qp.step.lpNorm<Eigen::Infinity>();
        if(result.step_norm<=config.convergence_tolerance && violation(current)<=config.qp_tolerance) {
            result.solved=true;result.status="converged";break;
        }
        bool accepted=false;
        double alpha=1;
        const double penalty=std::max(1e4,100*current.cost);
        const double merit=current.cost+penalty*current.constraints.cwiseMax(0).sum();
        for(int line=0;line<config.max_line_search && alpha>=config.minimum_step;++line,alpha*=0.5) {
            if(config.max_solve_seconds>0 && elapsed()>=config.max_solve_seconds) {result.status="timeout";break;}
            auto trial=forces;
            for(int k=0;k<n;++k) trial[k]+=alpha*qp.step[k];
            // Only roundoff at a QP bound is corrected here; no unconstrained force clipping.
            bool input_ok=true;
            for(double& u:trial) {
                if(u<lower-1e-6 || u>upper+1e-6) {input_ok=false;break;}
                u=std::clamp(u,lower,upper);
            }
            if(!input_ok) continue;
            Evaluation next=eval(trial); ++result.line_search_steps;
            const double next_merit=next.cost+penalty*next.constraints.cwiseMax(0).sum();
            const bool preserve_feasible=violation(current)>config.qp_tolerance || violation(next)<=config.qp_tolerance;
            if(preserve_feasible && next_merit<merit-1e-12) {
                forces=std::move(trial);current=std::move(next);accepted=true;++result.accepted_steps;break;
            }
        }
        if(!accepted) {if(result.status!="timeout") result.status="line_search_failed";break;}
    }
    result.forces=std::move(forces);result.final_cost=current.cost;
    result.constraint_violation=violation(current);
    result.feasible=result.constraint_violation<=config.qp_tolerance;
    result.solve_seconds=elapsed();
    return result;
}
