#include "control/trajectory.hpp"
#include "model/linearize.hpp"
#include <memory>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
template<class F> void reject(F function) {
    bool rejected=false;try{function();}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Invalid tracking input accepted");
}
int main() try {
    char error[1024]{};
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML("model/double_cartpole.xml",nullptr,error,sizeof(error)),mj_deleteModel);
    require(bool(model),error);
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
    require(bool(data),"Data allocation failed");
    int qp[3],qv[3];const char* names[]={"cart_slide","pole_1_hinge","pole_2_hinge"};
    for(int i=0;i<3;++i){const int id=mj_name2id(model.get(),mjOBJ_JOINT,names[i]);require(id>=0,"Missing joint");qp[i]=model->jnt_qposadr[id];qv[i]=model->jnt_dofadr[id];}
    const auto linear=linearize_discrete(model.get(),data.get(),{},0,qp,qv,1,1e-6,1e-5);
    const LqrConfig weights;
    const auto terminal=solve_discrete_lqr(linear.A,linear.B,weights);
    SwingupPlan equilibrium;equilibrium.times={0,0.02};equilibrium.states={{},{}};equilibrium.forces={0,0};
    const TrajectoryTracker tracker(model.get(),data.get(),equilibrium,qp,qv,terminal,weights);
    require(tracker.steps()==10,"Wrong physical-step tracking horizon");
    const State error_state{0.001,0.002,0.003,0.004,-0.002,-0.001};
    const double expected=compute_lqr_force(error_state,terminal,weights);
    for(std::size_t k=0;k<tracker.steps();++k) {
        require(tracker.compute_force(k*model->opt.timestep,tracker.reference(k))==0,"Reference feedback must be zero");
        require(std::abs(tracker.compute_force(k*model->opt.timestep,error_state)-expected)<1e-7,
            "At equilibrium, finite-horizon gains must recover the existing LQR physical force");
    }
    State equivalent{};equivalent.q1=2*std::acos(-1.0);equivalent.q2=-2*std::acos(-1.0);
    require(std::abs(tracker.compute_force(0,equivalent))<1e-10,"Tracking angle error is not periodic");
    reject([&]{tracker.compute_force(0.001,{});});
    reject([&]{tracker.compute_force(0.02,{});});
    State invalid{};invalid.dp=std::numeric_limits<double>::quiet_NaN();
    reject([&]{tracker.compute_force(0,invalid);});
    equilibrium.states.back().q1=0.01;
    reject([&]{TrajectoryTracker bad(model.get(),data.get(),equilibrium,qp,qv,terminal,weights);});
    const auto plan=load_swingup_plan("config/plans/sqp_swingup.csv",model->opt.timestep,-20,20);
    const TrajectoryTracker moving(model.get(),data.get(),plan,qp,qv,terminal,weights);
    require(moving.steps()==2000,"Dense swing-up reference was not generated");
    for(std::size_t k=0;k<moving.steps();++k) {
        const double t=k*model->opt.timestep;
        require(std::abs(moving.compute_force(t,moving.reference(k))-plan.force_at(t))<1e-10,
            "Nominal tracking changed the feedforward plan");
    }
    std::cout<<"Tracking equilibrium equivalence, units, periodic errors, timing and dynamic consistency passed\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
