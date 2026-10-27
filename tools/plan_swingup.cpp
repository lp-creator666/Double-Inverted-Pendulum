#include "optimization/sqp_solver.hpp"
#include "model/linearize.hpp"
#include "control/lqr.hpp"
#include "control/supervisor.hpp"
#include "control/trajectory.hpp"
#include "model/dynamics.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <iomanip>
#include <filesystem>
#include <sstream>
#include <algorithm>

// Independently replay the candidate with continuous mjData, then use actual
// measured state for LQR. Passing the endpoint gate alone is not acceptance.
bool verify_capture(const mjModel* model,mjData* data,const State& initial,
    const int* qp,const int* qv,const std::vector<double>& forces,int hold) {
    const auto linear=linearize_discrete(model,data,{},0,qp,qv,1,1e-6,1e-6);
    const LqrConfig weights;
    const auto lqr=solve_discrete_lqr(linear.A,linear.B,weights);
    mj_resetData(model,data);
    const double positions[]={initial.p,initial.q1,initial.q2};
    const double velocities[]={initial.dp,initial.dq1,initial.dq2};
    for(int i=0;i<3;++i){data->qpos[qp[i]]=positions[i];data->qvel[qv[i]]=velocities[i];}
    mj_forward(model,data);
    const int feedforward_steps=static_cast<int>(forces.size())*hold;
    const int balance_steps=static_cast<int>(std::ceil(6/model->opt.timestep));
    const int cart=mj_name2id(model,mjOBJ_JOINT,"cart_slide");
    std::ofstream log("logs/swingup_capture.csv");
    log<<std::setprecision(17)<<"time,p,dp,q1,dq1,q2,dq2,requested_force,applied_force,stage\n";
    bool settled=true;
    for(int k=0;k<feedforward_steps+balance_steps;++k) {
        const State x=read_state(data,qp,qv);
        const bool balance=k>=feedforward_steps;
        const double request=balance?compute_lqr_force(local_balance_state(x),lqr,weights):forces[k/hold];
        const double applied=std::clamp(request,model->actuator_ctrlrange[0],model->actuator_ctrlrange[1]);
        data->ctrl[0]=applied; mj_step(model,data);
        const State y=read_state(data,qp,qv);
        log<<data->time<<','<<y.p<<','<<y.dp<<','<<y.q1<<','<<y.dq1<<','<<y.q2<<','<<y.dq2<<','<<request<<','<<applied<<','<<(balance?"LQR":"plan")<<'\n';
        if(!y.finite() || y.p<=model->jnt_range[2*cart]+0.15 || y.p>=model->jnt_range[2*cart+1]-0.15 ||
            std::abs(y.dp)>8 || std::abs(y.dq1)>30 || std::abs(y.dq2)>30) return false;
        for(int warning=0;warning<mjNWARNING;++warning) if(data->warning[warning].number) return false;
        if(k>=feedforward_steps+balance_steps-static_cast<int>(1/model->opt.timestep)) {
            const auto z=local_balance_state(y);
            settled=settled && std::abs(z.p)<0.05 && std::abs(z.dp)<0.05 &&
                std::abs(z.q1)<0.02 && std::abs(z.q1+z.q2)<0.02 &&
                std::abs(z.dq1)<0.05 && std::abs(z.dq1+z.dq2)<0.05;
        }
    }
    return settled && log.good();
}
int main(int argc,char** argv) try {
    if(argc>1 && std::string(argv[1])=="--help") {
        std::cout<<"plan_swingup [q1_rad=pi] [seed_csv=config/seeds/sqp_swingup_seed.csv] [iterations=20] [q2_rad=0] [dq1=0] [dq2=0]\n"
            <<"Use '-' for seed_csv to explore zero/sine seeds. Output: logs/swingup_verified.csv (only after stable replay).\n";
        return 0;
    }
    if(argc>7) throw std::invalid_argument("Too many planner arguments; use --help");
    auto number=[](const char* arg) {
        std::size_t end;const double v=std::stod(arg,&end);
        if(end!=std::string(arg).size() || !std::isfinite(v)) throw std::invalid_argument("Invalid planner number");
        return v;
    };
    char message[1024]{};
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML("model/double_cartpole.xml",nullptr,message,sizeof(message)),mj_deleteModel);
    if(!model) throw std::runtime_error(message);
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
    if(!data) throw std::runtime_error("Failed to allocate prediction data");
    int qp[3],qv[3];const char* names[]={"cart_slide","pole_1_hinge","pole_2_hinge"};
    for(int i=0;i<3;++i){int id=mj_name2id(model.get(),mjOBJ_JOINT,names[i]);if(id<0) throw std::runtime_error("Missing joint");qp[i]=model->jnt_qposadr[id];qv[i]=model->jnt_dofadr[id];}
    State initial{};initial.q1=argc>1?number(argv[1]):std::acos(-1.0);
    initial.q2=argc>4?number(argv[4]):0;
    initial.dq1=argc>5?number(argv[5]):0;
    initial.dq2=argc>6?number(argv[6]):0;
    const double iterations=argc>3?number(argv[3]):20;
    if(iterations<1 || iterations>1000 || iterations!=std::round(iterations)) throw std::invalid_argument("Iterations must be an integer in [1,1000]");
    SqpConfig config;config.max_iterations=static_cast<int>(iterations);config.steps_per_control=40;config.trust_radius=1;config.regularization=1;
    config.qp_max_iterations=300;config.qp_tolerance=1e-5;config.max_line_search=12;
    NmpcCostConfig weights;weights.state_weights={1,1,10,1,10,1};
    weights.terminal_weight=10000;weights.input_weight=0.001;weights.input_change_weight=0.001;
    const int n=50;
    std::filesystem::create_directories("logs");
    std::vector<double> warm;
    const std::string seed_path=argc>2?argv[2]:"config/seeds/sqp_swingup_seed.csv";
    if(seed_path!="-") {
        std::ifstream input(seed_path); std::string line;
        if(!input || !std::getline(input,line) || line!="force") throw std::runtime_error("Invalid warm-start CSV header");
        while(std::getline(input,line)) { std::istringstream row(line); double value; char extra;
            if(!(row>>value) || row>>extra || !std::isfinite(value)) throw std::runtime_error("Invalid warm-start force");
            warm.push_back(value);
        }
        if(warm.size()!=n) throw std::runtime_error("Warm start requires 50 forces at 0.08 s");
    }
    std::filesystem::remove("logs/swingup_verified.csv");
    std::filesystem::remove("logs/swingup_capture.csv");
    double best=1e100;
    bool success=false;
    for(int trial=0;trial<(warm.empty()?3:1);++trial){
        std::vector<double> u(n);
        for(int k=0;k<n;++k) u[k]=trial==0?0: (trial%2?1:-1)*0.5*std::sin((trial/2+1)*2*std::acos(-1.0)*k/n);
        if(!warm.empty()) u=warm;
        const auto r=solve_nmpc_sqp(model.get(),data.get(),initial,qp,qv,u,weights,config);
        std::cout<<"iterations="<<r.iterations<<" accepted="<<r.accepted_steps<<" qp_iterations="<<r.qp_iterations
            <<" qp_residuals="<<r.qp_primal_residual<<','<<r.qp_stationarity_residual<<','<<r.qp_complementarity_residual<<std::endl;
        const auto trajectory=predict_nonlinear_horizon(model.get(),data.get(),initial,qp,qv,r.forces,config.steps_per_control);
        const auto& x=trajectory.states.back();
        std::cout<<"seed="<<trial<<" status="<<r.status<<" cost="<<r.initial_cost<<" -> "<<r.final_cost<<" feasible="<<r.feasible<<" end="<<x.p<<","<<x.dp<<","<<x.q1<<","<<x.dq1<<","<<x.q2<<","<<x.dq2<<" sec="<<r.solve_seconds<<std::endl;
        const bool captured=r.feasible && terminal_capture_ready(x);
        std::cout<<"terminal_capture="<<captured<<" max_constraint_violation="<<r.constraint_violation<<std::endl;
        if(captured) {
            const bool stable=verify_capture(model.get(),data.get(),initial,qp,qv,r.forces,config.steps_per_control);
            std::cout<<"continuous_replay_and_lqr_stable="<<stable<<std::endl;
            success=success||stable;
            if(stable) {
                std::ofstream output("logs/swingup_verified.csv");
                output<<std::setprecision(17)<<"time,p,dp,q1,dq1,q2,dq2,force\n";
                for(std::size_t k=0;k<trajectory.states.size();++k) {
                    const auto& s=trajectory.states[k];
                    output<<k*config.steps_per_control*model->opt.timestep<<','<<s.p<<','<<s.dp<<','<<s.q1<<','<<s.dq1<<','<<s.q2<<','<<s.dq2<<','<<(k<r.forces.size()?r.forces[k]:0)<<'\n';
                }
                if(!output.good()) throw std::runtime_error("Failed to save verified trajectory");
            }
        }
        if(r.feasible && r.final_cost<best){best=r.final_cost;std::ofstream output("logs/swingup_candidate.csv");output<<std::setprecision(17)<<"force\n";for(double f:r.forces)output<<f<<'\n';
            std::ofstream states("logs/swingup_candidate_states.csv");
            states<<std::setprecision(17)<<"time,p,dp,q1,dq1,q2,dq2\n";
            for(std::size_t k=0;k<trajectory.states.size();++k) {const auto& s=trajectory.states[k];states<<k*config.steps_per_control*model->opt.timestep<<','<<s.p<<','<<s.dp<<','<<s.q1<<','<<s.dq1<<','<<s.q2<<','<<s.dq2<<'\n';}
        }
    }
    return success?0:2; // A finite, feasible downward trajectory is not a swing-up success.
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
