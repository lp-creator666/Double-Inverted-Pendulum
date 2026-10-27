#include "control/lmpc.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
static void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){
 try {
    MatrixA A{}; VectorB B{};
    for(int i=0;i<kStateDimension;++i) A[i*kStateDimension+i]=0.8;
    A[0]=1.0; A[1]=0.1; A[kStateDimension+1]=1.0; B[0]=0.005; B[1]=0.1;
    LqrConfig weights; LmpcConfig config; const auto lqr=solve_discrete_lqr(A,B,weights);
    const auto zero=compute_lmpc_control(A,B,State{},lqr,config,weights);
    require(zero.solved,"Zero-state LMPC did not solve");
    require(std::abs(zero.requested_force)<1e-10,"Zero-state force is not zero");
    State perturbed{}; perturbed.p=0.2; perturbed.q1=0.02;
    const auto result=compute_lmpc_control(A,B,perturbed,lqr,config,weights);
    require(result.solved,"Perturbed LMPC did not solve");
    require(std::isfinite(result.requested_force),"LMPC force is non-finite");
    require(result.first_scaled_input>=config.input_min-1e-10 &&
            result.first_scaled_input<=config.input_max+1e-10,"Input bound violated");
    auto mismatched=config; mismatched.scaling.input_scale=10.0;
    bool rejected=false; try{compute_lmpc_control(A,B,State{},lqr,mismatched,weights);}
    catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Mismatched scales accepted");
    std::cout<<"LMPC controller checks passed\n";
 } catch(const std::exception& e) { std::cerr << e.what() << "\n"; return 1; }
}
