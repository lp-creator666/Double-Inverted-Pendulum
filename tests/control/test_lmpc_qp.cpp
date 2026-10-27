#include "control/lmpc.hpp"
#include "control/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Cholesky>
namespace {
void require(bool ok,const char* message) {
    if(!ok) throw std::runtime_error(message);
}
bool close(double a,double b) {
    return std::abs(a-b)<=1e-9*std::max({1.0,std::abs(a),std::abs(b)});
}
}
int main() {
    constexpr int nx=kStateDimension,N=LmpcConfig::horizon;
    using M6=Eigen::Matrix<double,nx,nx>;
    using V6=Eigen::Matrix<double,nx,1>;
    using MN=Eigen::Matrix<double,N,N>;
    using VN=Eigen::Matrix<double,N,1>;
    using R6=Eigen::Matrix<double,nx,nx,Eigen::RowMajor>;
    using RN=Eigen::Matrix<double,N,N,Eigen::RowMajor>;
    MatrixA A{}; VectorB B{};
    for(int i=0;i<nx;++i) A[i*nx+i]=0.8;
    A[0]=1; A[1]=0.1; A[nx+1]=1;
    B[0]=0.005; B[1]=0.1;
    LqrConfig weights;
    weights.scaling=Scaling{{2,3,0.5,2,0.7,1.5},4};
    LmpcConfig config; config.scaling=weights.scaling;
    const auto lqr=solve_discrete_lqr(A,B,weights);
    const auto scaled=scale_linearization(A,B,weights.scaling);
    const M6 As=Eigen::Map<const R6>(scaled.A.data());
    const V6 Bs=Eigen::Map<const V6>(scaled.B.data());
    const M6 P=Eigen::Map<const R6>(lqr.P.data());
    M6 Q=M6::Zero();
    for(int i=0;i<nx;++i) Q(i,i)=weights.q_diagonal[i];
    double worst_cost=0,worst_input=0;
    for(const StateVector z0 : {StateVector{0.2,-0.1,0.03,0,0,0},
                               StateVector{-0.1,0.04,0,0.02,-0.01,0.03},StateVector{}}) {
        const auto qp=build_lmpc_qp(scaled.A,scaled.B,z0,config,weights,lqr.P);
        const MN H=Eigen::Map<const RN>(qp.H.data());
        const VN g=Eigen::Map<const VN>(qp.g.data());
        require((H-H.transpose()).cwiseAbs().maxCoeff()<1e-12,"H symmetry");
        Eigen::LLT<MN> factor(H);
        require(factor.info()==Eigen::Success,"H positive definiteness");
        for(int experiment=0;experiment<4;++experiment) {
            VN V=VN::Zero();
            if(experiment==1) for(int k=0;k<N;++k) V[k]=(k%2==0)?0.1:-0.05;
            if(experiment==2) V[N-1]=0.2;
            if(experiment==3) V[0]=-0.3;
            V6 z=Eigen::Map<const V6>(z0.data());
            double direct=0;
            require(qp.constraint_count == 4*(N+1),"State constraint count");
        for(int k=0;k<N;++k) {
                direct+=z.dot(Q*z)+weights.r*V[k]*V[k];
                z=(As*z+Bs*V[k]).eval();
            }
            direct+=z.dot(P*z);
            const double expanded=0.5*V.dot(H*V)+g.dot(V)+qp.constant_cost;
            worst_cost=std::max(worst_cost,std::abs(direct-expanded));
            require(close(direct,expanded),"QP versus rollout cost mismatch");
        }
        const VN optimum=factor.solve(-g);
        require(optimum.allFinite(),"Non-finite optimum");
        double expected=0;
        for(int i=0;i<nx;++i) expected-=lqr.K[i]*z0[i];
        worst_input=std::max(worst_input,std::abs(optimum[0]-expected));
        require(close(optimum[0],expected),"First unconstrained input versus LQR");
        for(int k=0;k<N;++k) {
            require(qp.lower_bound[k]==config.input_min,"Lower bound");
            require(qp.upper_bound[k]==config.input_max,"Upper bound");
        }
    }
    auto rejects=[&](const LmpcConfig& c,const LqrConfig& w,const MatrixA& p) {
        bool rejected=false;
        try {build_lmpc_qp(scaled.A,scaled.B,StateVector{},c,w,p);}
        catch(const std::invalid_argument&) {rejected=true;}
        require(rejected,"Invalid QP data accepted");
    };
    auto bad_config=config; bad_config.input_min=2; rejects(bad_config,weights,lqr.P);
    auto bad_weights=weights; bad_weights.r=0; rejects(config,bad_weights,lqr.P);
    bad_weights=weights; bad_weights.q_diagonal[0]=-1; rejects(config,bad_weights,lqr.P);
    auto bad_P=lqr.P; bad_P[0]=std::numeric_limits<double>::quiet_NaN();
    rejects(config,weights,bad_P);
    bad_P=lqr.P; bad_P[1]+=1; rejects(config,weights,bad_P);
    bad_P={}; rejects(config,weights,bad_P);
    const auto constrained_problem = build_lmpc_qp(
        scaled.A, scaled.B, StateVector{0.2,-0.1,0,0,0,0},
        config, weights, lqr.P);
    const auto solution = solve_box_qp(constrained_problem);
    require(solution.solved, "Box QP did not converge");
    require(constrained_problem.constraint_count == 4 * (N + 1),
            "State constraints were not constructed");
    for (int i=0;i<N;++i)
        require(solution.variables[i] >= config.input_min - 1e-12 &&
                solution.variables[i] <= config.input_max + 1e-12,
                "Box QP violated input bound");
    // Coupled active state constraint: optimum of distance to (2,3) with
    // v0+v1<=1 and v>=0 is (0,1). Plain alternating projections can miss it.
    LmpcQpProblem coupled;
    for(int i=0;i<N;++i) {coupled.H[i*N+i]=1;coupled.lower_bound[i]=0;coupled.upper_bound[i]=10;}
    coupled.g[0]=-2;coupled.g[1]=-3;
    coupled.constraint_count=1;coupled.constraint_matrix[0]=1;coupled.constraint_matrix[1]=1;
    coupled.constraint_lower[0]=-10;coupled.constraint_upper[0]=1;
    const auto exact=solve_box_qp(coupled);
    require(exact.solved && std::abs(exact.variables[0])<1e-7 && std::abs(exact.variables[1]-1)<1e-7,
        "Coupled LMPC QP did not reach known constrained optimum");
    coupled.constraint_lower[0]=-2;coupled.constraint_upper[0]=-1;
    require(!solve_box_qp(coupled).solved,"Infeasible LMPC QP reported success");
    std::cout<<"QP construction and box solve checks passed; max cost difference="<<worst_cost
             <<", max first-input difference="<<worst_input<<"\n";
}
