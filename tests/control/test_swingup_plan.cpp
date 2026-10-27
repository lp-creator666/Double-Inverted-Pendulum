#include "control/trajectory.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main() try {
    const auto path=std::filesystem::temp_directory_path()/"double_pendulum_plan_test.csv";
    auto write=[&](const std::string& rows) {
        std::ofstream out(path); out<<"time,p,dp,q1,dq1,q2,dq2,force\n"<<rows;
    };
    auto reject=[&](const std::string& rows) {
        write(rows);bool rejected=false;
        try {load_swingup_plan(path.string(),0.002,-20,20);} catch(const std::invalid_argument&) {rejected=true;}
        if(!rejected) throw std::runtime_error("Invalid trajectory accepted");
    };
    write("0,0,0,3.141592653589793,0,0,0,1\n0.08,0,0,0,0,0,0,-2\n0.16,0,0,0,0,0,0,0\n");
    const auto plan=load_swingup_plan(path.string(),0.002,-20,20);
    if(plan.force_at(0)!=1 || plan.force_at(0.078)!=1 || plan.force_at(0.08)!=-2 ||
        plan.force_at(0.08-1e-14)!=-2 || plan.duration()!=0.16)
        throw std::runtime_error("Plan sample timing mismatch");
    reject("0,0,0,0,0,0,0,21\n0.08,0,0,0,0,0,0,0\n");
    reject("0,0,0,0,0,0,0,1\n0.081,0,0,0,0,0,0,0\n");
    reject("0,0,0,0,0,0,0,1\n0,0,0,0,0,0,0,0\n");
    reject("0,0,0,0,0,0,0,1\n0.08,0,0,0,0,3.14,0,0\n");
    reject("0,0,0,0,0,0,0,1\n0.08,0,0,0,0,0,0,0,extra\n");
    reject("0,0,0,0,0,0,0,nan\n0.08,0,0,0,0,0,0,0\n");
    reject("");
    std::filesystem::remove(path);
    std::cout<<"Plan parsing, timing, force bounds and terminal checks passed\n";
    return 0;
} catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
