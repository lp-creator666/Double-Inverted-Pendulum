#include "app/application.hpp"
#include <iostream>
#include <cmath>
#include <string>
#include <memory>
#include <stdexcept>

#include <mujoco/mujoco.h>

#include "app/viewer.hpp"
#include "types.hpp"
#include "model/dynamics.hpp"

#include "model/linearize.hpp"
#include "model/diagnostics.hpp"
#include "control/lqr.hpp"
#include "control/lmpc.hpp"
#include "optimization/sqp_solver.hpp"
#include "control/supervisor.hpp"
#include "swingup_controller.hpp"
#include "control/trajectory.hpp"

#include "control/online_sqp_controller.hpp"

namespace {
double parse_number(const std::string& text) {
    std::size_t end = 0;
    const double value = std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument("无效数字: " + text);
    }
    return value;
}
}

int run_application(int argc, char** argv) {
    RunOptions options;
    bool headless = false;
    bool use_lqr = false;
    bool use_lmpc = false;
    bool use_sqp = false;
    bool use_swingup = false;
    std::string plan_path;
    std::shared_ptr<SwingupPlan> plan;
    bool explicit_initial_state=false;
    bool track_plan=false;
    bool online_plan=false;
    double solve_budget=0.5;
    double plan_q1_offset=0,plan_q2_offset=0;
    double plan_p_offset=0,plan_dp_offset=0,plan_dq1_offset=0,plan_dq2_offset=0;
    double initial_q1 = 0.02;
    double initial_q2 = 0.0;
    double initial_dq1 = 0.0;
    double initial_dq2 = 0.0;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "用法: double_pendulum [选项]\n"
                             "  --headless          无窗口运行\n"
                             "  --lqr               LQR 局部平衡控制\n"
                             "  --lmpc              LMPC 约束平衡控制\n"
                             "  --sqp               SQP 非线性预测控制（基线）\n"
                             "  --swingup           能量摆起并切换 LQR\n"
                             "  --plan FILE         离线 SQP 轨迹执行后切换 LQR；初始状态取自文件\n"
                             "  --track-plan FILE   沿离线 SQP 轨迹做时变 LQR 反馈，再切换平衡 LQR\n"
                             "  --online-plan FILE  以轨迹为种子，每段从实测状态重新做 SQP\n"
                             "  --solve-budget S    在线单次软求解预算（默认 0.5 s；允许仿真慢于实时）\n"
                             "  --plan-q1-offset-deg DEG  第一关节初始偏差（不改变参考轨迹）\n"
                             "  --plan-q2-offset-deg DEG  第二关节初始相对角偏差\n"
                             "  --plan-p-offset M / --plan-dp-offset M_S  小车初始偏差\n"
                             "  --plan-dq1-offset RAD_S / --plan-dq2-offset RAD_S  初始速度偏差\n"
                             "  --push-force N      外加于小车的扰动力，独立于执行器输入\n"
                             "  --push-start S      扰动开始时刻，默认 1 秒\n"
                             "  --push-duration S   扰动持续时间\n"
                             "  --force N           恒定请求力（N）\n"
                             "  --duration S        仿真时长；无窗口模式必填\n"
                             "  --q1 RAD            第一关节初始角\n"
                             "  --q2 RAD            第二关节初始相对角\n"
                             "  --dq1 RAD_S         第一关节初始角速度\n"
                             "  --dq2 RAD_S         第二关节初始相对角速度\n"
                             "  --output FILE       CSV 输出路径\n";
                return 0;
            }
            if (arg == "--headless") {
                headless = true;
                continue;
            }
            if (arg == "--lqr") {
                use_lqr = true;
                continue;
            }
            if (arg == "--lmpc") {
                use_lmpc = true;
                continue;
            }
            if (arg == "--sqp") {
                use_sqp = true;
                continue;
            }
            if (arg == "--swingup") {
                use_swingup = true;
                continue;
            }
            if (i + 1 >= argc) throw std::invalid_argument("缺少参数值: " + arg);
            const std::string value = argv[++i];
            if (arg == "--force") {
                options.requested_force = parse_number(value);
            } else if (arg == "--duration") {
                options.duration = parse_number(value);
            } else if (arg == "--q1") {
                explicit_initial_state=true;
                initial_q1 = parse_number(value);
            } else if (arg == "--q2") {
                explicit_initial_state=true;
                initial_q2 = parse_number(value);
            } else if (arg == "--dq1") {
                explicit_initial_state=true;
                initial_dq1 = parse_number(value);
            } else if (arg == "--dq2") {
                explicit_initial_state=true;
                initial_dq2 = parse_number(value);
            } else if (arg == "--plan" || arg == "--track-plan" || arg == "--online-plan") {
                if(!plan_path.empty()) throw std::invalid_argument("Choose only one plan mode");
                plan_path=value;
                track_plan=arg=="--track-plan";
                online_plan=arg=="--online-plan";
            } else if(arg=="--solve-budget") {
                solve_budget=parse_number(value);
            } else if(arg=="--plan-q1-offset-deg") {
                plan_q1_offset=parse_number(value)*(std::acos(-1.0)/180);
            } else if(arg=="--plan-q2-offset-deg") {
                plan_q2_offset=parse_number(value)*(std::acos(-1.0)/180);
            } else if(arg=="--plan-p-offset") {
                plan_p_offset=parse_number(value);
            } else if(arg=="--plan-dp-offset") {
                plan_dp_offset=parse_number(value);
            } else if(arg=="--plan-dq1-offset") {
                plan_dq1_offset=parse_number(value);
            } else if(arg=="--plan-dq2-offset") {
                plan_dq2_offset=parse_number(value);
            } else if(arg=="--push-force") {
                options.push_force=parse_number(value);
            } else if(arg=="--push-start") {
                options.push_start=parse_number(value);
            } else if(arg=="--push-duration") {
                options.push_duration=parse_number(value);
            } else if (arg == "--output") {
                options.log_path = value;
            } else {
                throw std::invalid_argument("未知参数: " + arg);
            }
        }
        if ((use_lqr ? 1 : 0) + (use_lmpc ? 1 : 0) + (use_sqp ? 1 : 0) +
            (use_swingup ? 1 : 0) + (!plan_path.empty() ? 1 : 0) > 1) {
            throw std::invalid_argument("controller modes are mutually exclusive");
        }
        if(!plan_path.empty() && explicit_initial_state)
            throw std::invalid_argument("--plan supplies the initial state; do not combine with initial-state options");
        if(plan_path.empty() && (plan_q1_offset!=0 || plan_q2_offset!=0 || plan_p_offset!=0 || plan_dp_offset!=0 || plan_dq1_offset!=0 || plan_dq2_offset!=0))
            throw std::invalid_argument("Plan initial offsets require --plan or --track-plan");
        if(solve_budget<=0 || (!online_plan && solve_budget!=0.5))
            throw std::invalid_argument("Positive --solve-budget requires --online-plan");
        if(options.push_start<0 || options.push_duration<0 || (options.push_force!=0 && options.push_duration<=0))
            throw std::invalid_argument("Push requires nonnegative start and positive duration");
        if ((use_lqr || use_lmpc || use_sqp || use_swingup || !plan_path.empty()) && options.requested_force != 0.0) {
            throw std::invalid_argument("controller mode cannot be combined with a nonzero --force");
        }
        if (headless && options.duration <= 0) {
            throw std::invalid_argument("--headless 必须同时指定正的 --duration");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n使用 --help 查看用法。\n";
        return 1;
    }

    const char* xml_path = "model/double_cartpole.xml";

    char error[1024] = "";

    mjModel* model = mj_loadXML(
        xml_path,
        nullptr,
        error,
        sizeof(error)
    );

    if (model == nullptr) {
        std::cerr <<"MuJoCo模型加载失败：\n"
                  << error << std::endl;
        return 1;
    }
    model->opt.enableflags |= mjENBL_ENERGY;
    
    mjData* data = mj_makeData(model);

    if (data == nullptr) {
        std::cerr << "MuJoCo 数据创建失败" << std::endl;
        mj_deleteModel(model);
        return 1;
    }

    mjData* prediction_data = mj_makeData(model);
    if (prediction_data == nullptr) {
        std::cerr << "预测数据创建失败\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    std::cout << "MuJoCo 模型加载成功\n";
    std::cout << "时间步长: " << model->opt.timestep << " s\n";
    std::cout << "qpos 数量 nq: " << model->nq << "\n";
    std::cout << "qvel 数量 nv: " << model->nv << "\n";
    std::cout << "执行器数量 nu: " << model->nu << "\n";
    std::cout << "仿真时间: " << data->time << " s\n";

    const char* joint_names[3] = {
        "cart_slide",
        "pole_1_hinge",
        "pole_2_hinge"
    };

    int joint_ids[3];
    int qpos_addresses[3];
    int qvel_addresses[3];

    for (int i = 0; i < 3; ++i) {
        joint_ids[i] = mj_name2id(model, mjOBJ_JOINT, joint_names[i]);

        if (joint_ids[i] < 0) {
            std::cerr << "找不到关节: " << joint_names[i] << std::endl;
            mj_deleteData(prediction_data);
            mj_deleteData(data);
            mj_deleteModel(model);
            return 1;
        }

        qpos_addresses[i] = model->jnt_qposadr[joint_ids[i]];
        qvel_addresses[i] = model->jnt_dofadr[joint_ids[i]];

        std::cout << "关节: " << joint_names[i]
                  << ", ID: " << joint_ids[i]
                  << ", qpos 地址: " << qpos_addresses[i]
                  << ", qvel 地址: " << qvel_addresses[i] << "\n";
    }

    if(!plan_path.empty()) try {
        plan=std::make_shared<SwingupPlan>(load_swingup_plan(plan_path,model->opt.timestep,
            model->actuator_ctrlrange[0],model->actuator_ctrlrange[1]));
        const auto& start=plan->states.front();
        initial_q1=start.q1+plan_q1_offset; initial_q2=start.q2+plan_q2_offset;
        initial_dq1=start.dq1+plan_dq1_offset;initial_dq2=start.dq2+plan_dq2_offset;
        data->qpos[qpos_addresses[0]]=start.p+plan_p_offset;data->qvel[qvel_addresses[0]]=start.dp+plan_dp_offset;
    } catch(const std::exception& exception) {
        std::cerr<<exception.what()<<'\n';
        mj_deleteData(prediction_data);mj_deleteData(data);mj_deleteModel(model);return 1;
    }
    data->qpos[qpos_addresses[1]] = initial_q1;
    data->qpos[qpos_addresses[2]] = initial_q2;
    data->qvel[qvel_addresses[1]] = initial_dq1;
    data->qvel[qvel_addresses[2]] = initial_dq2;
    mj_forward(model, data);
    const State state = read_state(
        data,
        qpos_addresses,
        qvel_addresses
    );

    std::cout << "\n六维状态 [p, dp, q1, dq1, q2, dq2]:\n";

    std::cout << "["
              << state.p << ", "
              << state.dp << ", "
              << state.q1 << ", "
              << state.dq1 << ", "
              << state.q2 << ", "
              << state.dq2 << "]\n";

    std::cout << "第二杆绝对角: "
          << state.q1 + state.q2 << " rad\n";
          
    const State current_state = read_state(
        data,
        qpos_addresses,
        qvel_addresses
    );
    const State predicted_state = predict_state(
        model,
        prediction_data,
        current_state,
        qpos_addresses,
        qvel_addresses,
        options.requested_force,
        10
    );

    // 在直立平衡点附近计算单步离散线性模型。
    const State equilibrium_state{};
    const int linearization_steps = 1;
    const double state_epsilon = 1e-6;
    const double input_epsilon = 1e-6;
    const Linearization linearization = linearize_discrete(
        model,
        prediction_data,
        equilibrium_state,
        0.0,
        qpos_addresses,
        qvel_addresses,
        linearization_steps,
        state_epsilon,
        input_epsilon
    );

    std::cout << "预测 10 个物理步后的状态（约 0.02 s）：\n"
              << "[" << predicted_state.p << ", "
              << predicted_state.dp << ", "
              << predicted_state.q1 << ", "
              << predicted_state.dq1 << ", "
              << predicted_state.q2 << ", "
              << predicted_state.dq2 << "]\n";

    std::cout << "\n离散线性化矩阵 A:\n";
    for (int row = 0; row < kStateDimension; ++row) {
        std::cout << "[ ";
        for (int column = 0; column < kStateDimension; ++column) {
            std::cout << linearization.A[row * kStateDimension + column];
            if (column + 1 < kStateDimension) {
                std::cout << ", ";
            }
        }
        std::cout << " ]\n";
    }

    std::cout << "离散线性化矩阵 B:\n[ ";
    for (int row = 0; row < kStateDimension; ++row) {
        std::cout << linearization.B[row];
        if (row + 1 < kStateDimension) {
            std::cout << ", ";
        }
    }
    std::cout << " ]\n";

    try {
        run_model_diagnostics(model, prediction_data, qpos_addresses,
                              qvel_addresses, linearization);
        const LqrConfig config;
        const LqrResult lqr = solve_discrete_lqr(
            linearization.A, linearization.B, config);
        if(plan) {
            if(online_plan) {
                OnlineSqpConfig settings;settings.solve_budget=solve_budget;
                auto controller=std::make_shared<OnlineSqpController>(model,*plan,qpos_addresses,qvel_addresses,lqr,config,
                    options.log_path+".solver.csv",settings);
                options.feedback=[controller,data](const State& measured){return controller->compute_force(data->time,measured);};
                options.stage=[controller]{return controller->stage_code();};
            } else {
            std::shared_ptr<TrajectoryTracker> tracker;
            if(track_plan) {
                tracker=std::make_shared<TrajectoryTracker>(model,prediction_data,*plan,qpos_addresses,qvel_addresses,lqr,config);
                std::cout<<"Trajectory TVLQR: "<<tracker->steps()<<" gains, setup "<<tracker->setup_seconds()<<" s\n";
            }
            options.feedback=[plan,tracker,lqr,config,data,balancing=false](const State& measured) mutable {
                if(data->time+1e-10<plan->duration())
                    return tracker?tracker->compute_force(data->time,measured):plan->force_at(data->time);
                if(!balancing) {
                    if(!terminal_capture_ready(measured)) throw std::runtime_error("Planned swing-up missed terminal capture region");
                    balancing=true;
                    std::cout<<"Offline SQP plan -> LQR at t="<<data->time<<" s\n";
                }
                return compute_lqr_force(local_balance_state(measured),lqr,config);
            };
            options.stage=[plan,data,track_plan]{return data->time+1e-10>=plan->duration()?3:(track_plan?5:1);};
            }
        } else if (use_lqr) {
            options.stage=[] {return 3;};
            options.feedback = [lqr, config](const State& state) {
                return compute_lqr_force(state, lqr, config);
            };
        } else if (use_lmpc) {
            options.stage=[] {return 6;};
            LmpcConfig mpc_config;
            mpc_config.input_min=model->actuator_ctrlrange[0]/mpc_config.scaling.input_scale;
            mpc_config.input_max=model->actuator_ctrlrange[1]/mpc_config.scaling.input_scale;
            const int cart=mj_name2id(model,mjOBJ_JOINT,"cart_slide");
            mpc_config.cart_position_min=model->jnt_range[2*cart]+0.15;
            mpc_config.cart_position_max=model->jnt_range[2*cart+1]-0.15;
            options.feedback = [linearization, lqr, config, mpc_config](const State& state) {
                const LmpcControlResult control = compute_lmpc_control(
                    linearization.A, linearization.B, state, lqr, mpc_config, config);
                if (!control.solved) {
                    throw std::runtime_error("LMPC QP did not converge");
                }
                return control.requested_force;
            };
        } else if (use_sqp || use_swingup) {
            SqpConfig sqp_config;
            sqp_config.max_iterations = 4;
            sqp_config.steps_per_control = 5;
            sqp_config.max_line_search = 5;
            NmpcCostConfig nmpc_cost;
            auto warm_start = std::make_shared<std::vector<double>>(12, 0.0);
            auto previous_force = std::make_shared<double>(0.0);
            auto held_force = std::make_shared<double>(0.0);
            auto hold_count = std::make_shared<int>(0);
            auto supervisor = std::make_shared<Supervisor>();
            options.stage=[supervisor]{return supervisor->stage()==ControlStage::Lqr?3:
                supervisor->stage()==ControlStage::SwingUp?7:2;};
            auto swingup = std::make_shared<SwingupController>(model, prediction_data);
            if (use_swingup) supervisor->start_swingup();
            options.feedback = [&, sqp_config, nmpc_cost, lqr, config,
                                warm_start, previous_force,
                                held_force, hold_count, supervisor, swingup](const State& state) mutable {
                const ControlStage stage = supervisor->update(state, true);
                if (stage == ControlStage::Fault) {
                    throw std::runtime_error("Supervisor entered fault state");
                }
                if (stage == ControlStage::Lqr) {
                    *hold_count=0;
                    const double force=compute_lqr_force(local_balance_state(state), lqr, config);
                    *previous_force=std::clamp(force,model->actuator_ctrlrange[0],model->actuator_ctrlrange[1]);
                    std::fill(warm_start->begin(),warm_start->end(),*previous_force);
                    return force;
                }
                if (stage == ControlStage::SwingUp) {
                    double force = swingup->compute_force(model, data, state);
                    if (!std::isfinite(force)) {
                        throw std::runtime_error("Swing-up produced non-finite force");
                    }
                    return std::clamp(force, model->actuator_ctrlrange[0],
                                      model->actuator_ctrlrange[1]);
                }
                if (*hold_count > 0) {
                    --(*hold_count);
                    return *held_force;
                }
                const SqpResult result = solve_nmpc_sqp(
                    model, prediction_data, state, qpos_addresses, qvel_addresses,
                    *warm_start, nmpc_cost, sqp_config, *previous_force);
                if (!result.feasible || (!result.solved && result.accepted_steps == 0)) {
                    supervisor->update(state, false);
                    throw std::runtime_error("SQP failed outside capture region: " + result.status);
                }
                const double force = result.forces.front();
                for (std::size_t i = 0; i + 1 < warm_start->size(); ++i) {
                    (*warm_start)[i] = result.forces[i + 1];
                }
                warm_start->back() = result.forces.back();
                *previous_force = force;
                *held_force = force;
                *hold_count = 4;
                return force;
            };
        }
        std::cout << "Control mode: "
                  << (plan ? (online_plan?"Seeded online SQP -> LQR":track_plan?"Offline SQP plan + TVLQR -> LQR":"Offline SQP plan -> LQR") : use_lqr ? "LQR" : (use_lmpc ? "LMPC" :
                     (use_sqp ? "SQP baseline" :
                     (use_swingup ? "SwingUp -> LQR" : "constant force"))))
                  << "\n";

        std::cout << "\nLQR numerical verification:\n"
                  << "iterations: " << lqr.iterations << "\n"
                  << "relative change: " << lqr.final_change << "\n"
                  << "Riccati residual: " << lqr.riccati_residual << "\n"
                  << "P symmetry error: " << lqr.symmetry_error << "\n"
                  << "P minimum eigenvalue: " << lqr.min_p_eigenvalue << "\n"
                  << "closed-loop spectral radius: " << lqr.closed_loop_radius << "\n"
                  << "K in scaled coordinates: [ ";
        for (double value : lqr.K) std::cout << value << " ";
        std::cout << "]\n";

    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        mj_deleteData(prediction_data);
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    const int run_result = headless
        ? run_headless(model, data, qpos_addresses, qvel_addresses, options)
        : run_viewer(model, data, qpos_addresses, qvel_addresses, options);

    mj_deleteData(prediction_data);
    mj_deleteData(data);
    mj_deleteModel(model);

    return run_result;
}
