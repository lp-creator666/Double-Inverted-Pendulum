#include "viewer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <GLFW/glfw3.h>

#include "logger.hpp"
#include "mujoco_interface.hpp"

namespace {
void report_glfw_error(int code, const char* description) {
    std::cerr << "GLFW error " << code << ": " << description << '\n';
}

bool valid_state(const mjModel* model, const mjData* data) {
    if (!std::isfinite(data->time)) return false;
    for (int i = 0; i < model->nq; ++i) {
        if (!std::isfinite(data->qpos[i])) return false;
    }
    for (int i = 0; i < model->nv; ++i) {
        if (!std::isfinite(data->qvel[i])) return false;
    }
    return true;
}

bool has_mujoco_warning(const mjData* data) {
    for (int i = 0; i < mjNWARNING; ++i) {
        if (data->warning[i].number != 0) return true;
    }
    return false;
}

struct SafetyLimits {
    double min_position;
    double max_position;
    double max_cart_speed = 10.0;
    double max_joint_speed = 50.0;
};

bool step_and_log(const mjModel* model, mjData* data,
                  const int* qpos_addresses, const int* qvel_addresses,
                  const RunOptions& options, const SafetyLimits& limits,
                  CsvLogger& logger, std::string& stop_reason) {
    const State current_state = read_state(data, qpos_addresses, qvel_addresses);
    if (!valid_state(model, data) || !current_state.finite()) {
        stop_reason = "non-finite state before control";
        return false;
    }
    const double requested = options.feedback
        ? options.feedback(current_state) : options.requested_force;
    if (!std::isfinite(requested)) {
        stop_reason = "non-finite requested force";
        return false;
    }
    const double applied = std::clamp(
        requested,
        model->actuator_ctrlrange[0],
        model->actuator_ctrlrange[1]
    );
    mju_zero(data->ctrl, model->nu);
    data->ctrl[0] = applied;
    mj_step(model, data);

    const State state = read_state(data, qpos_addresses, qvel_addresses);
    logger.write(data->time, state, requested, applied);

    if (!valid_state(model, data) || !state.finite()) {
        stop_reason = "non-finite state";
    } else if (has_mujoco_warning(data)) {
        stop_reason = "MuJoCo warning";
    } else if (state.p <= limits.min_position || state.p >= limits.max_position) {
        stop_reason = "cart reached track safety margin";
    } else if (std::abs(state.dp) > limits.max_cart_speed) {
        stop_reason = "cart speed limit exceeded";
    } else if (std::abs(state.dq1) > limits.max_joint_speed ||
               std::abs(state.dq2) > limits.max_joint_speed) {
        stop_reason = "joint speed limit exceeded";
    }
    return stop_reason.empty();
}

bool prepare_run(const mjModel* model, const mjData* data,
                 SafetyLimits& limits, std::string& error) {
    if (model->nu != 1 || !model->actuator_ctrllimited[0] ||
        !std::isfinite(model->actuator_ctrlrange[0]) ||
        !std::isfinite(model->actuator_ctrlrange[1]) ||
        model->actuator_ctrlrange[0] >= model->actuator_ctrlrange[1]) {
        error = "Expected one actuator with a finite control range";
        return false;
    }
    const int cart_joint = mj_name2id(model, mjOBJ_JOINT, "cart_slide");
    if (cart_joint < 0 || !model->jnt_limited[cart_joint]) {
        error = "缺少带范围的 cart_slide 关节";
        return false;
    }
    const double track_min = model->jnt_range[2 * cart_joint];
    const double track_max = model->jnt_range[2 * cart_joint + 1];
    constexpr double track_margin = 0.10;
    limits.min_position = track_min + track_margin;
    limits.max_position = track_max - track_margin;
    if (!std::isfinite(limits.min_position) ||
        !std::isfinite(limits.max_position) ||
        limits.min_position >= limits.max_position) {
        error = "cart_slide 轨道范围无效";
        return false;
    }
    if (!valid_state(model, data)) {
        error = "初始状态不是有限值";
        return false;
    }
    return true;
}

void print_periodic_state(const mjData* data, const int* qpos_addresses,
                          const int* qvel_addresses, double& next_print_time) {
    if (data->time < next_print_time) return;
    const State state = read_state(data, qpos_addresses, qvel_addresses);
    std::cout << "t=" << data->time
              << ", p=" << state.p
              << ", q1=" << state.q1
              << ", q2=" << state.q2
              << ", pole2_angle=" << state.q1 + state.q2 << '\n';
    next_print_time += 0.5;
}
} // namespace

int run_headless(const mjModel* model, mjData* data,
                 const int* qpos_addresses, const int* qvel_addresses,
                 const RunOptions& options) {
    if (!model || !data || !std::isfinite(model->opt.timestep) ||
        model->opt.timestep <= 0 || options.duration <= 0 ||
        !std::isfinite(options.requested_force)) {
        std::cerr << "无窗口运行参数无效\n";
        return 1;
    }
    try {
        CsvLogger logger(options.log_path);
        SafetyLimits limits;
        std::string error;
        if (!prepare_run(model, data, limits, error)) {
            std::cerr << "Simulation stopped: " << error << '\n';
            return 1;
        }
        logger.write(data->time, read_state(data, qpos_addresses, qvel_addresses),
                     0.0, 0.0);
        double next_print_time = data->time + 0.5;
        const long long steps = static_cast<long long>(
            std::ceil(options.duration / model->opt.timestep));
        for (long long i = 0; i < steps; ++i) {
            std::string stop_reason;
            if (!step_and_log(model, data, qpos_addresses, qvel_addresses,
                              options, limits, logger, stop_reason)) {
                std::cerr << "Simulation stopped: " << stop_reason << '\n';
                return 1;
            }
            print_periodic_state(data, qpos_addresses, qvel_addresses, next_print_time);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}

int run_viewer(const mjModel* model, mjData* data,
               const int* qpos_addresses, const int* qvel_addresses,
               const RunOptions& options) {
    if (!model || !data || !std::isfinite(model->opt.timestep) ||
        model->opt.timestep <= 0 || !std::isfinite(options.requested_force)) {
        std::cerr << "窗口运行参数无效\n";
        return 1;
    }
    try {
        SafetyLimits limits;
        std::string error;
        if (!prepare_run(model, data, limits, error)) {
            std::cerr << "Simulation stopped: " << error << '\n';
            return 1;
        }
        CsvLogger logger(options.log_path);
        glfwSetErrorCallback(report_glfw_error);
        if (!glfwInit()) return 1;
        GLFWwindow* window = glfwCreateWindow(
            1200, 800, "Double Inverted Pendulum - Esc to close", nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            return 1;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        mjvCamera camera;
        mjvOption option;
        mjvScene scene;
        mjrContext context;
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
        mjv_makeScene(model, &scene, 2000);
        mjr_makeContext(model, &context, mjFONTSCALE_150);
        camera.type = mjCAMERA_FREE;
        camera.lookat[0] = 0;
        camera.lookat[1] = 0;
        camera.lookat[2] = 3.0;
        camera.distance = 9;
        camera.azimuth = 90;
        camera.elevation = -15;

        logger.write(data->time, read_state(data, qpos_addresses, qvel_addresses),
                     0.0, 0.0);
        double previous_wall_time = glfwGetTime();
        double accumulated_time = 0;
        double next_print_time = data->time + 0.5;
        int result = 0;
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            if (options.duration > 0 && data->time >= options.duration) break;
            const double current_wall_time = glfwGetTime();
            accumulated_time += std::min(current_wall_time - previous_wall_time, 0.05);
            previous_wall_time = current_wall_time;
            while (accumulated_time >= model->opt.timestep) {
                std::string stop_reason;
                if (!step_and_log(model, data, qpos_addresses, qvel_addresses,
                                  options, limits, logger, stop_reason)) {
                    std::cerr << "Simulation stopped: " << stop_reason << '\n';
                    result = 1;
                    break;
                }
                accumulated_time -= model->opt.timestep;
                print_periodic_state(data, qpos_addresses, qvel_addresses, next_print_time);
                if (options.duration > 0 && data->time >= options.duration) break;
            }
            if (result != 0) break;
            int width = 0, height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            if (width > 0 && height > 0) {
                const mjrRect viewport{0, 0, width, height};
                mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
                mjr_render(viewport, &scene, &context);
                glfwSwapBuffers(window);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        mjr_freeContext(&context);
        mjv_freeScene(&scene);
        glfwDestroyWindow(window);
        glfwTerminate();
        return result;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
