#include "viewer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <GLFW/glfw3.h>

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
    // MuJoCo 在数值失稳时可能自动重置数据，因此也检查警告。
    for (int i = 0; i < mjNWARNING; ++i) {
        if (data->warning[i].number != 0) return false;
    }
    return true;
}
} // namespace

int run_viewer(const mjModel* model, mjData* data) {
    if (!model || !data || !std::isfinite(model->opt.timestep) ||
        model->opt.timestep <= 0) {
        std::cerr << "Invalid model, data or timestep\n";
        return 1;
    }

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

    double previous_wall_time = glfwGetTime();
    double accumulated_time = 0;
    int result = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (glfwWindowShouldClose(window)) break;

        const double current_wall_time = glfwGetTime();
        // 卡顿时最多补算 50 ms，避免窗口长时间无响应。
        accumulated_time += std::min(current_wall_time - previous_wall_time, 0.05);
        previous_wall_time = current_wall_time;

        while (accumulated_time >= model->opt.timestep) {
            // 当前仅演示无控制仿真；后续在公共仿真循环接入控制器。
            mju_zero(data->ctrl, model->nu);
            mj_step(model, data);
            accumulated_time -= model->opt.timestep;
            if (!valid_state(model, data)) {
                std::cerr << "Simulation stopped: non-finite state or MuJoCo warning\n";
                result = 1;
                break;
            }
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

    // 渲染资源释放时 OpenGL 上下文必须仍然存在。
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
