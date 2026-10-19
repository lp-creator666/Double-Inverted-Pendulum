#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#include "viewer.hpp"
#include "types.hpp"

State read_state(
    const mjData* data,
    const int* qpos_addresses,
    const int* qvel_addresses
){
    State state;
    state.p = data->qpos[qpos_addresses[0]];
    state.dp = data->qvel[qvel_addresses[0]];

    state.q1 = data->qpos[qpos_addresses[1]];
    state.dq1 = data->qvel[qvel_addresses[1]];
    state.q2 = data->qpos[qpos_addresses[2]];
    state.dq2 = data->qvel[qvel_addresses[2]];
    return state;
} //读取函数

int main() {
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
    
    mjData* data = mj_makeData(model);

    if (data == nullptr) {
        std::cerr << "MuJoCo 数据创建失败" << std::endl;
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
          
    // 第一关节偏转 0.02 rad，第二关节相对角保持为零。
    data->qpos[qpos_addresses[1]] = 0.02;
    mj_forward(model, data);

    std::cout << "打开可视化窗口：零输入自由运动，Esc 退出。\n";
    const int viewer_result = run_viewer(model, data);

    mj_deleteData(data);
    mj_deleteModel(model);

    return viewer_result;
}
