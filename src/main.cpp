#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

int main(){
    const char* xml_path = "model/double_cartpole.xml";

    char error[1024] = "";

    mjModel* model = mj_loadXML(
        xml_path,
        nullptr,
        error,
        sizeof(error)
    );

    if(model == nullptr){
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

    mj_deleteData(data);
    mj_deleteModel(model);

    return 0;
}
