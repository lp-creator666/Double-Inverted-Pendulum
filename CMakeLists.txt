cmake_minimum_required(VERSION 3.16)

project(DoubleInvertedPendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 当前机器上的 MuJoCo 头文件和动态库位置。
set(MUJOCO_ROOT
    "/home/w/.local/lib/python3.10/site-packages/mujoco"
    CACHE PATH "MuJoCo installation directory"
)

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_LIBRARY "${MUJOCO_ROOT}/libmujoco.so.3.4.0")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
    message(FATAL_ERROR
        "MuJoCo header not found: ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
endif()

if(NOT EXISTS "${MUJOCO_LIBRARY}")
    message(FATAL_ERROR
        "MuJoCo library not found: ${MUJOCO_LIBRARY}")
endif()

find_package(Eigen3 REQUIRED)

add_executable(double_pendulum
    src/main.cpp
)

target_include_directories(double_pendulum PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
)

target_link_libraries(double_pendulum PRIVATE
    "${MUJOCO_LIBRARY}"
    Eigen3::Eigen
    dl
    pthread
    m
)

set_target_properties(double_pendulum PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}"
)
