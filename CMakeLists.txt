cmake_minimum_required(VERSION 3.16)

project(DoubleInvertedPendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen complex SVD/eigensolver templates exceed the default MinGW section limit.
if(MINGW)
    add_compile_options(-Wa,-mbig-obj)
endif()

# 当前机器上的 MuJoCo 头文件和动态库位置。
set(MUJOCO_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/build/deps/mujoco"
    CACHE PATH "MuJoCo installation directory"
)

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
if(WIN32)
    if(MINGW)
        set(MUJOCO_LIBRARY "${MUJOCO_ROOT}/mujoco.dll")
    else()
        find_library(MUJOCO_LIBRARY NAMES mujoco HINTS "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib" REQUIRED)
    endif()
else()
    find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco.so.3.4.0
        HINTS "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib" REQUIRED)
endif()

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
    message(FATAL_ERROR
        "MuJoCo header not found: ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
endif()

if(NOT EXISTS "${MUJOCO_LIBRARY}")
    message(FATAL_ERROR
        "MuJoCo library not found: ${MUJOCO_LIBRARY}")
endif()

find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    find_path(EIGEN3_INCLUDE_DIR Eigen/SVD
        HINTS "${CMAKE_CURRENT_SOURCE_DIR}/build/deps/eigen-3.4.0"
        PATH_SUFFIXES eigen3 REQUIRED)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

find_package(OpenGL REQUIRED)
get_filename_component(COMPILER_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
get_filename_component(COMPILER_ROOT "${COMPILER_BIN}" DIRECTORY)
find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h HINTS "${COMPILER_ROOT}/include" REQUIRED)
find_library(GLFW_LIBRARY NAMES glfw3 glfw HINTS "${COMPILER_ROOT}/lib" REQUIRED)

add_executable(double_pendulum
    src/main.cpp
    src/lqr.cpp
    src/lmpc.cpp
    src/model_diagnostics.cpp
    src/viewer.cpp
    src/mujoco_interface.cpp
    src/logger.cpp
    src/dynamics.cpp
    src/linearize.cpp
    src/controllability.cpp
    src/scaling.cpp
)

target_include_directories(double_pendulum PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${GLFW_INCLUDE_DIR}"
)

target_link_libraries(double_pendulum PRIVATE
    Eigen3::Eigen
    "${MUJOCO_LIBRARY}"
    "${GLFW_LIBRARY}"
    OpenGL::GL
)

if(WIN32)
    target_link_libraries(double_pendulum PRIVATE gdi32)
    add_custom_command(TARGET double_pendulum POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_ROOT}/mujoco.dll" "$<TARGET_FILE_DIR:double_pendulum>")
else()
    find_package(Threads REQUIRED)
    target_link_libraries(double_pendulum PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
endif()

set_target_properties(double_pendulum PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}"
)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_controllability
        tests/test_controllability.cpp
        src/controllability.cpp)
    target_include_directories(test_controllability PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include" "${MUJOCO_INCLUDE_DIR}")
    target_link_libraries(test_controllability PRIVATE Eigen3::Eigen)
    add_test(NAME controllability_svd COMMAND test_controllability)
endif()

if(BUILD_TESTING)
    add_test(NAME local_model_diagnostics COMMAND double_pendulum
        --headless --duration 0.002 --force 0 --output logs/model_check.csv)
    set_tests_properties(local_model_diagnostics PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

if(BUILD_TESTING)
    add_executable(test_lqr tests/test_lqr.cpp src/lqr.cpp src/scaling.cpp)
    target_include_directories(test_lqr PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include" "${MUJOCO_INCLUDE_DIR}")
    target_link_libraries(test_lqr PRIVATE Eigen3::Eigen)
    add_test(NAME lqr_solver COMMAND test_lqr)
endif()

if(BUILD_TESTING)
    add_executable(test_lmpc tests/test_lmpc.cpp src/lmpc.cpp src/scaling.cpp)
    target_include_directories(test_lmpc PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include" "${MUJOCO_INCLUDE_DIR}")
    target_link_libraries(test_lmpc PRIVATE Eigen3::Eigen)
    add_test(NAME lmpc_prediction COMMAND test_lmpc)
endif()
