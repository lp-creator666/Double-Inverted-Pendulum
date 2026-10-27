cmake_minimum_required(VERSION 3.16)

project(DoubleInvertedPendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
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

add_library(pendulum_core STATIC
    src/model/dynamics.cpp
    src/model/linearize.cpp
    src/model/diagnostics.cpp
    src/control/lqr.cpp
    src/control/lmpc.cpp
    src/control/lmpc_qp.cpp
    src/control/trajectory.cpp
    src/control/supervisor.cpp
    src/control/online_sqp_controller.cpp
    src/optimization/nonlinear_model.cpp
    src/optimization/nmpc_cost.cpp
    src/optimization/sqp_solver.cpp
    src/optimization/dense_qp.cpp
)
target_include_directories(pendulum_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include" "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(pendulum_core PUBLIC Eigen3::Eigen "${MUJOCO_LIBRARY}")

# Kept for comparison and teaching; multiple shooting is not the main SQP algorithm.
add_library(pendulum_research STATIC
    research/swingup_controller.cpp research/nmpc_shooting.cpp)
target_include_directories(pendulum_research PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/research/include")
target_include_directories(pendulum_research PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/research/include")
target_link_libraries(pendulum_research PUBLIC pendulum_core)

add_executable(double_pendulum src/main.cpp src/app/application.cpp
    src/app/viewer.cpp src/app/logger.cpp)
target_include_directories(double_pendulum PRIVATE "${GLFW_INCLUDE_DIR}")
target_link_libraries(double_pendulum PRIVATE pendulum_core pendulum_research
    "${GLFW_LIBRARY}" OpenGL::GL)
if(WIN32)
    target_link_libraries(double_pendulum PRIVATE gdi32)
    add_custom_command(TARGET double_pendulum POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_ROOT}/mujoco.dll" "$<TARGET_FILE_DIR:double_pendulum>")
else()
    find_package(Threads REQUIRED)
    target_link_libraries(double_pendulum PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
endif()
set_target_properties(double_pendulum PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}")

add_executable(plan_swingup tools/plan_swingup.cpp)
target_link_libraries(plan_swingup PRIVATE pendulum_core)
set_target_properties(plan_swingup PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}")

include(CTest)
if(BUILD_TESTING)
    # Reuse the same compiled implementation in the application and every test.
    function(pendulum_test target source test_name)
        add_executable(${target} ${source})
        target_link_libraries(${target} PRIVATE pendulum_core)
        set_target_properties(${target} PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}")
        add_test(NAME ${test_name} COMMAND ${target})
        set_tests_properties(${test_name} PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endfunction()
    pendulum_test(test_nmpc_trajectory_cost tests/optimization/test_nmpc_trajectory_cost.cpp nmpc_trajectory_cost)
    pendulum_test(test_nmpc_shooting tests/research/test_nmpc_shooting.cpp nmpc_multiple_shooting)
    target_link_libraries(test_nmpc_shooting PRIVATE pendulum_research)
    pendulum_test(test_nmpc_linearization tests/optimization/test_nmpc_linearization.cpp nmpc_trajectory_linearization)
    pendulum_test(test_sqp_solver tests/optimization/test_sqp_solver.cpp sqp_baseline)
    pendulum_test(test_supervisor tests/control/test_supervisor.cpp supervisor_hysteresis)
    pendulum_test(test_swingup_controller tests/research/test_swingup_controller.cpp swingup_controller)
    target_link_libraries(test_swingup_controller PRIVATE pendulum_research)
    pendulum_test(test_controllability tests/model/test_controllability.cpp controllability_svd)
    pendulum_test(test_lqr tests/control/test_lqr.cpp lqr_solver)
    pendulum_test(test_lmpc tests/control/test_lmpc.cpp lmpc_prediction)
    pendulum_test(test_lmpc_qp tests/control/test_lmpc_qp.cpp lmpc_qp_construction)
    pendulum_test(test_lmpc_controller tests/control/test_lmpc_controller.cpp lmpc_controller)
    pendulum_test(test_nmpc_prediction tests/optimization/test_nmpc_prediction.cpp nmpc_prediction)
    pendulum_test(test_dense_qp tests/optimization/test_dense_qp.cpp dense_qp_kkt)
    pendulum_test(test_swingup_plan tests/control/test_swingup_plan.cpp swingup_plan_validation)
    pendulum_test(test_trajectory_tracker tests/control/test_trajectory_tracker.cpp trajectory_tracking_numerics)

    add_test(NAME local_model_diagnostics COMMAND double_pendulum
        --headless --duration 0.002 --force 0 --output logs/model_check.csv)
    set_tests_properties(local_model_diagnostics PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        function(pendulum_acceptance name script seconds)
            add_test(NAME ${name} COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/acceptance/${script}"
                "$<TARGET_FILE:double_pendulum>")
            set_tests_properties(${name} PROPERTIES
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT ${seconds})
        endfunction()
        pendulum_acceptance(planned_swingup_closed_loop check_planned_swingup.py 90)
        pendulum_acceptance(trajectory_tracking_closed_loop check_trajectory_tracking.py 300)
        pendulum_acceptance(project_acceptance check_project_acceptance.py 600)
        set_tests_properties(project_acceptance PROPERTIES LABELS acceptance)
    endif()
endif()