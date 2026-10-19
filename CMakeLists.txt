cmake_minimum_required(VERSION 3.16)

project(DoubleInvertedPendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

find_package(OpenGL REQUIRED)
get_filename_component(COMPILER_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
get_filename_component(COMPILER_ROOT "${COMPILER_BIN}" DIRECTORY)
find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h HINTS "${COMPILER_ROOT}/include" REQUIRED)
find_library(GLFW_LIBRARY NAMES glfw3 glfw HINTS "${COMPILER_ROOT}/lib" REQUIRED)

add_executable(double_pendulum
    src/main.cpp
    src/viewer.cpp
)

target_include_directories(double_pendulum PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${GLFW_INCLUDE_DIR}"
)

target_link_libraries(double_pendulum PRIVATE
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
