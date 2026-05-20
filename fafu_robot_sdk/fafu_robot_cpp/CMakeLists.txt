# =============================================================================
#  fafu_robot_sdk/fafu_robot_cpp/CMakeLists.txt
#
#  ★ 完全自包含 ★
#  本目录已经把所有需要的源码 (协议层 + serial_cmake 三方串口库) 都 vendor 进来,
#  所以构建过程不再引用 ../../Panthera-HT_SDK/ 任何内容. 你可以把整个
#  fafu_robot_sdk/ 单独拷到任何位置, 仍然能在 fafu_robot_cpp/ 内完成构建.
#
#  构建 (Windows / VS 2022, Release, 推荐):
#    cd fafu_robot_sdk\fafu_robot_cpp
#    build.bat
#
#  手工 (跨平台):
#    cd fafu_robot_sdk/fafu_robot_cpp
#    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
#          -Dpybind11_DIR="$(python -c 'import pybind11; print(pybind11.get_cmake_dir())')"
#    cmake --build build --config Release -j
#
#  产物 (POST_BUILD 自动复制到 ../fafu_robot_python/):
#    panthera_motor.cpXY-win_amd64.pyd   ← pybind11 模块
#    serial_cmake.dll                    ← Windows 运行时依赖
#
#  关键 CMake 变量 (按需 -D 覆盖):
#    -DFAFU_ROBOT_PYTHON_DIR=<path>    ← 产物复制目标 (默认 ../fafu_robot_python)
#    -Dpybind11_DIR=<cmake_dir>        ← pybind11 cmake 配置目录
# =============================================================================
cmake_minimum_required(VERSION 3.18)
project(panthera_motor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

if(MSVC)
    add_compile_options(/utf-8 /W3 /bigobj)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# Output everything (libs, .pyd, helpers) under build/bin so .dll 和 .pyd
# 永远在同一目录, 调试时手工跑也方便.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# -----------------------------------------------------------------------------
#  pybind11
# -----------------------------------------------------------------------------
find_package(pybind11 CONFIG REQUIRED)

# -----------------------------------------------------------------------------
#  Vendored: serial_cmake (跨平台串口库, 原 wjwwood/serial 的 fork)
# -----------------------------------------------------------------------------
set(_SERIAL_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_part/serial_cmake)
if(NOT EXISTS ${_SERIAL_CMAKE_DIR}/CMakeLists.txt)
    message(FATAL_ERROR
        "未找到 third_part/serial_cmake/. 这是已 vendor 的源码, 不应缺失.\n"
        "如果丢了, 从 Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/serial_cmake/ 重新拷一份.")
endif()
add_subdirectory(${_SERIAL_CMAKE_DIR} ${CMAKE_BINARY_DIR}/serial_cmake_build)

# -----------------------------------------------------------------------------
#  Vendored: hightorque_serial (调试板 USB 串口协议 C++ 实现)
# -----------------------------------------------------------------------------
add_library(hightorque_serial_debug STATIC
    src/hightorque_serial.cpp
)

target_include_directories(hightorque_serial_debug PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hightorque_serial_debug PUBLIC serial_cmake)

# -----------------------------------------------------------------------------
#  pybind11 模块: panthera_motor
#  (名字保留 "panthera_motor" 以兼容 FafuRobotController 里的 `import panthera_motor`.)
# -----------------------------------------------------------------------------
pybind11_add_module(panthera_motor
    bindings.cpp
)

target_link_libraries(panthera_motor PRIVATE
    hightorque_serial_debug
)

target_include_directories(panthera_motor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -----------------------------------------------------------------------------
#  POST_BUILD: 把产物 (.pyd + serial_cmake.dll) 复制到 ../fafu_robot_python/
# -----------------------------------------------------------------------------
if(NOT DEFINED FAFU_ROBOT_PYTHON_DIR)
    set(FAFU_ROBOT_PYTHON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../fafu_robot_python)
endif()

add_custom_command(TARGET panthera_motor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FAFU_ROBOT_PYTHON_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:panthera_motor>
        ${FAFU_ROBOT_PYTHON_DIR}/$<TARGET_FILE_NAME:panthera_motor>
    COMMENT "复制 panthera_motor 模块到 ${FAFU_ROBOT_PYTHON_DIR}/"
)

if(WIN32)
    add_custom_command(TARGET panthera_motor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:serial_cmake>
            ${FAFU_ROBOT_PYTHON_DIR}/$<TARGET_FILE_NAME:serial_cmake>
        COMMENT "复制 serial_cmake.dll 到 ${FAFU_ROBOT_PYTHON_DIR}/"
    )
endif()

# -----------------------------------------------------------------------------
#  Configure-time summary
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "================================================================")
message(STATUS " fafu_robot_cpp (panthera_motor pybind11) 配置完成")
message(STATUS "   pybind11_DIR             = ${pybind11_DIR}")
message(STATUS "   Python                   = ${PYTHON_EXECUTABLE} (${PYTHON_VERSION_STRING})")
message(STATUS "   Vendored serial_cmake    = ${_SERIAL_CMAKE_DIR}")
message(STATUS "   FAFU_ROBOT_PYTHON_DIR    = ${FAFU_ROBOT_PYTHON_DIR}")
message(STATUS "================================================================")
message(STATUS "")
