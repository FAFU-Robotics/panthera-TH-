# =============================================================================
#  motor_example_debug
#
#  调试板 USB 串口协议的 C++ 示例 (1:1 移植自 panthera_python/scripts/motor_example/).
#  复用 SDK 已有的 serial_cmake 库 (位于 ../motor_cpp/third_part/serial_cmake/).
#
#  构建:
#    cmake -S . -B build
#    cmake --build build --config Release
#
#  运行 (Windows):
#    build/Release/11_multi_joint_control.exe
#
#  运行 (Linux):
#    build/11_multi_joint_control
# =============================================================================
cmake_minimum_required(VERSION 3.12)
project(motor_example_debug LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 让所有可执行文件和 DLL 输出到同一目录, 方便 Windows 上找 serial_cmake.dll
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(MSVC)
    add_compile_options(/utf-8 /W3)
    # 让 Windows.h 不要乱定义 min/max 宏 (我们 #define NOMINMAX 已挡, 这里再保险)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# -----------------------------------------------------------------------------
#  复用 SDK 已有的 serial_cmake (跨平台串口库)
# -----------------------------------------------------------------------------
set(_SERIAL_CMAKE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../motor_cpp/third_part/serial_cmake)

if(NOT EXISTS ${_SERIAL_CMAKE_DIR}/CMakeLists.txt)
    message(FATAL_ERROR
        "未找到 serial_cmake: ${_SERIAL_CMAKE_DIR}\n"
        "请确认你在 Panthera-HT_SDK/panthera_cpp/motor_example_debug/ 目录下构建.")
endif()

add_subdirectory(${_SERIAL_CMAKE_DIR} ${CMAKE_BINARY_DIR}/serial_cmake_build)

# -----------------------------------------------------------------------------
#  hightorque_serial 静态库 (协议层)
# -----------------------------------------------------------------------------
add_library(hightorque_serial_debug STATIC
    src/hightorque_serial.cpp
)

target_include_directories(hightorque_serial_debug PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hightorque_serial_debug PUBLIC serial_cmake)

# -----------------------------------------------------------------------------
#  示例可执行程序 (业务层 + main)
# -----------------------------------------------------------------------------
set(_EXAMPLES
    01_motor_get_status
    09_set_zero
    11_multi_joint_control
    12_demo_new_features
    13_test_many
    14_set_limits
)

foreach(_ex IN LISTS _EXAMPLES)
    add_executable(${_ex} src/${_ex}.cpp)
    target_link_libraries(${_ex} PRIVATE hightorque_serial_debug)

    if(WIN32)
        # 把 serial_cmake.dll 复制到 exe 旁边, 双击就能跑
        add_custom_command(TARGET ${_ex} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:serial_cmake>
                $<TARGET_FILE_DIR:${_ex}>
            COMMENT "复制 serial_cmake.dll 到 ${_ex} 目录"
        )
    endif()
endforeach()

# 把示例配置文件 robot.cfg 复制到 exe 同目录, 默认相对路径就能找到
# 11/12/13/14 都需要它
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/robot.cfg)
    foreach(_cfg_user 11_multi_joint_control 12_demo_new_features 13_test_many 14_set_limits)
        add_custom_command(TARGET ${_cfg_user} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/robot.cfg
                $<TARGET_FILE_DIR:${_cfg_user}>/robot.cfg
            COMMENT "复制 robot.cfg 到 ${_cfg_user} 目录"
        )
    endforeach()
endif()
