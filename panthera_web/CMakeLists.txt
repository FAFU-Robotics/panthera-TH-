# =============================================================================
#  panthera_web / CMakeLists.txt
#
#  构建 Python 扩展模块 panthera_motor (pybind11), 把同仓库
#  Panthera-HT_SDK/panthera_cpp/motor_example_debug 的 C++ API 暴露给 Python.
#
#  构建 (Windows / VS 2022, Release):
#    cd panthera_web
#    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release ^
#          -Dpybind11_DIR="<pybind11_cmake_dir>"
#    cmake --build build --config Release -j
#
#  pybind11_DIR 用 `python -c "import pybind11; print(pybind11.get_cmake_dir())"`
#  拿到. build.bat 已经自动这样做了.
#
#  产物: build/Release/panthera_motor.cp38-win_amd64.pyd  (后缀依 Python 版本)
#        + serial_cmake.dll
#  这两个文件会被 CMake 复制到 panthera_web/ 根目录, 之后 `import panthera_motor`
#  就直接能 work.
# =============================================================================
cmake_minimum_required(VERSION 3.18)
project(panthera_motor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

if(MSVC)
    add_compile_options(/utf-8 /W3 /bigobj)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# -----------------------------------------------------------------------------
#  pybind11 / Python
# -----------------------------------------------------------------------------
# 优先用环境变量 / -D 指定的 pybind11_DIR; 否则尝试 find_package
find_package(pybind11 CONFIG REQUIRED)

# -----------------------------------------------------------------------------
#  复用 motor_example_debug 的 hightorque_serial_debug 静态库
#  以及它依赖的 serial_cmake
# -----------------------------------------------------------------------------
set(_MEDIR ${CMAKE_CURRENT_SOURCE_DIR}/../Panthera-HT_SDK/panthera_cpp/motor_example_debug)

if(NOT EXISTS ${_MEDIR}/CMakeLists.txt)
    message(FATAL_ERROR
        "未找到 motor_example_debug: ${_MEDIR}\n"
        "请确认仓库结构: <root>/panthera_web/  和  "
        "<root>/Panthera-HT_SDK/panthera_cpp/motor_example_debug/")
endif()

# 直接 add_subdirectory, CMake 会复用其中的 hightorque_serial_debug 目标
add_subdirectory(${_MEDIR} ${CMAKE_BINARY_DIR}/motor_example_debug_build EXCLUDE_FROM_ALL)

# -----------------------------------------------------------------------------
#  pybind11 模块
# -----------------------------------------------------------------------------
pybind11_add_module(panthera_motor
    bindings.cpp
)

target_link_libraries(panthera_motor PRIVATE
    hightorque_serial_debug   # motor_example_debug 提供的静态库
)

target_include_directories(panthera_motor PRIVATE
    ${_MEDIR}/include
)

# -----------------------------------------------------------------------------
#  把产物 (.pyd + serial_cmake.dll) 复制回项目根, 方便 `import panthera_motor`
# -----------------------------------------------------------------------------
add_custom_command(TARGET panthera_motor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:panthera_motor>
        ${CMAKE_CURRENT_SOURCE_DIR}/$<TARGET_FILE_NAME:panthera_motor>
    COMMENT "复制 panthera_motor 模块到 panthera_web/"
)

if(WIN32)
    add_custom_command(TARGET panthera_motor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:serial_cmake>
            ${CMAKE_CURRENT_SOURCE_DIR}/$<TARGET_FILE_NAME:serial_cmake>
        COMMENT "复制 serial_cmake.dll 到 panthera_web/"
    )
endif()

message(STATUS "")
message(STATUS "================================================================")
message(STATUS " panthera_motor (pybind11) 配置完成")
message(STATUS "   pybind11_DIR  = ${pybind11_DIR}")
message(STATUS "   Python        = ${PYTHON_EXECUTABLE} (${PYTHON_VERSION_STRING})")
message(STATUS "   依赖 SDK 路径 = ${_MEDIR}")
message(STATUS "================================================================")
message(STATUS "")
