# panthera_web — Panthera-HT 调试板 Web 控制

把同仓库 `Panthera-HT_SDK/panthera_cpp/motor_example_debug` 的 C++ API
通过 **pybind11** 封装成 Python 模块 `panthera_motor`，再用 **Flask** 提供
REST 接口，配套一个**单页网页**，对每个电机做独立位置控制。

```
浏览器  ──HTTP──▶  Flask (app.py)  ──pybind11──▶  HightorqueSerial (C++)  ──USB串口──▶  调试板
```
启动服务的事你自己来（在你的终端里跑）：

cd F:\1LWP\Python\Panthera_Workspace\panthera_web
python app.py --port 5050
然后浏览器开 http://localhost:5050/ → 点 连接。

## 目录结构

```
panthera_web/
├── bindings.cpp                 # pybind11 绑定 (HightorqueSerial / RobotConfig / ...)
├── CMakeLists.txt               # 构建 panthera_motor.pyd, 复用 motor_example_debug
├── build.bat                    # Windows 一键构建脚本
├── app.py                       # Flask 后端 + REST API
├── requirements.txt
├── templates/
│   └── index.html               # 单页 UI
├── static/
│   ├── app.js                   # 前端逻辑 (vanilla JS, 无构建)
│   └── style.css                # 深色主题样式
├── panthera_motor.cp38-win_amd64.pyd  # 构建产物 (后缀依 Python 版本)
└── serial_cmake.dll             # 构建产物 (运行时依赖)
```

## 依赖

- **C++ 侧**: CMake 3.18+ / Visual Studio 2019+ (含 C++ 桌面开发 workload)
- **Python 侧**: Python 3.7+, `pybind11`, `flask`, `flask-cors`

```bash
pip install -r requirements.txt
```

## 构建 pybind11 模块

```bat
cd panthera_web
build.bat                   # Release 构建 (推荐)
:: 或者手工:
:: cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
::       -Dpybind11_DIR="<pip 装的 pybind11 路径>"
:: cmake --build build --config Release -j
```

成功后 `panthera_web/` 下会多出两个文件：
- `panthera_motor.cpXY-win_amd64.pyd` — 主模块
- `serial_cmake.dll` — 跨平台串口库 (运行时必需)

可以用以下命令快速验证导入：

```bat
python -c "import panthera_motor as pm; print(pm.list_serial_ports())"
```

## 启动 Web 服务

```bat
cd panthera_web
python app.py
:: 默认监听 0.0.0.0:5000
:: 用浏览器打开 http://localhost:5000/
```

> **PowerShell 用户注意**: 调用本目录下的 `.bat` 脚本要带 `.\` 前缀，
> 不能直接写 `build.bat`。即:
> ```powershell
> .\build.bat
> ```

> **conda 多环境用户注意**: `.pyd` 是按 Python 版本 ABI 编译的 (例如
> `cp310-win_amd64`)，只能被同版本的 Python 加载。所以 **先激活目标
> conda 环境，再 `pip install -r requirements.txt`，再 `.\build.bat`**。

可选参数：
```
--host 0.0.0.0           监听地址
--port 5000              端口
--cfg path/to/robot.cfg  默认配置文件 (POST /api/connect 不传 cfg_path 时用)
--debug                  Flask debug 模式
```

## Web UI 用法

1. 打开 http://localhost:5000/
2. 顶部 "串口" 下拉里挑选 `★` 标记的候选 (调试板会被自动识别)
3. 点击 **连接** —— 配置文件 (`robot.cfg`) 自动加载，所有电机的卡片就出来了
4. 每个电机卡片：
   - 实时显示 `pos / vel / trq / mode / fault / 软限位 flag`
   - 滑块或输入框设目标位置 (单位由顶部 "位置单位" 决定)
   - **▶ 移动** / **■ 停止** / **⊠ 刹车** / **i 版本** / **设零** / **◎ 0**
5. 顶部全局按钮：**全部回零** / **全部停止** / **CAN 状态**

UI 用 `setInterval` 每 120 ms 刷新一次状态 (`/api/states` 走后端 cache，不会卡串口)。

## REST API 速查

| Method | Path | 说明 |
|---|---|---|
| GET    | `/api/ports`                       | 列出串口 (含 VID/PID 候选过滤标记) |
| POST   | `/api/connect`                     | body: `{port, baudrate?, cfg_path?, poll_hz?}` |
| POST   | `/api/disconnect`                  | 断开串口 |
| GET    | `/api/status`                      | 当前连接状态 / 配置 / 电机ids / stats |
| GET    | `/api/states`                      | 所有电机最新状态 (cache) |
| GET    | `/api/state/<id>`                  | 单个电机实时读 (走串口) |
| POST   | `/api/motor/<id>/move`             | body: `{pos, vel_max_rps?, acc_rpss?, unit?}` |
| POST   | `/api/motor/<id>/set_velocity`     | body: `{vel_rps}` |
| POST   | `/api/motor/<id>/stop`             | 停止 (PWM off) |
| POST   | `/api/motor/<id>/brake`            | 刹车 |
| POST   | `/api/motor/<id>/reset_zero`       | body: `{confirm: true}` 把当前位置写为零点 |
| POST   | `/api/motor/<id>/save_config`      | 保存当前配置到电机 flash |
| POST   | `/api/motor/<id>/motor_reset`      | body: `{confirm: true}` 软复位电机 |
| GET    | `/api/motor/<id>/version`          | 读固件版本 |
| POST   | `/api/stop_all`                    | 全部停止 |
| POST   | `/api/home_all`                    | body: `{vel_max_rps?}` 全部回零 (一帧广播) |
| POST   | `/api/move_many`                   | body: `{vel_max_rps?, unit?, targets:{id:pos}}` 批量移动 |
| GET    | `/api/can_status`                  | 调试板 CAN 总线状态 (Ok/ErrorWarning/...) |

返回都是 JSON，公共字段：`{ok: bool, message: str, ...}`。

## 安全提醒

- 第一次跑先把 `robot.cfg` 里的 `motor_ids` 改成单关节 (例如 `1`)，确认 OK 再加。
- `limits.4 ~ limits.7` 是保守初值，**必须按你机械臂的实际行程修改 `robot.cfg`**。
- "停止" 只是关 PWM，**重力下电机仍会下坠**——肩 / 肘等承重关节请额外注意。
- 调试期间手边备好"拔 USB"的能力 (Web 里点 "全部停止" 走 USB；如果通讯卡死，物理拔线最快)。
- "设零" / "motor_reset" 都需要 `confirm=true`，UI 已加二次确认弹窗。

## 排错

- **PowerShell 报 `无法将"build.bat"项识别为 cmdlet`**
  → PowerShell 默认不从当前目录运行命令，要写 `.\build.bat`；或者改用 cmd。

- **`ModuleNotFoundError: No module named 'flask'`**
  → 你激活的 conda 环境里没装依赖。务必**先激活目标环境再装**：
  ```
  conda activate panthera
  pip install -r requirements.txt
  ```
  注意 `pip install` 只装到当前激活环境；在 `(base)` 里装的不会被 `(panthera)` 看到。

- **重新编译时报 `Error copying file ... serial_cmake.dll`** (MSB3073)
  → 之前启动的 Flask 进程还活着，占用了 `serial_cmake.dll`。
  解决：先 `Stop-Process -Name python -Force` 杀掉所有 python 进程，再 `.\build.bat`。

- **换了 Python 版本之后 import 报 ImportError**
  → `.pyd` 文件名包含 Python ABI 后缀 (`cp38-win_amd64` / `cp310-win_amd64`)，
  Python 只会加载匹配自己版本的那个。换环境后请用对应环境重新跑 `.\build.bat`。

- **`import panthera_motor` 报 `ImportError: DLL load failed`**
  → 检查 `serial_cmake.dll` 是否在 `panthera_web/` 目录下；
  C++ 运行时没装的话装一下 VC++ 2015-2022 Redist。

- **连接失败: 无法打开串口 COMxx**
  → 端口名错了 / 调试板 USB 没插 / 被其它程序占用 (有些"串口助手"会一直占着)。

- **连接成功但所有电机 `mode 0 / —`**
  → 调试板和电机之间 CAN 没通：检查 24V 电源 / 终端电阻 / motor_ids 是否真的对应。

- **滑块超出范围**
  → 限位是按"圈"存的，UI 上单位切换会自动换算；超界后驱动层会自动 clamp，但 UI 上仍要看 `pos_limit_flag` 标志。
