// =============================================================================
//  fafu_robot_controller.hpp
//
//  原生 C++ 机械臂 SDK 主头文件.
//
//  本 SDK 是 fafu_robot_sdk/fafu_robot_python/fafu_robot_controller.py 的
//  C++ 同构实现, 借鉴 Panthera-HT_SDK/.../panthera/Panthera.cpp 的代码风格.
//
//  关注点:
//    - **关节空间** 控制 (move_j / 速度 / 力矩) 与夹爪控制 (含力控 grasp).
//    - **不** 包含笛卡尔运动 (move_p / move_l), URDF / Pinocchio / IK 等.
//      若需要笛卡尔, 建议在上层用 Python (fafu_robot_python) 配合 wrs 库做规划,
//      然后回传关节路径给本 SDK.
//
//  典型用法:
//
//      #include "fafu/fafu_robot_controller.hpp"
//      using namespace fafu_robot;
//
//      FafuRobotController::Options opts;
//      opts.has_gripper      = true;
//      opts.gripper_motor_id = 7;
//
//      FafuRobotController arm("robot.cfg", opts);     // 自动 enable + start polling
//      arm.move_j({0, 0, 0, 0, 0, 0});                  // 6 关节归零, 阻塞 S-curve
//      arm.open_gripper();
//      auto r = arm.grasp({.force_threshold = 500});    // 力控抓取
//      if (r.grasped) std::cout << "got it!\n";
//      // 析构时自动 close_connection (gripper=brake, joints=stop)
//
//  线程模型:
//    - 构造完成后内部有 1 个 RX 线程 (异步串口接收) + 1 个 polling 线程 (50Hz 缓存
//      状态), 由底层 hightorque::HightorqueSerial 管理.
//    - 对外 API 不保证线程安全; 同一实例不要在多个线程并发调用 move_j / grasp 等.
//
//  错误处理:
//    - 大部分方法在参数错 / 通信失败时 *返回 bool false* 或抛 std::runtime_error
//      (沿用 Panthera::* 的"非致命错误打印 + 返回 false"风格).
//    - 仅"启动失败 / 配置缺失"会在构造期抛异常 (std::runtime_error).
// =============================================================================
#pragma once

#include "hightorque_serial.hpp"        // hightorque::HightorqueSerial, MotorState, ...
#include "robot_config.hpp"             // hightorque::RobotConfig

#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace fafu_robot {

// ============================================================================
//  常量 / 枚举
// ============================================================================

// 电机模式 (与底层调试板协议一致, 对应 hightorque_serial.set_motor_mode 的 mode 参数)
inline constexpr uint8_t MODE_POSITION = 0x0A;   // 位置控制 / 保持
inline constexpr uint8_t MODE_BRAKE    = 0x0F;   // 短路刹车 (耗能小, 抗动)
inline constexpr uint8_t MODE_STOP     = 0x00;   // PWM 关闭, 可手推

// close_connection() 时如何对待电机
enum class ReleaseMode {
    Stop,   // 0x00, PWM off, 自由可手推                 (默认: joints)
    Brake,  // 0x0F, 短路刹车, 抗动但不出力              (默认: gripper)
    Hold,   // 0x0A, 继续维持上一次位置 (耗电, 不要久挂)
};

// ============================================================================
//  力控抓取结果
// ============================================================================

// Python 侧 fafu_robot_controller.GraspResult 的 1:1 对应.
struct GraspResult {
    // 是否认为"抓到物体".
    //   true:  detected_object_force / detected_object_stall
    //   false: reached_target / no_movement / timeout
    bool        grasped = false;

    // 触发停止的原因, 见下面 5 种字符串:
    //   "detected_object_force"  |torque| >= effort_threshold
    //   "detected_object_stall"  夹爪闭合 >= min_close_deg 后又停滞
    //   "reached_target"         到达 target_angle (无障碍)
    //   "no_movement"            停滞但闭合 < min_close_deg
    //   "timeout"                超时
    std::string reason;

    // 终止时的夹爪角度 (弧度)
    double      angle_rad      = 0.0;

    // 自调用开始累计闭合的角度 (度数, >= 0)
    double      closed_deg     = 0.0;

    // 期间观测到的最大 |torque| (原始 int16)
    int         peak_torque_raw = 0;

    // 本次调用墙钟时长 (秒)
    double      duration_s     = 0.0;

    std::string to_string() const;
};

// ============================================================================
//  FafuRobotController 主控制器
// ============================================================================

class FafuRobotController {
public:
    // ----------------------------------------------------------------------
    //  构造 / 析构
    // ----------------------------------------------------------------------
    struct Options {
        // 串口端口名, "" 或 "auto" 表示自动枚举 (find_likely_debug_boards).
        // 留空则用 cfg.port.
        std::string port;

        // 串口波特率; 0 表示用 cfg.baudrate (一般 4Mbps).
        uint32_t    baudrate         = 0;

        // 是否把 cfg.motor_ids 里的 gripper_motor_id 视为夹爪 (从关节空间命令里
        // 排除). 默认 false.
        bool        has_gripper      = false;

        // 当 has_gripper=true 时必须提供, 且必须出现在 cfg.motor_ids 里.
        int         gripper_motor_id = 0;

        // 构造完后是否自动把每个电机切到 MODE_POSITION (0x0A). 默认 true.
        bool        auto_enable      = true;

        // 是否启动 50Hz 后台 state polling 线程 (供 get_joint_values 拿缓存).
        // 默认 true.
        bool        auto_polling     = true;

        // 是否启用异步 RX 模式 (推荐). std::nullopt 表示沿用 cfg.use_async_rx.
        std::optional<bool> async_rx = std::nullopt;
    };

    // 主构造函数; 内部会:
    //   1. RobotConfig::load(cfg_path)
    //   2. 选串口 -> HightorqueSerial
    //   3. apply_limits_to(ht_)
    //   4. precheck (每个电机 read_motor_state 一次)
    //   5. opts.auto_enable: enable() 所有电机
    //   6. opts.async_rx:    enable_async_rx + 预热 RX cache
    //   7. opts.auto_polling: start_state_polling
    //
    // 抛: std::runtime_error (配置错误 / 串口打不开 / 电机不响应)
    explicit FafuRobotController(const std::string& cfg_path, const Options& opts = {});

    // 析构会自动调 close_connection({Stop, Brake}) (跟 Python __exit__ 一致).
    ~FafuRobotController();

    FafuRobotController(const FafuRobotController&)            = delete;
    FafuRobotController& operator=(const FafuRobotController&) = delete;

    // ----------------------------------------------------------------------
    //  只读属性
    // ----------------------------------------------------------------------
    const hightorque::RobotConfig& cfg() const  { return cfg_; }
    const std::string&             port() const { return port_; }
    uint32_t                       baudrate() const { return baudrate_; }
    const std::vector<int>&        joint_motor_ids() const { return joint_motor_ids_; }
    const std::vector<int>&        all_motor_ids() const   { return cfg_.motor_ids; }
    int                            num_joints() const      { return static_cast<int>(joint_motor_ids_.size()); }
    bool                           has_gripper() const     { return has_gripper_; }
    int                            gripper_motor_id() const{ return gripper_motor_id_; }

    // Escape hatch: 拿到底层 driver. 慎用, 不要在多线程里乱调.
    hightorque::HightorqueSerial& driver() { return *ht_; }

    // 当前是否所有电机都处于 MODE_POSITION
    bool is_enabled();

    // ----------------------------------------------------------------------
    //  电源管理
    // ----------------------------------------------------------------------
    void enable();    // 全部 -> MODE_POSITION, 抛异常表示有电机不响应
    void disable();   // 全部 -> MODE_STOP  (自由手推)
    void brake();     // 全部 -> MODE_BRAKE (短路刹车)

    // ----------------------------------------------------------------------
    //  关节空间运动
    // ----------------------------------------------------------------------
    struct MoveOpts {
        bool   is_radians = true;     // joint_angles 单位 (true=弧度, false=度)
        int    speed      = 50;       // 速度百分比 (1..100), 实际 v_avg = speed/100 * 0.5 turns/s
        bool   block      = true;     // true=阻塞 + S-curve, false=单帧 send 立即返回
        double tolerance  = 0.01;     // 阻塞 fallback 的位置 tolerance (rad 或 deg, 跟 is_radians)
    };

    // 把所有 *关节* 电机驱动到 joint_angles. 夹爪 (若 has_gripper) 保持当前位置.
    //   joint_angles.size() 必须 == num_joints()
    // 阻塞模式下走 S-curve + run_control_loop, 至少 0.3s; 非阻塞模式下一帧 set_many_pos_vel_tqe.
    bool move_j(const std::vector<double>& joint_angles, const MoveOpts& opts = {});

    // 所有关节归零 (joint_angles = {0, 0, ...}).
    bool go_home(int speed = 20, bool block = true);

    // ----------------------------------------------------------------------
    //  状态读取 (从 polling 缓存; prefer_cache=false 则现读)
    // ----------------------------------------------------------------------
    // 返回所有关节电机的角度 (弧度)
    std::vector<double> get_joint_values(bool prefer_cache = true);

    // 返回所有关节电机的角速度 (rad/s)
    std::vector<double> get_joint_velocities(bool prefer_cache = true);

    // 返回 motor_id -> MotorState 映射 (所有电机, 含夹爪)
    std::map<int, hightorque::MotorState> get_motor_states(bool prefer_cache = true);

    // ----------------------------------------------------------------------
    //  夹爪控制 (仅 has_gripper=true 时可用)
    // ----------------------------------------------------------------------
    struct GripperOpts {
        // 硬件层力矩上限 (raw int16). std::nullopt = 走 set_pos_vel_acc (无 effort 参数).
        std::optional<int> effort = std::nullopt;

        bool   is_radians      = true;
        double vel             = 0.3;    // turns/s
        double acc             = 0.5;    // turns/s^2 (effort 提供时无效)
        bool   block           = true;
        double timeout_s       = 8.0;
        double tolerance_deg   = 1.5;

        // 软力控提前停的阈值 (raw int16 |torque|). std::nullopt = 不做.
        // 用 grasp() 通常用 grasp 自带的 force_threshold, 不需要在这里设.
        std::optional<int> effort_threshold = std::nullopt;
    };

    // 驱动夹爪到指定 angle (弧度 / 度), Piper 风格 (angle + effort).
    // 如果 block=true 且 effort_threshold 非空, 返回的 GraspResult 描述结果.
    // 否则返回 std::nullopt (兼容 Piper 的 void 返回).
    std::optional<GraspResult> gripper_control(double angle,
                                               const GripperOpts& opts = {});

    // 把夹爪 *打开* (= 朝软限位上限走). angle=nullopt 时直接走上限.
    // 注意: Fafu 夹爪是 "更开 = 角度更大 (less negative)".
    bool open_gripper(std::optional<double> angle = std::nullopt,
                      const GripperOpts& opts = {});

    // 把夹爪 *关闭* (= 朝软限位下限走).
    bool close_gripper(std::optional<double> angle = std::nullopt,
                       const GripperOpts& opts = {});

    struct GraspOpts {
        // 关闭目标角度. std::nullopt 时用软限位下限.
        std::optional<double> target_angle = std::nullopt;
        bool                  is_radians   = true;

        // *Python 侧* 力检测阈值 (raw int16). 见 GraspResult::reason 触发条件.
        int                   force_threshold = 500;

        // 硬件层力矩上限 (raw int16). std::nullopt 表示不限.
        std::optional<int>    effort       = std::nullopt;

        double                vel          = 0.15;   // 闭合速度 (turns/s, 默认比 open 慢)
        double                acc          = 0.5;
        double                timeout_s    = 5.0;
        double                min_close_deg = 3.0;   // < 此值的停滞算 no_movement
    };

    // 力控抓取: 朝关闭方向走, 一旦 |torque| >= force_threshold 立即返回 grasped=true.
    // 抛 std::runtime_error: 控制器无夹爪.
    GraspResult grasp(const GraspOpts& opts = {});

    // grasp 的对称操作 — 把夹爪打开释放物体. 内部就是 open_gripper().
    void release(const GripperOpts& opts = {});

    // 夹爪当前状态. 抛: 没夹爪 / 没收到回执.
    hightorque::MotorState get_gripper_state();

    // ----------------------------------------------------------------------
    //  软限位 (运行时配置)
    // ----------------------------------------------------------------------
    // 给某个电机加 / 改软限位 (单位由 unit 决定; 默认 Turns).
    void set_limit(int motor_id, double lo, double hi,
                   hightorque::PosUnit unit = hightorque::PosUnit::Turns);

    // 返回 (lo, hi) 圈, 或 nullopt 表示未配置.
    std::optional<std::pair<double, double>> get_limit(int motor_id) const;

    void disable_limit(int motor_id);
    void clear_limits();

    // ----------------------------------------------------------------------
    //  急停 / 状态
    // ----------------------------------------------------------------------
    // 所有电机立即切 STOP (PWM off). 一般跟 resume() 配对.
    void emergency_stop();

    // 从急停恢复 — 把所有电机切回 MODE_POSITION.
    void resume();

    // 取调试板原始 `can status` 字符串
    std::string get_status();

    // 解析后的 CanStatus 结构
    hightorque::CanStatus get_can_status();

    // 把某个电机的当前位置标定为 0. confirm=false 时只打印警告, 不执行.
    // 该操作是硬件级永久写入, 误用会让其他参考点失准, 所以默认必须显式 confirm=true.
    void reset_zero(int motor_id, bool confirm = false);

    // ----------------------------------------------------------------------
    //  关闭连接
    //
    //  析构里也会调一次, 但显式调用允许定制 joint_release / gripper_release 策略.
    // ----------------------------------------------------------------------
    void close_connection(ReleaseMode joint_release   = ReleaseMode::Stop,
                          ReleaseMode gripper_release = ReleaseMode::Brake);

private:
    // ---- helpers ----
    static double rad_to_turns_(double rad);
    static double turns_to_rad_(double turns);
    static int    clamp_speed_(int speed);
    static std::string release_mode_name_(ReleaseMode m);

    // 从 cfg.port 解析实际端口 (auto / 显式 / 缺省).
    std::string pick_serial_port_(const std::string& preferred);

    // 给每个电机 read_motor_state 一次, 失败抛异常.
    void precheck_communication_();

    // 一次性把所有电机切到 mode, 带重试. label 用于打印.
    bool switch_mode_all_(uint8_t mode, const char* label, int max_retry);

    // 入参做长度 / NaN 校验, 返回归一到 turns 的 joint_angles (用于内部 set_many_*).
    std::vector<double> validate_joint_angles_(const std::vector<double>& angles,
                                               bool is_radians);

    // 读取一组 motor_id 的 MotorState; prefer_cache=true 优先 cache.
    std::map<int, hightorque::MotorState> read_states_(const std::vector<int>& ids,
                                                       bool prefer_cache);

    // 阻塞等夹爪到 target_turns / stall / force / timeout. 返回 GraspResult.
    GraspResult wait_until_gripper_done_(double target_turns,
                                         double timeout_s,
                                         std::optional<double> tolerance_turns,
                                         std::optional<int> effort_threshold,
                                         std::optional<double> min_progress_turns);

    GraspResult make_grasp_result_(const std::string& reason, bool grasped,
                                   double last_pos_turns, double start_pos_turns,
                                   int peak_torque, double duration_s);

    // 构造 set_many_pos_vel_tqe 用的 cmd 列表 — targets 里没指定的 motor 用 hold pos.
    std::vector<hightorque::HightorqueSerial::ManyMotorCmd>
        build_many_cmds_holding_others_(const std::map<int, double>& targets_turns,
                                        double vel_rps);

    // 阻塞 S-curve 关节运动. (mirror of Python _move_scurve.)
    void move_scurve_(const std::map<int, double>& targets_turns, int speed_pct);

    // 软限位读取 (gripper 用). 返回 (lo, hi) 圈 / nullopt.
    std::optional<std::pair<double, double>> gripper_limit_turns_() const;

    // 获取夹爪当前 turns (cache 优先).
    std::optional<double> gripper_current_turns_();

    // ---- 数据 ----
    hightorque::RobotConfig                       cfg_;
    std::string                                   cfg_path_;
    std::unique_ptr<hightorque::HightorqueSerial> ht_;

    std::string                                   port_;
    uint32_t                                      baudrate_ = 0;

    bool                                          has_gripper_      = false;
    int                                           gripper_motor_id_ = 0;
    std::vector<int>                              joint_motor_ids_; // = motor_ids - {gripper}

    bool                                          owns_polling_ = false;
    bool                                          owns_rx_      = false;

    // S-curve 调参 (与 Python 同步)
    static constexpr double VEL_AVG_MAX_TPS_ = 0.5;
    static constexpr double DT_MIN_S_        = 0.3;
    static constexpr int    SETTLE_MS_       = 300;

    // 夹爪闭环阈值 (与 Python 同步)
    static constexpr double GRIPPER_TOLERANCE_TURNS_   = 0.005;   // ~1.8 deg
    static constexpr double GRIPPER_STALL_VEL_TPS_     = 0.005;   // < 1.8 deg/s
    static constexpr double GRIPPER_STALL_PATIENCE_S_  = 0.3;
    static constexpr double GRIPPER_MIN_PROGRESS_TURNS_ = 0.008;  // ~2.9 deg
};

} // namespace fafu_robot
