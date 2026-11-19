#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace upperlimb_robot
{

  // ===================== 系统常量定义 =====================

  // 总轴数
  // 控制层内部数组索引统一使用 0~19
  static constexpr std::size_t AXIS_NUM = 20;

  // 六维力传感器数量
  // 传感器数组索引统一使用 0~3
  static constexpr std::size_t FORCE_SENSOR_NUM = 4;

  // ===================== 统一重力补偿参数 =====================
  //
  // 所有使用 Pinocchio 重力力矩的控制算法都共用这里的参数。
  // 左右臂分别配置，并且每条臂的 7 个主动关节分别配置。
  //
  // 修正公式：
  //   tau_gravity[i] = gain[i] * tau_gravity_model[i] + bias_nm[i]
  //
  // 参数顺序：
  //   左臂：joint1  ~ joint7
  //   右臂：joint11 ~ joint17
  //
  // 注意：bias_nm 使用 Pinocchio 模型方向，右臂最终发送时的整体负号
  // 仍由各控制算法原有的驱动方向映射负责。
  struct GravityCompensationParameters
  {
    std::array<double, 7> gain{};
    std::array<double, 7> bias_nm{};
  };

  extern const GravityCompensationParameters LEFT_ARM_GRAVITY_COMPENSATION;
  extern const GravityCompensationParameters RIGHT_ARM_GRAVITY_COMPENSATION;

  // ===================== 统一关节摩擦补偿参数 =====================
  //
  // 所有扭矩控制算法共用这里的摩擦补偿参数。
  // 左右臂分别配置，并且每条臂的 7 个主动关节分别配置。
  //
  // 补偿公式：
  //   tau_friction[i]
  //       = coulomb_nm[i] * tanh(dq[i] / velocity_smooth_rad_s[i])
  //       + viscous_nm_per_rad_s[i] * dq[i]
  //
  // 参数含义：
  //   coulomb_nm              -> 低速近似恒定的库仑摩擦补偿，单位 Nm；
  //   viscous_nm_per_rad_s    -> 随速度增加的粘性摩擦补偿，单位 Nm/(rad/s)；
  //   velocity_smooth_rad_s   -> 零速度附近的方向平滑范围，单位 rad/s。
  //
  // tau_friction 与实际关节速度方向相同，用于抵消机构摩擦。
  // 右臂最终发送时的整体负号仍由各控制算法原有的驱动方向映射负责。
  struct FrictionCompensationParameters
  {
    std::array<double, 7> coulomb_nm{};
    std::array<double, 7> viscous_nm_per_rad_s{};
    std::array<double, 7> velocity_smooth_rad_s{};
  };

  extern const FrictionCompensationParameters LEFT_ARM_FRICTION_COMPENSATION;
  extern const FrictionCompensationParameters RIGHT_ARM_FRICTION_COMPENSATION;

  // ===================== 全局大臂传感器六维分量反馈增益 =====================
  // 左右大臂共用一组按世界坐标轴定义的6个无量纲增益；手部传感器不使用。
  // 世界坐标约定：+X向左、+Y向后、+Z向上。具体参数在 control_algorithms.cpp 顶部配置。
  extern const double ARM_FORCE_GAIN_WORLD_X;
  extern const double ARM_FORCE_GAIN_WORLD_Y;
  extern const double ARM_FORCE_GAIN_WORLD_Z;
  extern const double ARM_MOMENT_GAIN_WORLD_X;
  extern const double ARM_MOMENT_GAIN_WORLD_Y;
  extern const double ARM_MOMENT_GAIN_WORLD_Z;

  // ===================== 控制模式定义 =====================
  //
  // 说明：
  // 1. 保持保留模式的原有编号不变（0, 1, 2, 3, 4, 5, 6, 8, 9, 13, 14）
  // 2. 已移除本次中期基础实验不需要的旧模式（7, 10, 11, 12, 15, 16, 17, 18）
  //
  enum ControlMode : int
  {
    // 空闲模式（安全停止，默认不输出控制命令）
    MODE_IDLE = 0,

    // 左臂预设位置模式（原样保留）
    MODE_LEFT_PRESET_POSITION = 1,

    // 右臂预设位置模式（原样保留）
    MODE_RIGHT_PRESET_POSITION = 2,

    // 左臂主动导纳模式
    MODE_ACTIVE = 3,

    // 左臂零力拖动模式
    MODE_ZERO_FORCE = 4,

    // 左臂正弦被动模式（PID跟踪）
    MODE_PASSIVE = 5,

    // 左臂正弦被动模式（PD+前馈跟踪）
    MODE_PASSIVE_SINE_PD_FF = 6,

    // 右臂零力拖动模式
    MODE_RIGHT_ZERO_FORCE = 8,

    // 双臂零力拖动模式
    MODE_BILATERAL_ZERO_FORCE = 9,

    // 双侧主动导纳模式
    MODE_BILATERAL_ACTIVE = 13,

    // 右臂主动导纳模式
    MODE_RIGHT_ACTIVE = 14,
  };

  // ===================== 实验参数配置定义 =====================

  // 被动正弦轨迹关节组合选择（SG1~SG3 始终保持进入模式时的起始位不动）
  enum PassiveJointProfile : int
  {
    PASSIVE_JOINTS_COMBINED = 0, // 组合联动：GH1~GH3 + EB1（joint4~joint7）
    PASSIVE_JOINT_J4_ONLY = 1,   // 单关节 GH1（肩外展/内收，joint4）
    PASSIVE_JOINT_J5_ONLY = 2,   // 单关节 GH2（肩内旋/外旋，joint5）
    PASSIVE_JOINT_J6_ONLY = 3,   // 单关节 GH3（肩前屈/后伸，joint6）
    PASSIVE_JOINT_J7_ONLY = 4,   // 单关节 EB1（肘屈曲/伸展，joint7）
  };

  // 被动正弦两档低速选择
  enum PassiveSpeedLevel : int
  {
    PASSIVE_SPEED_SLOW = 0,     // 低速1档（慢速：0.5x 原频，f4~f6=0.05 Hz, f7=0.06 Hz）
    PASSIVE_SPEED_STANDARD = 1, // 低速2档（标准低速：1.0x 原频，f4~f6=0.10 Hz, f7=0.12 Hz）
  };

  // 主动导纳阻尼三档预设
  enum AdmittanceDampingLevel : int
  {
    ADMITTANCE_DAMPING_LOW = 0,      // 低阻尼档（0.70x 基准阻尼）
    ADMITTANCE_DAMPING_STANDARD = 1, // 标准阻尼档（1.00x 基准阻尼，原实机参数）
    ADMITTANCE_DAMPING_HIGH = 2,     // 高阻尼档（1.50x 基准阻尼）
  };

  // 实验运行参数（由控制节点在 MODE_IDLE 安全互锁下更新）
  struct ExperimentRuntimeParams
  {
    int passive_joint_profile{PASSIVE_JOINTS_COMBINED};
    int passive_speed_level{PASSIVE_SPEED_STANDARD};
    int admittance_damping_level{ADMITTANCE_DAMPING_STANDARD};
  };

  // 设置/读取当前实验运行参数（线程安全由调用方在控制线程中保证）
  void setExperimentRuntimeParams(const ExperimentRuntimeParams &params);
  ExperimentRuntimeParams getExperimentRuntimeParams();

  // ===================== 电机状态缓存 =====================
  //
  // 对外统一使用真实物理量：
  // position : deg
  // velocity : deg/s
  // torque   : Nm
  //
  struct MotorStateCache
  {
    std::array<double, AXIS_NUM> position{};
    std::array<double, AXIS_NUM> velocity{};
    std::array<double, AXIS_NUM> torque{};
    std::array<bool, AXIS_NUM> enable{};
  };

  // ===================== 单个六维力传感器 =====================
  //
  // 力单位：N，力矩单位：Nm
  //
  struct ForceSensorData
  {
    double fx{0.0};
    double fy{0.0};
    double fz{0.0};
    double mx{0.0};
    double my{0.0};
    double mz{0.0};
  };

  // ===================== 传感器索引定义 =====================
  enum ForceSensorIndex : std::size_t
  {
    SENSOR_RIGHT_HAND = 0,
    SENSOR_RIGHT_ARM = 1,
    SENSOR_LEFT_HAND = 2,
    SENSOR_LEFT_ARM = 3,
  };

  // ===================== 传感器状态缓存 =====================
  struct SensorStateCache
  {
    std::array<ForceSensorData, FORCE_SENSOR_NUM> sensor{};
    std::array<ForceSensorData, FORCE_SENSOR_NUM> bias{};
    bool valid{false};
  };

  // ===================== 控制输出缓存 =====================
  //
  // “算法层 -> 执行层”统一输出接口，数组索引 0~19
  //
  struct ControlOutputCache
  {
    std::array<double, AXIS_NUM> target_position{};
    std::array<double, AXIS_NUM> target_velocity{};
    std::array<double, AXIS_NUM> target_torque{};
    std::array<double, AXIS_NUM> target_acc{};
    std::array<double, AXIS_NUM> target_dec{};
    std::array<short, AXIS_NUM> target_smooth_time{};

    std::array<bool, AXIS_NUM> use_position{};
    std::array<bool, AXIS_NUM> use_velocity{};
    std::array<bool, AXIS_NUM> use_torque{};

    bool valid{false};
  };

  // ===================== 算法中间量（用于同频异步 CSV 记录） =====================
  //
  // 仅用于日志记录，不改变控制输出。
  // 数组索引与控制层索引 0~19 一致：
  //   右臂 joint11~joint17 -> 0~6
  //   左臂 joint1~joint7   -> 10~16
  //
  struct AlgorithmDiagData
  {
    // 原始轨迹参考位置（deg），被动正弦为 q_ref，主动导纳为 q_adm
    std::array<double, AXIS_NUM> qd_deg{};

    // 导纳内部参考状态（deg / deg/s）
    std::array<double, AXIS_NUM> qa_deg{};
    std::array<double, AXIS_NUM> dqa_deg_s{};

    // 映射交互力矩 J^T F（Nm，Pinocchio 模型方向）
    std::array<double, AXIS_NUM> tau_int_nm{};

    // 重力补偿项（Nm，含逐关节 gain + bias）
    std::array<double, AXIS_NUM> tau_grav_nm{};

    // 摩擦补偿项（Nm，含库仑 + 粘性）
    std::array<double, AXIS_NUM> tau_fric_nm{};

    // 单关节位置或力矩触发限幅标志
    std::array<bool, AXIS_NUM> saturated{};

    // 经去零偏、符号修正、死区、世界轴增益与限幅处理后的六维力（传感器 LOCAL 系）
    std::array<ForceSensorData, FORCE_SENSOR_NUM> proc_ft{};

    // 任意六维力分量触发限幅标志
    bool ft_saturated{false};
  };

  // ===================== 单位换算接口 =====================
  // 传入真实物理轴号 axis（1~20）
  double jointDegToPulse(int axis, double joint_deg);
  double pulseToJointDeg(int axis, double pulse);

  double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec);
  double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms);

  double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2);
  double pulsePerMs2ToJointDegPerSec2(int axis, double pulse_per_ms2);

  double torqueRawToNm(int axis, double raw);
  double torqueNmToRaw(int axis, double torque_nm);

  // ===================== 模式状态重置接口 =====================
  void resetActiveMode();
  void resetRightActiveMode();
  void resetBilateralActiveMode();
  void resetPassiveMode();
  void resetPassivePDFFMode();

  // ===================== 控制算法接口 =====================

  // 空闲模式
  void runIdle(const MotorStateCache &state, ControlOutputCache &output);

  // 预设位姿模式（严格原样保留）
  void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output);
  void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output);

  // 零力拖动模式
  void runZeroForceMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  void runRightZeroForceMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  void runBilateralZeroForceMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  // 主动导纳模式（使用实际控制周期 dt_s）
  void runActiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  void runRightActiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  void runBilateralActiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  // 被动正弦跟踪模式（使用实际控制周期 dt_s）
  void runPassiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

  void runPassivePDFFMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag = nullptr);

} // namespace upperlimb_robot
