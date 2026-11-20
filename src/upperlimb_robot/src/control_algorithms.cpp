#include "upperlimb_robot/control_algorithms.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/force.hpp>

namespace upperlimb_robot
{

  // ===================== 全局统一重力补偿参数 =====================
  //
  // 所有控制模式共用这两套参数。
  // 左臂顺序：joint1~joint7（基于 2026-11-20 140431 复测微调 SG1 重力欠补，其余关节已定稿）
  const GravityCompensationParameters LEFT_ARM_GRAVITY_COMPENSATION{
      {1.37, 1.05, 1.00, 1.25, 1.14, 1.06, 1.00},
      {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00}};

  // 右臂顺序：joint11~joint17（待右臂零力实测后再针对性精标）
  const GravityCompensationParameters RIGHT_ARM_GRAVITY_COMPENSATION{
      {1.25, 1.05, 1.00, 1.00, 1.00, 1.00, 1.00},
      {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00}};

  // ===================== 全局统一关节摩擦补偿参数 =====================
  //
  // 左臂顺序：joint1~joint7
  // 2026-11-20 140431 复测：SG3/GH1-3/EB1 已轻盈稳定；SG1 实测总库仑 7.72 Nm、总粘性 4.10 Nm*s/rad，
  // 现将 SG1 库仑提升至 3.60 Nm (46.6%)、粘性提升至 2.00 Nm*s/rad (48.8%)，仍严格 <50% 无源稳定。
  const FrictionCompensationParameters LEFT_ARM_FRICTION_COMPENSATION{
      {3.60, 0.35, 1.30, 1.65, 0.95, 0.75, 0.22},
      {2.00, 0.40, 0.90, 0.90, 0.70, 0.80, 0.50},
      {0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.06}};

  // 右臂顺序：joint11~joint17
  const FrictionCompensationParameters RIGHT_ARM_FRICTION_COMPENSATION{
      {0.22, 0.20, 0.15, 0.15, 0.10, 0.10, 0.08},
      {0.40, 0.40, 0.40, 0.40, 0.40, 0.40, 0.40},
      {0.06, 0.06, 0.03, 0.03, 0.03, 0.03, 0.03}};

  // ===================== 全局统一大臂传感器六维分量反馈增益 =====================
  // 世界坐标：+X向左，+Y向后，+Z向上。
  // local Fx/Mx -> world Z 增益；local Fy/My -> world Y 增益；local Fz/Mz -> world X 增益。
  constexpr double ARM_FORCE_GAIN_WORLD_X = 1.20;  // 世界X方向力，映射到传感器局部Fz
  constexpr double ARM_FORCE_GAIN_WORLD_Y = 1.00;  // 世界Y方向力，映射到传感器局部Fy
  constexpr double ARM_FORCE_GAIN_WORLD_Z = 1.00;  // 世界Z方向力，映射到传感器局部Fx
  constexpr double ARM_MOMENT_GAIN_WORLD_X = 1.00; // 世界X方向力矩，映射到传感器局部Mz
  constexpr double ARM_MOMENT_GAIN_WORLD_Y = 1.00; // 世界Y方向力矩，映射到传感器局部My
  constexpr double ARM_MOMENT_GAIN_WORLD_Z = 1.00; // 世界Z方向力矩，映射到传感器局部Mx

  static_assert(ARM_FORCE_GAIN_WORLD_X >= 0.0 &&
                    ARM_FORCE_GAIN_WORLD_Y >= 0.0 &&
                    ARM_FORCE_GAIN_WORLD_Z >= 0.0 &&
                    ARM_MOMENT_GAIN_WORLD_X >= 0.0 &&
                    ARM_MOMENT_GAIN_WORLD_Y >= 0.0 &&
                    ARM_MOMENT_GAIN_WORLD_Z >= 0.0,
                "Upper-arm sensor feedback gains must be nonnegative");

  namespace
  {

    // ===================== 当前实验运行参数 =====================
    // 仅由控制节点在 MODE_IDLE 安全互锁下更新。
    ExperimentRuntimeParams g_exp_params{};

    // ===================== 控制步长安全限幅 =====================
    // 防止异常调度或首帧导致 dt 过小/过大，保证 100 Hz / 50 Hz 下积分与微分尺度准确。
    inline double clampControlDt(double dt_s, double fallback_dt_s = 0.01)
    {
      if (!std::isfinite(dt_s) || dt_s <= 0.0)
      {
        return fallback_dt_s;
      }
      if (dt_s < 0.002)
      {
        return 0.002;
      }
      if (dt_s > 0.05)
      {
        return 0.05;
      }
      return dt_s;
    }

    // 主动导纳三档预设阻尼比例
    inline double getAdmittanceDampingScale(int level)
    {
      switch (level)
      {
      case ADMITTANCE_DAMPING_LOW:
        return 0.70;
      case ADMITTANCE_DAMPING_HIGH:
        return 1.50;
      case ADMITTANCE_DAMPING_STANDARD:
      default:
        return 1.00;
      }
    }

    // 被动正弦两档低速频率比例
    inline double getPassiveSpeedScale(int level)
    {
      if (level == PASSIVE_SPEED_SLOW)
      {
        return 0.50;
      }
      return 1.00;
    }

    // 按统一世界坐标轴的6个增益，缩放“大臂传感器局部坐标系”中的 wrench。
    inline void applyUpperArmSensorWorldAxisGains(
        double &fx, double &fy, double &fz,
        double &mx, double &my, double &mz)
    {
      fx *= ARM_FORCE_GAIN_WORLD_Z;
      fy *= ARM_FORCE_GAIN_WORLD_Y;
      fz *= ARM_FORCE_GAIN_WORLD_X;
      mx *= ARM_MOMENT_GAIN_WORLD_Z;
      my *= ARM_MOMENT_GAIN_WORLD_Y;
      mz *= ARM_MOMENT_GAIN_WORLD_X;
    }

    inline void applyUpperArmSensorWorldAxisGains(ForceSensorData &w)
    {
      applyUpperArmSensorWorldAxisGains(
          w.fx, w.fy, w.fz, w.mx, w.my, w.mz);
    }

    inline bool clampAbsWithFlag(double &x, double lim)
    {
      if (x > lim)
      {
        x = lim;
        return true;
      }
      if (x < -lim)
      {
        x = -lim;
        return true;
      }
      return false;
    }

    inline bool clampBetweenWithFlag(double &x, double lo, double hi)
    {
      if (x < lo)
      {
        x = lo;
        return true;
      }
      if (x > hi)
      {
        x = hi;
        return true;
      }
      return false;
    }

    inline double clampAbs(double x, double lim)
    {
      if (x > lim)
        return lim;
      if (x < -lim)
        return -lim;
      return x;
    }

    inline double clampBetween(double x, double lo, double hi)
    {
      if (x < lo)
        return lo;
      if (x > hi)
        return hi;
      return x;
    }

    // 连续软死区（shrinkage）：避免跨越 deadzone 阈值时的力矩阶跃冲击
    inline double applyDeadzone(double x, double dz)
    {
      if (x > dz)
      {
        return x - dz;
      }
      if (x < -dz)
      {
        return x + dz;
      }
      return 0.0;
    }

    inline bool clampWrenchWithFlag(ForceSensorData &w, double force_limit, double moment_limit)
    {
      bool sat = false;
      sat |= clampAbsWithFlag(w.fx, force_limit);
      sat |= clampAbsWithFlag(w.fy, force_limit);
      sat |= clampAbsWithFlag(w.fz, force_limit);
      sat |= clampAbsWithFlag(w.mx, moment_limit);
      sat |= clampAbsWithFlag(w.my, moment_limit);
      sat |= clampAbsWithFlag(w.mz, moment_limit);
      return sat;
    }

    // ===================== 编码器参数 =====================
    constexpr double PerAngle00 = 524288.0 / 360.0;
    constexpr double PerAngle01 = 262144.0 / 360.0;
    constexpr double PerAngle02 = 2048.0 / 360.0;

    // ===================== 零位偏置 =====================
    // 左臂零位偏置角，单位：deg
    constexpr double SZO10 = 249.46;
    constexpr double SZO20 = 12172.4808;
    constexpr double SZO30 = -12.1935;
    constexpr double SZO40 = 0;
    constexpr double SZO50 = 23313.27;
    constexpr double SZO60 = -728.71;
    constexpr double SZO70 = -119.21;
    constexpr double SZO80 = -23.2018;
    constexpr double SZO90 = -23.2018;
    constexpr double SZO100 = -833.12;

    // 右臂零位偏置角，单位：deg
    constexpr double SZRO10 = -1168.8423;
    constexpr double SZRO20 = -100.2187;
    constexpr double SZRO30 = 1009.6985;
    constexpr double SZRO40 = -157.2542;
    constexpr double SZRO50 = 76.6393;
    constexpr double SZRO60 = 1231.8668;
    constexpr double SZRO70 = 6597.68;
    constexpr double SZRO80 = -447.8906;
    constexpr double SZRO90 = 0.0;
    constexpr double SZRO100 = -1200.0;

    // ===================== 单轴参数表 =====================
    struct JointParam
    {
      double per_angle;    // pulse / deg
      double zero_offset;  // deg
      double ratio;        // 电机角 / 关节角
      double torque_scale; // Nm / raw
    };

    constexpr JointParam joint_map[21] = {
        {},

        // 右臂 1~10
        {PerAngle01, SZRO10, 121.0, 61.0 / 1000.0},
        {PerAngle00, SZRO20, 121.0, 61.0 / 1000.0},
        {PerAngle01, SZRO30, 121.0, 43.68 / 1000.0},
        {PerAngle01, SZRO40, 101.0, 49.0 / 1000.0},
        {PerAngle01, SZRO50, 121.0, 43.68 / 1000.0},
        {PerAngle00, SZRO60, 101.0, 36.4 / 1000.0},
        {PerAngle01, SZRO70, 100.0, 18.0 / 1000.0},
        {PerAngle02, SZRO80, 2048.0, 0.0},
        {PerAngle01, SZRO90, 100.0, 3.9 / 1000.0},
        {PerAngle00, SZRO100, 100.0, 3.9 / 1000.0},

        // 左臂 11~20
        {PerAngle00, SZO10, 121.0, 61.0 / 1000.0},
        {PerAngle01, SZO20, 121.0, 61.0 / 1000.0},
        {PerAngle01, SZO30, 121.0, 43.68 / 1000.0},
        {PerAngle01, SZO40, 101.0, 49.0 / 1000.0},
        {PerAngle01, SZO50, 121.0, 43.68 / 1000.0},
        {PerAngle00, SZO60, 101.0, 36.4 / 1000.0},
        {PerAngle01, SZO70, 100.0, 18.0 / 1000.0},
        {PerAngle02, SZO80, 2048.0, 0.0},
        {PerAngle01, SZO90, 100.0, 3.9 / 1000.0},
        {PerAngle01, SZO100, 100.0, 3.9 / 1000.0},
    };

    inline bool isLeftAxis(int axis)
    {
      return axis >= 11 && axis <= 20;
    }

    inline const JointParam *getJointParam(int axis)
    {
      if (axis < 1 || axis > 20)
      {
        return nullptr;
      }
      return &joint_map[axis];
    }

    // pulse/ms -> deg/s（只保留幅值）
    inline double pulsePerMsToJointSpeedDegPerSecMagnitude(int axis, double pulse_per_ms)
    {
      const auto *p = getJointParam(axis);
      if (p == nullptr)
      {
        return 0.0;
      }

      return std::abs(pulse_per_ms) / (p->per_angle * p->ratio) * 1000.0;
    }

    // pulse/ms^2 -> deg/s^2（只保留幅值）
    inline double pulsePerMs2ToJointAccelDegPerSec2Magnitude(int axis, double pulse_per_ms2)
    {
      const auto *p = getJointParam(axis);
      if (p == nullptr)
      {
        return 0.0;
      }

      return std::abs(pulse_per_ms2) / (p->per_angle * p->ratio) * 1000000.0;
    }

    // 单臂独立 URDF 中，7 个主动关节的 v 索引固定为 0~6。
    constexpr std::array<int, 7> SINGLE_ARM_V_IDS = {0, 1, 2, 3, 4, 5, 6};

    inline void applyArmGravityCompensation(
        Eigen::VectorXd &gravity,
        const std::array<int, 7> &v_ids,
        const GravityCompensationParameters &parameters)
    {
      for (std::size_t i = 0; i < 7; ++i)
      {
        const int v_id = v_ids[i];
        if (v_id < 0 || v_id >= gravity.size())
        {
          continue;
        }

        gravity(v_id) =
            parameters.gain[i] * gravity(v_id) +
            parameters.bias_nm[i];
      }
    }

    inline double calculateJointFrictionCompensation(
        double joint_velocity_rad_s,
        std::size_t joint_index,
        const FrictionCompensationParameters &parameters)
    {
      if (joint_index >= 7 || !std::isfinite(joint_velocity_rad_s))
      {
        return 0.0;
      }

      const double coulomb = parameters.coulomb_nm[joint_index];
      const double viscous = parameters.viscous_nm_per_rad_s[joint_index];
      const double smooth = parameters.velocity_smooth_rad_s[joint_index];

      if (!std::isfinite(coulomb) ||
          !std::isfinite(viscous) ||
          !std::isfinite(smooth) ||
          smooth <= 0.0)
      {
        return 0.0;
      }

      const double tau_coulomb =
          coulomb * std::tanh(joint_velocity_rad_s / smooth);

      const double tau_viscous =
          viscous * joint_velocity_rad_s;

      return tau_coulomb + tau_viscous;
    }

    inline void addArmFrictionCompensation(
        Eigen::VectorXd &torque,
        const Eigen::VectorXd &velocity,
        const std::array<int, 7> &v_ids,
        const FrictionCompensationParameters &parameters)
    {
      for (std::size_t i = 0; i < 7; ++i)
      {
        const int v_id = v_ids[i];

        if (v_id < 0 ||
            v_id >= torque.size() ||
            v_id >= velocity.size())
        {
          continue;
        }

        torque(v_id) +=
            calculateJointFrictionCompensation(
                velocity(v_id), i, parameters);
      }
    }

    // ===================== 各模式内部状态（支持重复进入前重置） =====================

    // 左臂主动导纳内部状态
    bool left_active_admittance_initialized = false;
    Eigen::VectorXd left_active_q_adm;
    Eigen::VectorXd left_active_dq_adm;

    // 右臂主动导纳内部状态
    bool right_active_admittance_initialized = false;
    Eigen::VectorXd right_active_q_adm;
    Eigen::VectorXd right_active_dq_adm;

    // 双侧主动导纳内部状态
    int bilateral_active_debug_counter = 0;
    int bilateral_active_fault_counter = 0;
    bool bilateral_active_admittance_initialized = false;
    Eigen::VectorXd bilateral_active_q_adm;
    Eigen::VectorXd bilateral_active_dq_adm;

    // 左臂被动正弦（PID）内部状态
    bool passive_pid_initialized = false;
    bool passive_pid_first_step = true;
    double passive_pid_elapsed_t = 0.0;
    std::array<double, 7> passive_pid_q0{};
    std::array<double, 4> passive_pid_e_int{};
    std::array<double, 4> passive_pid_e_prev{};
    std::array<double, 4> passive_pid_de_filt{};

    // 左臂被动正弦（PD+FF）内部状态
    bool passive_pdff_initialized = false;
    double passive_pdff_elapsed_t = 0.0;
    std::array<double, 7> passive_pdff_q0{};

  } // namespace

  // ===================== 实验运行参数接口 =====================

  void setExperimentRuntimeParams(const ExperimentRuntimeParams &params)
  {
    g_exp_params = params;
  }

  ExperimentRuntimeParams getExperimentRuntimeParams()
  {
    return g_exp_params;
  }

  std::string formatParamPreset(int mode, const ExperimentRuntimeParams &params)
  {
    if (mode == MODE_PASSIVE || mode == MODE_PASSIVE_SINE_PD_FF)
    {
      const char *joint_tag = "combined";
      switch (params.passive_joint_profile)
      {
      case PASSIVE_JOINT_J4_ONLY:
        joint_tag = "GH1";
        break;
      case PASSIVE_JOINT_J5_ONLY:
        joint_tag = "GH2";
        break;
      case PASSIVE_JOINT_J6_ONLY:
        joint_tag = "GH3";
        break;
      case PASSIVE_JOINT_J7_ONLY:
        joint_tag = "EB1";
        break;
      case PASSIVE_JOINTS_COMBINED:
      default:
        joint_tag = "GH1to3_EB1";
        break;
      }
      const char *speed_tag =
          (params.passive_speed_level == PASSIVE_SPEED_SLOW) ? "slow_0.5x" : "std_1.0x";
      return std::string("passive_") + joint_tag + "_" + speed_tag;
    }

    if (mode == MODE_ACTIVE || mode == MODE_RIGHT_ACTIVE || mode == MODE_BILATERAL_ACTIVE)
    {
      switch (params.admittance_damping_level)
      {
      case ADMITTANCE_DAMPING_LOW:
        return "admittance_damp_low_0.7x";
      case ADMITTANCE_DAMPING_HIGH:
        return "admittance_damp_high_1.5x";
      case ADMITTANCE_DAMPING_STANDARD:
      default:
        return "admittance_damp_std_1.0x";
      }
    }

    if (mode == MODE_ZERO_FORCE || mode == MODE_RIGHT_ZERO_FORCE || mode == MODE_BILATERAL_ZERO_FORCE)
    {
      return "zero_force_default";
    }

    if (mode == MODE_LEFT_PRESET_POSITION || mode == MODE_RIGHT_PRESET_POSITION)
    {
      return "preset_position";
    }

    return "idle";
  }

  // ===================== 位置换算 =====================

  double jointDegToPulse(int axis, double joint_deg)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    if (isLeftAxis(axis))
    {
      return (p->zero_offset + p->ratio * joint_deg) * p->per_angle;
    }

    return (p->zero_offset - p->ratio * joint_deg) * p->per_angle;
  }

  double pulseToJointDeg(int axis, double pulse)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    if (isLeftAxis(axis))
    {
      return (pulse / p->per_angle - p->zero_offset) / p->ratio;
    }

    return (p->zero_offset - pulse / p->per_angle) / p->ratio;
  }

  // ===================== 速度换算 =====================

  double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    return std::abs(joint_speed_deg_per_sec) * p->per_angle * p->ratio / 1000.0;
  }

  double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    if (isLeftAxis(axis))
    {
      return (pulse_per_ms / p->per_angle) / p->ratio * 1000.0;
    }

    return -(pulse_per_ms / p->per_angle) / p->ratio * 1000.0;
  }

  // ===================== 加速度换算 =====================

  double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    return std::abs(joint_acc_deg_per_sec2) * p->per_angle * p->ratio / 1000000.0;
  }

  double pulsePerMs2ToJointDegPerSec2(int axis, double pulse_per_ms2)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    if (isLeftAxis(axis))
    {
      return (pulse_per_ms2 / p->per_angle) / p->ratio * 1000000.0;
    }

    return -(pulse_per_ms2 / p->per_angle) / p->ratio * 1000000.0;
  }

  // ===================== 扭矩换算 =====================

  double torqueRawToNm(int axis, double raw)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    return raw * p->torque_scale;
  }

  double torqueNmToRaw(int axis, double torque_nm)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr || p->torque_scale == 0.0)
    {
      return 0.0;
    }

    return torque_nm / p->torque_scale;
  }

  // ===================== 模式状态重置函数 =====================

  void resetActiveMode()
  {
    left_active_admittance_initialized = false;
    left_active_q_adm.resize(0);
    left_active_dq_adm.resize(0);
    std::cout << "[resetActiveMode] 左臂主动模式导纳状态已重置" << std::endl;
  }

  void resetRightActiveMode()
  {
    right_active_admittance_initialized = false;
    right_active_q_adm.resize(0);
    right_active_dq_adm.resize(0);
    std::cout << "[resetRightActiveMode] 右臂主动模式导纳状态已重置" << std::endl;
  }

  void resetBilateralActiveMode()
  {
    bilateral_active_debug_counter = 0;
    bilateral_active_fault_counter = 0;
    bilateral_active_admittance_initialized = false;
    bilateral_active_q_adm.resize(0);
    bilateral_active_dq_adm.resize(0);
    std::cout << "[resetBilateralActiveMode] 双侧主动模式状态已重置" << std::endl;
  }

  void resetPassiveMode()
  {
    passive_pid_initialized = false;
    passive_pid_first_step = true;
    passive_pid_elapsed_t = 0.0;
    passive_pid_q0.fill(0.0);
    passive_pid_e_int.fill(0.0);
    passive_pid_e_prev.fill(0.0);
    passive_pid_de_filt.fill(0.0);
    std::cout << "[resetPassiveMode] 左臂被动正弦（PID）状态已重置" << std::endl;
  }

  void resetPassivePDFFMode()
  {
    passive_pdff_initialized = false;
    passive_pdff_elapsed_t = 0.0;
    passive_pdff_q0.fill(0.0);
    std::cout << "[resetPassivePDFFMode] 左臂被动正弦（PD+FF）状态已重置" << std::endl;
  }

  // ===================== 空闲模式 =====================
  void runIdle(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;
    output = ControlOutputCache{};
    output.valid = false;
  }

  // ===================== 左臂主动导纳模式（MODE_ACTIVE = 3） =====================
  void runActiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    output = ControlOutputCache{};
    output.valid = false;

    constexpr std::size_t IDX_J1 = 10;
    constexpr std::size_t IDX_J2 = 11;
    constexpr std::size_t IDX_J3 = 12;
    constexpr std::size_t IDX_J4 = 13;
    constexpr std::size_t IDX_J5 = 14;
    constexpr std::size_t IDX_J6 = 15;
    constexpr std::size_t IDX_J7 = 16;
    const std::array<std::size_t, 7> left_ctrl_id = {
        IDX_J1, IDX_J2, IDX_J3, IDX_J4, IDX_J5, IDX_J6, IDX_J7};

    // Pinocchio 懒加载上下文
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/left_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        frame_id_arm = model.getFrameId("ft_sensor_frame");
        frame_id_hand = model.getFrameId("ft_sensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runActiveMode] 没找到 frame: ft_sensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runActiveMode] 没找到 frame: ft_sensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr << "[runActiveMode] 当前 Pinocchio 模型自由度不是 7。"
                    << " model.nq=" << model.nq << " model.nv=" << model.nv
                    << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout << "[runActiveMode] Pinocchio 初始化成功。"
                    << " urdf_path=" << urdf_path
                    << " nq=" << model.nq
                    << " nv=" << model.nv
                    << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runActiveMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / M_PI;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      q(static_cast<Eigen::Index>(i)) = state.position[left_ctrl_id[i]] * DEG2RAD;
      v(static_cast<Eigen::Index>(i)) = state.velocity[left_ctrl_id[i]] * DEG2RAD;
    }

    if (!q.allFinite() || !v.allFinite())
    {
      std::cerr << "[runActiveMode] q/v 出现 NaN 或 Inf，本轮不输出" << std::endl;
      resetActiveMode();
      output.valid = false;
      return;
    }

    // 导纳状态初始化
    if (!left_active_admittance_initialized ||
        left_active_q_adm.size() != model.nq ||
        left_active_dq_adm.size() != model.nv)
    {
      left_active_q_adm = q;
      left_active_dq_adm = Eigen::VectorXd::Zero(model.nv);
      left_active_admittance_initialized = true;
    }

    // 读取并去偏两个六维力传感器
    const auto &raw_hand = sensor_state.sensor[SENSOR_LEFT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_LEFT_HAND];
    const auto &raw_arm = sensor_state.sensor[SENSOR_LEFT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_LEFT_ARM];

    ForceSensorData arm_w{
        raw_arm.fx - bias_arm.fx,
        raw_arm.fy - bias_arm.fy,
        raw_arm.fz - bias_arm.fz,
        raw_arm.mx - bias_arm.mx,
        raw_arm.my - bias_arm.my,
        raw_arm.mz - bias_arm.mz};

    ForceSensorData hand_w{
        raw_hand.fx - bias_hand.fx,
        raw_hand.fy - bias_hand.fy,
        raw_hand.fz - bias_hand.fz,
        raw_hand.mx - bias_hand.mx,
        raw_hand.my - bias_hand.my,
        raw_hand.mz - bias_hand.mz};

    // 传感器方向符号修正
    static const std::array<double, 6> arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    arm_w.fx *= arm_sign[0];
    arm_w.fy *= arm_sign[1];
    arm_w.fz *= arm_sign[2];
    arm_w.mx *= arm_sign[3];
    arm_w.my *= arm_sign[4];
    arm_w.mz *= arm_sign[5];

    hand_w.fx *= hand_sign[0];
    hand_w.fy *= hand_sign[1];
    hand_w.fz *= hand_sign[2];
    hand_w.mx *= hand_sign[3];
    hand_w.my *= hand_sign[4];
    hand_w.mz *= hand_sign[5];

    // 死区
    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    arm_w.fx = applyDeadzone(arm_w.fx, force_deadzone);
    arm_w.fy = applyDeadzone(arm_w.fy, force_deadzone);
    arm_w.fz = applyDeadzone(arm_w.fz, force_deadzone);
    arm_w.mx = applyDeadzone(arm_w.mx, torque_deadzone);
    arm_w.my = applyDeadzone(arm_w.my, torque_deadzone);
    arm_w.mz = applyDeadzone(arm_w.mz, torque_deadzone);

    hand_w.fx = applyDeadzone(hand_w.fx, force_deadzone);
    hand_w.fy = applyDeadzone(hand_w.fy, force_deadzone);
    hand_w.fz = applyDeadzone(hand_w.fz, force_deadzone);
    hand_w.mx = applyDeadzone(hand_w.mx, torque_deadzone);
    hand_w.my = applyDeadzone(hand_w.my, torque_deadzone);
    hand_w.mz = applyDeadzone(hand_w.mz, torque_deadzone);

    // 全局统一大臂传感器交互力反馈增益
    applyUpperArmSensorWorldAxisGains(arm_w);

    // 限幅
    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm
    bool ft_sat = false;
    ft_sat |= clampWrenchWithFlag(arm_w, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(hand_w, force_limit, moment_limit);

    // 计算两个传感器 frame 的 Jacobian
    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    pinocchio::Force f_arm = pinocchio::Force::Zero();
    f_arm.linear() << arm_w.fx, arm_w.fy, arm_w.fz;
    f_arm.angular() << arm_w.mx, arm_w.my, arm_w.mz;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << hand_w.fx, hand_w.fy, hand_w.fz;
    f_hand.angular() << hand_w.mx, hand_w.my, hand_w.mz;

    Eigen::VectorXd tau_ext =
        J_arm.transpose() * f_arm.toVector() +
        J_hand.transpose() * f_hand.toVector();

    // ===================== 导纳模型（使用实际控制周期 dt_s） =====================
    const double dt = clampControlDt(dt_s);
    const double damping_scale =
        getAdmittanceDampingScale(g_exp_params.admittance_damping_level);

    Eigen::VectorXd M_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd D_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd K_a = Eigen::VectorXd::Zero(model.nv);

    M_a(0) = 4.0;
    M_a(1) = 4.0;
    M_a(2) = 3.0;
    M_a(3) = 1.5;
    M_a(4) = 1.5;
    M_a(5) = 1.2;
    M_a(6) = 0.8;

    D_a(0) = 45.0 * damping_scale;
    D_a(1) = 45.0 * damping_scale;
    D_a(2) = 35.0 * damping_scale;
    D_a(3) = 20.0 * damping_scale;
    D_a(4) = 20.0 * damping_scale;
    D_a(5) = 18.0 * damping_scale;
    D_a(6) = 12.0 * damping_scale;

    K_a(0) = 18.0;
    K_a(1) = 18.0;
    K_a(2) = 14.0;
    K_a(3) = 8.0;
    K_a(4) = 8.0;
    K_a(5) = 6.0;
    K_a(6) = 4.0;

    Eigen::VectorXd ddq_adm = Eigen::VectorXd::Zero(model.nv);
    for (int i = 0; i < model.nv; ++i)
    {
      ddq_adm(i) =
          (tau_ext(i) - D_a(i) * left_active_dq_adm(i) -
           K_a(i) * (left_active_q_adm(i) - q(i))) /
          M_a(i);
    }

    left_active_dq_adm = left_active_dq_adm + ddq_adm * dt;
    left_active_q_adm = left_active_q_adm + left_active_dq_adm * dt;

    // 导纳参考限位
    const std::array<double, 7> q_min = {
        -30.0 * DEG2RAD, -30.0 * DEG2RAD, -30.0 * DEG2RAD,
        -60.0 * DEG2RAD, -60.0 * DEG2RAD, -60.0 * DEG2RAD,
        0.0 * DEG2RAD};
    const std::array<double, 7> q_max = {
        30.0 * DEG2RAD, 30.0 * DEG2RAD, 30.0 * DEG2RAD,
        60.0 * DEG2RAD, 60.0 * DEG2RAD, 60.0 * DEG2RAD,
        120.0 * DEG2RAD};
    const std::array<double, 7> dq_limit = {
        1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5};

    for (std::size_t i = 0; i < 7; ++i)
    {
      left_active_q_adm(static_cast<Eigen::Index>(i)) =
          clampBetween(left_active_q_adm(static_cast<Eigen::Index>(i)), q_min[i], q_max[i]);
      left_active_dq_adm(static_cast<Eigen::Index>(i)) =
          clampAbs(left_active_dq_adm(static_cast<Eigen::Index>(i)), dq_limit[i]);
    }

    // 重力补偿与摩擦补偿（分开记录，求和与原公式完全等价）
    Eigen::VectorXd tau_grav = pinocchio::computeGeneralizedGravity(model, *data, q);
    applyArmGravityCompensation(
        tau_grav, SINGLE_ARM_V_IDS, LEFT_ARM_GRAVITY_COMPENSATION);

    Eigen::VectorXd tau_fric = Eigen::VectorXd::Zero(model.nv);
    addArmFrictionCompensation(
        tau_fric, v, SINGLE_ARM_V_IDS, LEFT_ARM_FRICTION_COMPENSATION);

    Eigen::VectorXd g = tau_grav + tau_fric;

    // 扭矩跟踪控制
    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd Kv = Eigen::VectorXd::Zero(model.nv);

    Kp(0) = 20.0;
    Kp(1) = 20.0;
    Kp(2) = 16.0;
    Kp(3) = 18.0;
    Kp(4) = 18.0;
    Kp(5) = 16.0;
    Kp(6) = 10.0;

    Kv(0) = 4.0;
    Kv(1) = 4.0;
    Kv(2) = 3.0;
    Kv(3) = 2.5;
    Kv(4) = 2.5;
    Kv(5) = 2.0;
    Kv(6) = 1.5;

    Eigen::VectorXd tau_cmd =
        g + Kp.asDiagonal() * (left_active_q_adm - q) +
        Kv.asDiagonal() * (left_active_dq_adm - v);

    // 最终扭矩限幅（单轴额定扭矩 61/61/43.7/49/43.7/36.4/18 Nm，留足 SG1/GH1 重力+交互裕度）
    const std::array<double, 7> torque_limit = {
        50.0, 30.0, 22.0, 20.0, 16.0, 16.0, 10.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      double t_i = tau_cmd(static_cast<Eigen::Index>(i));
      const bool sat = clampAbsWithFlag(t_i, torque_limit[i]);
      tau_cmd(static_cast<Eigen::Index>(i)) = t_i;

      const std::size_t idx = left_ctrl_id[i];
      output.use_torque[idx] = true;
      output.target_torque[idx] = t_i;

      if (diag != nullptr)
      {
        diag->qd_deg[idx] = left_active_q_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->qa_deg[idx] = left_active_q_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->dqa_deg_s[idx] = left_active_dq_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->ddqa_deg_s2[idx] = ddq_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->tau_int_nm[idx] = tau_ext(static_cast<Eigen::Index>(i));
        diag->tau_grav_nm[idx] = tau_grav(static_cast<Eigen::Index>(i));
        diag->tau_fric_nm[idx] = tau_fric(static_cast<Eigen::Index>(i));
        diag->saturated[idx] = sat;
      }
    }

    if (diag != nullptr)
    {
      diag->proc_ft[SENSOR_LEFT_ARM] = arm_w;
      diag->proc_ft[SENSOR_LEFT_HAND] = hand_w;
      diag->ft_saturated = ft_sat;
    }

    // 末端其余关节固定（与原程序保持一致）
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

    output.use_position[18] = true;
    output.target_position[18] = 0.0;
    output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[18] = 25;

    output.use_position[19] = true;
    output.target_position[19] = 0.0;
    output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[19] = 25;

    output.valid = true;
  }

  // ===================== 右臂主动导纳模式（MODE_RIGHT_ACTIVE = 14） =====================
  void runRightActiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    output = ControlOutputCache{};
    output.valid = false;

    const std::array<std::size_t, 7> right_ctrl_id = {0, 1, 2, 3, 4, 5, 6};

    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/right_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        frame_id_arm = model.getFrameId("ft_rightsensor_frame");
        frame_id_hand = model.getFrameId("ft_rightsensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runRightActiveMode] 没找到 frame: ft_rightsensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runRightActiveMode] 没找到 frame: ft_rightsensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr << "[runRightActiveMode] 当前 Pinocchio 模型自由度不是 7。"
                    << " model.nq=" << model.nq << " model.nv=" << model.nv
                    << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout << "[runRightActiveMode] Pinocchio 初始化成功。"
                    << " urdf_path=" << urdf_path
                    << " nq=" << model.nq
                    << " nv=" << model.nv
                    << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runRightActiveMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / M_PI;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      q(static_cast<Eigen::Index>(i)) = state.position[right_ctrl_id[i]] * DEG2RAD;
      v(static_cast<Eigen::Index>(i)) = state.velocity[right_ctrl_id[i]] * DEG2RAD;
    }

    if (!q.allFinite() || !v.allFinite())
    {
      std::cerr << "[runRightActiveMode] q/v 出现 NaN 或 Inf，本轮不输出" << std::endl;
      resetRightActiveMode();
      output.valid = false;
      return;
    }

    if (!right_active_admittance_initialized ||
        right_active_q_adm.size() != model.nq ||
        right_active_dq_adm.size() != model.nv)
    {
      right_active_q_adm = q;
      right_active_dq_adm = Eigen::VectorXd::Zero(model.nv);
      right_active_admittance_initialized = true;
      std::cout << "[runRightActiveMode] 导纳状态已用当前右臂姿态初始化" << std::endl;
    }

    const auto &raw_hand = sensor_state.sensor[SENSOR_RIGHT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_RIGHT_HAND];
    const auto &raw_arm = sensor_state.sensor[SENSOR_RIGHT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_RIGHT_ARM];

    ForceSensorData arm_w{
        raw_arm.fx - bias_arm.fx,
        raw_arm.fy - bias_arm.fy,
        raw_arm.fz - bias_arm.fz,
        raw_arm.mx - bias_arm.mx,
        raw_arm.my - bias_arm.my,
        raw_arm.mz - bias_arm.mz};

    ForceSensorData hand_w{
        raw_hand.fx - bias_hand.fx,
        raw_hand.fy - bias_hand.fy,
        raw_hand.fz - bias_hand.fz,
        raw_hand.mx - bias_hand.mx,
        raw_hand.my - bias_hand.my,
        raw_hand.mz - bias_hand.mz};

    static const std::array<double, 6> arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    arm_w.fx *= arm_sign[0];
    arm_w.fy *= arm_sign[1];
    arm_w.fz *= arm_sign[2];
    arm_w.mx *= arm_sign[3];
    arm_w.my *= arm_sign[4];
    arm_w.mz *= arm_sign[5];

    hand_w.fx *= hand_sign[0];
    hand_w.fy *= hand_sign[1];
    hand_w.fz *= hand_sign[2];
    hand_w.mx *= hand_sign[3];
    hand_w.my *= hand_sign[4];
    hand_w.mz *= hand_sign[5];

    // 死区（保持原右臂主动模式灵敏度）
    const double force_deadzone = 0.5;   // N
    const double torque_deadzone = 0.03; // Nm

    arm_w.fx = applyDeadzone(arm_w.fx, force_deadzone);
    arm_w.fy = applyDeadzone(arm_w.fy, force_deadzone);
    arm_w.fz = applyDeadzone(arm_w.fz, force_deadzone);
    arm_w.mx = applyDeadzone(arm_w.mx, torque_deadzone);
    arm_w.my = applyDeadzone(arm_w.my, torque_deadzone);
    arm_w.mz = applyDeadzone(arm_w.mz, torque_deadzone);

    hand_w.fx = applyDeadzone(hand_w.fx, force_deadzone);
    hand_w.fy = applyDeadzone(hand_w.fy, force_deadzone);
    hand_w.fz = applyDeadzone(hand_w.fz, force_deadzone);
    hand_w.mx = applyDeadzone(hand_w.mx, torque_deadzone);
    hand_w.my = applyDeadzone(hand_w.my, torque_deadzone);
    hand_w.mz = applyDeadzone(hand_w.mz, torque_deadzone);

    applyUpperArmSensorWorldAxisGains(arm_w);

    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm
    bool ft_sat = false;
    ft_sat |= clampWrenchWithFlag(arm_w, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(hand_w, force_limit, moment_limit);

    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    pinocchio::Force f_arm = pinocchio::Force::Zero();
    f_arm.linear() << arm_w.fx, arm_w.fy, arm_w.fz;
    f_arm.angular() << arm_w.mx, arm_w.my, arm_w.mz;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << hand_w.fx, hand_w.fy, hand_w.fz;
    f_hand.angular() << hand_w.mx, hand_w.my, hand_w.mz;

    Eigen::VectorXd tau_ext =
        J_arm.transpose() * f_arm.toVector() +
        J_hand.transpose() * f_hand.toVector();

    if (!tau_ext.allFinite())
    {
      std::cerr << "[runRightActiveMode] tau_ext 出现 NaN 或 Inf，本轮不输出" << std::endl;
      resetRightActiveMode();
      output.valid = false;
      return;
    }

    // ===================== 导纳模型（使用实际控制周期 dt_s） =====================
    const double dt = clampControlDt(dt_s);
    const double damping_scale =
        getAdmittanceDampingScale(g_exp_params.admittance_damping_level);
    const double admittance_input_gain = 1.5;

    const std::array<double, 7> M_value = {
        1.5, 1.5, 1.2, 0.65, 0.65, 0.50, 0.40};
    const std::array<double, 7> D_value = {
        22.0, 22.0, 18.0, 10.0, 10.0, 8.0, 6.0};
    const std::array<double, 7> K_value = {
        2.0, 2.0, 1.5, 0.8, 0.8, 0.5, 0.3};
    const std::array<double, 7> Kp_value = {
        20.0, 20.0, 16.0, 18.0, 18.0, 16.0, 10.0};
    const std::array<double, 7> Kv_value = {
        4.0, 4.0, 3.0, 2.5, 2.5, 2.0, 1.5};

    Eigen::VectorXd M_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd D_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd K_a = Eigen::VectorXd::Zero(model.nv);

    for (int i = 0; i < model.nv; ++i)
    {
      M_a(i) = M_value[static_cast<std::size_t>(i)];
      D_a(i) = D_value[static_cast<std::size_t>(i)] * damping_scale;
      K_a(i) = K_value[static_cast<std::size_t>(i)];
    }

    Eigen::VectorXd ddq_adm = Eigen::VectorXd::Zero(model.nv);
    for (int i = 0; i < model.nv; ++i)
    {
      ddq_adm(i) =
          (admittance_input_gain * tau_ext(i) -
           D_a(i) * right_active_dq_adm(i) -
           K_a(i) * (right_active_q_adm(i) - q(i))) /
          M_a(i);
    }

    right_active_dq_adm = right_active_dq_adm + ddq_adm * dt;
    right_active_q_adm = right_active_q_adm + right_active_dq_adm * dt;

    // 导纳参考限位
    const std::array<double, 7> q_min = {
        -30.0 * DEG2RAD, -30.0 * DEG2RAD, -30.0 * DEG2RAD,
        -60.0 * DEG2RAD, -60.0 * DEG2RAD, -60.0 * DEG2RAD,
        0.0 * DEG2RAD};
    const std::array<double, 7> q_max = {
        30.0 * DEG2RAD, 30.0 * DEG2RAD, 30.0 * DEG2RAD,
        60.0 * DEG2RAD, 60.0 * DEG2RAD, 60.0 * DEG2RAD,
        120.0 * DEG2RAD};
    const std::array<double, 7> dq_limit = {
        1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5};

    for (std::size_t i = 0; i < 7; ++i)
    {
      right_active_q_adm(static_cast<Eigen::Index>(i)) =
          clampBetween(right_active_q_adm(static_cast<Eigen::Index>(i)), q_min[i], q_max[i]);
      right_active_dq_adm(static_cast<Eigen::Index>(i)) =
          clampAbs(right_active_dq_adm(static_cast<Eigen::Index>(i)), dq_limit[i]);
    }

    // 统一重力补偿与摩擦补偿
    Eigen::VectorXd tau_grav = pinocchio::computeGeneralizedGravity(model, *data, q);
    applyArmGravityCompensation(
        tau_grav, SINGLE_ARM_V_IDS, RIGHT_ARM_GRAVITY_COMPENSATION);

    Eigen::VectorXd tau_fric = Eigen::VectorXd::Zero(model.nv);
    addArmFrictionCompensation(
        tau_fric, v, SINGLE_ARM_V_IDS, RIGHT_ARM_FRICTION_COMPENSATION);

    Eigen::VectorXd g = tau_grav + tau_fric;

    if (!g.allFinite())
    {
      std::cerr << "[runRightActiveMode] g(q) 出现 NaN 或 Inf，本轮不输出" << std::endl;
      resetRightActiveMode();
      output.valid = false;
      return;
    }

    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd Kv = Eigen::VectorXd::Zero(model.nv);
    for (int i = 0; i < model.nv; ++i)
    {
      Kp(i) = Kp_value[static_cast<std::size_t>(i)];
      Kv(i) = Kv_value[static_cast<std::size_t>(i)];
    }

    Eigen::VectorXd tau_cmd =
        g +
        Kp.asDiagonal() * (right_active_q_adm - q) +
        Kv.asDiagonal() * (right_active_dq_adm - v);

    if (!tau_cmd.allFinite())
    {
      std::cerr << "[runRightActiveMode] tau_cmd 出现 NaN 或 Inf，本轮不输出" << std::endl;
      resetRightActiveMode();
      output.valid = false;
      return;
    }

    // 最终扭矩限幅与右臂输出方向修正（-1.0）（单轴额定扭矩 61/61/43.7/49/43.7/36.4/18 Nm）
    const std::array<double, 7> torque_limit = {
        50.0, 30.0, 22.0, 20.0, 16.0, 16.0, 10.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      double t_out = -tau_cmd(static_cast<Eigen::Index>(i));
      const bool sat = clampAbsWithFlag(t_out, torque_limit[i]);

      const std::size_t idx = right_ctrl_id[i];
      output.use_torque[idx] = true;
      output.target_torque[idx] = t_out;

      if (diag != nullptr)
      {
        diag->qd_deg[idx] = right_active_q_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->qa_deg[idx] = right_active_q_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->dqa_deg_s[idx] = right_active_dq_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->ddqa_deg_s2[idx] = ddq_adm(static_cast<Eigen::Index>(i)) * RAD2DEG;
        diag->tau_int_nm[idx] = tau_ext(static_cast<Eigen::Index>(i));
        diag->tau_grav_nm[idx] = tau_grav(static_cast<Eigen::Index>(i));
        diag->tau_fric_nm[idx] = tau_fric(static_cast<Eigen::Index>(i));
        diag->saturated[idx] = sat;
      }
    }

    if (diag != nullptr)
    {
      diag->proc_ft[SENSOR_RIGHT_ARM] = arm_w;
      diag->proc_ft[SENSOR_RIGHT_HAND] = hand_w;
      diag->ft_saturated = ft_sat;
    }

    output.valid = true;
  }

  // ===================== 左臂零力拖动模式（MODE_ZERO_FORCE = 4） =====================
  void runZeroForceMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    output = ControlOutputCache{};
    output.valid = false;

    const std::array<std::size_t, 7> left_ctrl_id = {10, 11, 12, 13, 14, 15, 16};

    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/left_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        frame_id_arm = model.getFrameId("ft_sensor_frame");
        frame_id_hand = model.getFrameId("ft_sensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runZeroForceMode] 没找到 frame: ft_sensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runZeroForceMode] 没找到 frame: ft_sensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr << "[runZeroForceMode] 当前 Pinocchio 模型自由度不是 7。"
                    << " model.nq=" << model.nq << " model.nv=" << model.nv
                    << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout << "[runZeroForceMode] Pinocchio 初始化成功。"
                    << " urdf_path=" << urdf_path
                    << " nq=" << model.nq
                    << " nv=" << model.nv
                    << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runZeroForceMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      q(static_cast<Eigen::Index>(i)) = state.position[left_ctrl_id[i]] * DEG2RAD;
      v(static_cast<Eigen::Index>(i)) = state.velocity[left_ctrl_id[i]] * DEG2RAD;
    }

    const auto &raw_hand = sensor_state.sensor[SENSOR_LEFT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_LEFT_HAND];
    const auto &raw_arm = sensor_state.sensor[SENSOR_LEFT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_LEFT_ARM];

    ForceSensorData arm_w{
        raw_arm.fx - bias_arm.fx,
        raw_arm.fy - bias_arm.fy,
        raw_arm.fz - bias_arm.fz,
        raw_arm.mx - bias_arm.mx,
        raw_arm.my - bias_arm.my,
        raw_arm.mz - bias_arm.mz};

    ForceSensorData hand_w{
        raw_hand.fx - bias_hand.fx,
        raw_hand.fy - bias_hand.fy,
        raw_hand.fz - bias_hand.fz,
        raw_hand.mx - bias_hand.mx,
        raw_hand.my - bias_hand.my,
        raw_hand.mz - bias_hand.mz};

    static const std::array<double, 6> arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    arm_w.fx *= arm_sign[0];
    arm_w.fy *= arm_sign[1];
    arm_w.fz *= arm_sign[2];
    arm_w.mx *= arm_sign[3];
    arm_w.my *= arm_sign[4];
    arm_w.mz *= arm_sign[5];

    hand_w.fx *= hand_sign[0];
    hand_w.fy *= hand_sign[1];
    hand_w.fz *= hand_sign[2];
    hand_w.mx *= hand_sign[3];
    hand_w.my *= hand_sign[4];
    hand_w.mz *= hand_sign[5];

    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    arm_w.fx = applyDeadzone(arm_w.fx, force_deadzone);
    arm_w.fy = applyDeadzone(arm_w.fy, force_deadzone);
    arm_w.fz = applyDeadzone(arm_w.fz, force_deadzone);
    arm_w.mx = applyDeadzone(arm_w.mx, torque_deadzone);
    arm_w.my = applyDeadzone(arm_w.my, torque_deadzone);
    arm_w.mz = applyDeadzone(arm_w.mz, torque_deadzone);

    hand_w.fx = applyDeadzone(hand_w.fx, force_deadzone);
    hand_w.fy = applyDeadzone(hand_w.fy, force_deadzone);
    hand_w.fz = applyDeadzone(hand_w.fz, force_deadzone);
    hand_w.mx = applyDeadzone(hand_w.mx, torque_deadzone);
    hand_w.my = applyDeadzone(hand_w.my, torque_deadzone);
    hand_w.mz = applyDeadzone(hand_w.mz, torque_deadzone);

    applyUpperArmSensorWorldAxisGains(arm_w);

    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm
    bool ft_sat = false;
    ft_sat |= clampWrenchWithFlag(arm_w, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(hand_w, force_limit, moment_limit);

    Eigen::VectorXd tau_grav = pinocchio::computeGeneralizedGravity(model, *data, q);
    applyArmGravityCompensation(
        tau_grav, SINGLE_ARM_V_IDS, LEFT_ARM_GRAVITY_COMPENSATION);

    Eigen::VectorXd tau_fric = Eigen::VectorXd::Zero(model.nv);
    addArmFrictionCompensation(
        tau_fric, v, SINGLE_ARM_V_IDS, LEFT_ARM_FRICTION_COMPENSATION);

    Eigen::VectorXd g = tau_grav + tau_fric;

    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    pinocchio::Force f_arm = pinocchio::Force::Zero();
    f_arm.linear() << arm_w.fx, arm_w.fy, arm_w.fz;
    f_arm.angular() << arm_w.mx, arm_w.my, arm_w.mz;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << hand_w.fx, hand_w.fy, hand_w.fz;
    f_hand.angular() << hand_w.mx, hand_w.my, hand_w.mz;

    Eigen::VectorXd tau_ext =
        J_arm.transpose() * f_arm.toVector() +
        J_hand.transpose() * f_hand.toVector();

    Eigen::VectorXd damping = Eigen::VectorXd::Zero(model.nv);
    damping(0) = 0.0;
    damping(1) = 0.4;
    damping(2) = 0.3;
    damping(3) = 0.0;
    damping(4) = 0.0;
    damping(5) = 0.2;
    damping(6) = 0.3;

    Eigen::VectorXd tau_damping = -(damping.asDiagonal() * v);
    Eigen::VectorXd tau = g + tau_ext + tau_damping;

    // 扭矩限幅（单轴额定扭矩 61/61/43.7/49/43.7/36.4/18 Nm，避免 SG1 重力>30Nm 与 GH1 拖动饱和）
    const std::array<double, 7> torque_limit = {
        50.0, 30.0, 22.0, 20.0, 16.0, 16.0, 10.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      double t_i = tau(static_cast<Eigen::Index>(i));
      const bool sat = clampAbsWithFlag(t_i, torque_limit[i]);

      const std::size_t idx = left_ctrl_id[i];
      output.use_torque[idx] = true;
      output.target_torque[idx] = t_i;

      if (diag != nullptr)
      {
        diag->tau_int_nm[idx] = tau_ext(static_cast<Eigen::Index>(i));
        diag->tau_grav_nm[idx] = tau_grav(static_cast<Eigen::Index>(i));
        diag->tau_fric_nm[idx] = tau_fric(static_cast<Eigen::Index>(i));
        diag->saturated[idx] = sat;
      }
    }

    if (diag != nullptr)
    {
      diag->proc_ft[SENSOR_LEFT_ARM] = arm_w;
      diag->proc_ft[SENSOR_LEFT_HAND] = hand_w;
      diag->ft_saturated = ft_sat;
    }

    // 末端其余关节固定
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

    output.use_position[18] = true;
    output.target_position[18] = 0.0;
    output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[18] = 25;

    output.use_position[19] = true;
    output.target_position[19] = 0.0;
    output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[19] = 25;

    output.valid = true;
  }

  // ===================== 右臂零力拖动模式（MODE_RIGHT_ZERO_FORCE = 8） =====================
  void runRightZeroForceMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    output = ControlOutputCache{};
    output.valid = false;

    const std::array<std::size_t, 7> right_ctrl_id = {0, 1, 2, 3, 4, 5, 6};

    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/right_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        frame_id_arm = model.getFrameId("ft_rightsensor_frame");
        frame_id_hand = model.getFrameId("ft_rightsensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runRightZeroForceMode] 没找到 frame: ft_rightsensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runRightZeroForceMode] 没找到 frame: ft_rightsensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr << "[runRightZeroForceMode] 当前 Pinocchio 模型自由度不是 7。"
                    << " model.nq=" << model.nq << " model.nv=" << model.nv
                    << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout << "[runRightZeroForceMode] Pinocchio 初始化成功。"
                    << " urdf_path=" << urdf_path
                    << " nq=" << model.nq
                    << " nv=" << model.nv
                    << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runRightZeroForceMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      q(static_cast<Eigen::Index>(i)) = state.position[right_ctrl_id[i]] * DEG2RAD;
      v(static_cast<Eigen::Index>(i)) = state.velocity[right_ctrl_id[i]] * DEG2RAD;
    }

    const auto &raw_hand = sensor_state.sensor[SENSOR_RIGHT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_RIGHT_HAND];
    const auto &raw_arm = sensor_state.sensor[SENSOR_RIGHT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_RIGHT_ARM];

    ForceSensorData arm_w{
        raw_arm.fx - bias_arm.fx,
        raw_arm.fy - bias_arm.fy,
        raw_arm.fz - bias_arm.fz,
        raw_arm.mx - bias_arm.mx,
        raw_arm.my - bias_arm.my,
        raw_arm.mz - bias_arm.mz};

    ForceSensorData hand_w{
        raw_hand.fx - bias_hand.fx,
        raw_hand.fy - bias_hand.fy,
        raw_hand.fz - bias_hand.fz,
        raw_hand.mx - bias_hand.mx,
        raw_hand.my - bias_hand.my,
        raw_hand.mz - bias_hand.mz};

    static const std::array<double, 6> arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    arm_w.fx *= arm_sign[0];
    arm_w.fy *= arm_sign[1];
    arm_w.fz *= arm_sign[2];
    arm_w.mx *= arm_sign[3];
    arm_w.my *= arm_sign[4];
    arm_w.mz *= arm_sign[5];

    hand_w.fx *= hand_sign[0];
    hand_w.fy *= hand_sign[1];
    hand_w.fz *= hand_sign[2];
    hand_w.mx *= hand_sign[3];
    hand_w.my *= hand_sign[4];
    hand_w.mz *= hand_sign[5];

    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    arm_w.fx = applyDeadzone(arm_w.fx, force_deadzone);
    arm_w.fy = applyDeadzone(arm_w.fy, force_deadzone);
    arm_w.fz = applyDeadzone(arm_w.fz, force_deadzone);
    arm_w.mx = applyDeadzone(arm_w.mx, torque_deadzone);
    arm_w.my = applyDeadzone(arm_w.my, torque_deadzone);
    arm_w.mz = applyDeadzone(arm_w.mz, torque_deadzone);

    hand_w.fx = applyDeadzone(hand_w.fx, force_deadzone);
    hand_w.fy = applyDeadzone(hand_w.fy, force_deadzone);
    hand_w.fz = applyDeadzone(hand_w.fz, force_deadzone);
    hand_w.mx = applyDeadzone(hand_w.mx, torque_deadzone);
    hand_w.my = applyDeadzone(hand_w.my, torque_deadzone);
    hand_w.mz = applyDeadzone(hand_w.mz, torque_deadzone);

    applyUpperArmSensorWorldAxisGains(arm_w);

    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm
    bool ft_sat = false;
    ft_sat |= clampWrenchWithFlag(arm_w, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(hand_w, force_limit, moment_limit);

    Eigen::VectorXd tau_grav = pinocchio::computeGeneralizedGravity(model, *data, q);
    applyArmGravityCompensation(
        tau_grav, SINGLE_ARM_V_IDS, RIGHT_ARM_GRAVITY_COMPENSATION);

    Eigen::VectorXd tau_fric = Eigen::VectorXd::Zero(model.nv);
    addArmFrictionCompensation(
        tau_fric, v, SINGLE_ARM_V_IDS, RIGHT_ARM_FRICTION_COMPENSATION);

    Eigen::VectorXd g = tau_grav + tau_fric;

    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    pinocchio::Force f_arm = pinocchio::Force::Zero();
    f_arm.linear() << arm_w.fx, arm_w.fy, arm_w.fz;
    f_arm.angular() << arm_w.mx, arm_w.my, arm_w.mz;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << hand_w.fx, hand_w.fy, hand_w.fz;
    f_hand.angular() << hand_w.mx, hand_w.my, hand_w.mz;

    Eigen::VectorXd tau_ext =
        J_arm.transpose() * f_arm.toVector() +
        J_hand.transpose() * f_hand.toVector();

    Eigen::VectorXd damping = Eigen::VectorXd::Zero(model.nv);
    damping(0) = 0.0;
    damping(1) = 0.4;
    damping(2) = 0.3;
    damping(3) = 0.0;
    damping(4) = 0.0;
    damping(5) = 0.2;
    damping(6) = 0.3;

    Eigen::VectorXd tau_damping = -(damping.asDiagonal() * v);
    Eigen::VectorXd tau = g + tau_ext + tau_damping;

    // 扭矩限幅（单轴额定扭矩 61/61/43.7/49/43.7/36.4/18 Nm，避免 SG1 重力>30Nm 与 GH1 拖动饱和）
    const std::array<double, 7> torque_limit = {
        50.0, 30.0, 22.0, 20.0, 16.0, 16.0, 10.0};

    // 右臂输出保留原 -1.0 方向修正
    for (std::size_t i = 0; i < 7; ++i)
    {
      double t_out = -tau(static_cast<Eigen::Index>(i));
      const bool sat = clampAbsWithFlag(t_out, torque_limit[i]);

      const std::size_t idx = right_ctrl_id[i];
      output.use_torque[idx] = true;
      output.target_torque[idx] = t_out;

      if (diag != nullptr)
      {
        diag->tau_int_nm[idx] = tau_ext(static_cast<Eigen::Index>(i));
        diag->tau_grav_nm[idx] = tau_grav(static_cast<Eigen::Index>(i));
        diag->tau_fric_nm[idx] = tau_fric(static_cast<Eigen::Index>(i));
        diag->saturated[idx] = sat;
      }
    }

    if (diag != nullptr)
    {
      diag->proc_ft[SENSOR_RIGHT_ARM] = arm_w;
      diag->proc_ft[SENSOR_RIGHT_HAND] = hand_w;
      diag->ft_saturated = ft_sat;
    }

    output.valid = true;
  }

  // ===================== 双侧零力拖动模式（MODE_BILATERAL_ZERO_FORCE = 9） =====================
  void runBilateralZeroForceMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    output = ControlOutputCache{};
    output.valid = false;

    constexpr std::size_t IDX_R_J18 = 7;
    constexpr std::size_t IDX_R_J19 = 8;
    constexpr std::size_t IDX_R_J20 = 9;
    constexpr std::size_t IDX_L_J8 = 17;
    constexpr std::size_t IDX_L_J9 = 18;
    constexpr std::size_t IDX_L_J10 = 19;

    const std::array<std::size_t, 7> left_ctrl_id = {10, 11, 12, 13, 14, 15, 16};
    const std::array<std::size_t, 7> right_ctrl_id = {0, 1, 2, 3, 4, 5, 6};

    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;

    static pinocchio::FrameIndex left_frame_id_arm = 0;
    static pinocchio::FrameIndex left_frame_id_hand = 0;
    static pinocchio::FrameIndex right_frame_id_arm = 0;
    static pinocchio::FrameIndex right_frame_id_hand = 0;

    static std::array<int, 7> left_q_id = {-1, -1, -1, -1, -1, -1, -1};
    static std::array<int, 7> left_v_id = {-1, -1, -1, -1, -1, -1, -1};
    static std::array<int, 7> right_q_id = {-1, -1, -1, -1, -1, -1, -1};
    static std::array<int, 7> right_v_id = {-1, -1, -1, -1, -1, -1, -1};

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/robot_arms.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        left_frame_id_arm = model.getFrameId("ft_sensor_frame");
        left_frame_id_hand = model.getFrameId("ft_sensor_frame_2");
        right_frame_id_arm = model.getFrameId("ft_rightsensor_frame");
        right_frame_id_hand = model.getFrameId("ft_rightsensor_frame_2");

        auto load_joint_index = [&](const std::string &joint_name, int &q_id, int &v_id) -> bool
        {
          const pinocchio::JointIndex jid = model.getJointId(joint_name);
          if (jid >= model.joints.size())
          {
            std::cerr << "[runBilateralZeroForceMode] 双臂 URDF 没找到 joint: "
                      << joint_name << std::endl;
            return false;
          }
          q_id = model.joints[jid].idx_q();
          v_id = model.joints[jid].idx_v();
          if (q_id < 0 || q_id >= model.nq || v_id < 0 || v_id >= model.nv)
          {
            std::cerr << "[runBilateralZeroForceMode] joint 索引非法: "
                      << joint_name << std::endl;
            return false;
          }
          return true;
        };

        bool ok = true;
        ok = ok && load_joint_index("joint1", left_q_id[0], left_v_id[0]);
        ok = ok && load_joint_index("joint2", left_q_id[1], left_v_id[1]);
        ok = ok && load_joint_index("joint3", left_q_id[2], left_v_id[2]);
        ok = ok && load_joint_index("joint4", left_q_id[3], left_v_id[3]);
        ok = ok && load_joint_index("joint5", left_q_id[4], left_v_id[4]);
        ok = ok && load_joint_index("joint6", left_q_id[5], left_v_id[5]);
        ok = ok && load_joint_index("joint7", left_q_id[6], left_v_id[6]);

        ok = ok && load_joint_index("joint11", right_q_id[0], right_v_id[0]);
        ok = ok && load_joint_index("joint12", right_q_id[1], right_v_id[1]);
        ok = ok && load_joint_index("joint13", right_q_id[2], right_v_id[2]);
        ok = ok && load_joint_index("joint14", right_q_id[3], right_v_id[3]);
        ok = ok && load_joint_index("joint15", right_q_id[4], right_v_id[4]);
        ok = ok && load_joint_index("joint16", right_q_id[5], right_v_id[5]);
        ok = ok && load_joint_index("joint17", right_q_id[6], right_v_id[6]);

        if (left_frame_id_arm >= model.frames.size() ||
            left_frame_id_hand >= model.frames.size() ||
            right_frame_id_arm >= model.frames.size() ||
            right_frame_id_hand >= model.frames.size() ||
            !ok || model.nq != 14 || model.nv != 14)
        {
          std::cerr << "[runBilateralZeroForceMode] 双臂整体 Pinocchio 模型检查失败。"
                    << " nq=" << model.nq << " nv=" << model.nv << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout << "[runBilateralZeroForceMode] 双臂整体 Pinocchio 初始化成功。"
                    << " urdf_path=" << urdf_path
                    << " nq=" << model.nq << " nv=" << model.nv
                    << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runBilateralZeroForceMode] 双臂 Pinocchio 初始化异常: "
                  << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      q(left_q_id[i]) = state.position[left_ctrl_id[i]] * DEG2RAD;
      v(left_v_id[i]) = state.velocity[left_ctrl_id[i]] * DEG2RAD;
      q(right_q_id[i]) = state.position[right_ctrl_id[i]] * DEG2RAD;
      v(right_v_id[i]) = state.velocity[right_ctrl_id[i]] * DEG2RAD;
    }

    auto read_wrench = [&](std::size_t sensor_id) -> ForceSensorData
    {
      const auto &raw = sensor_state.sensor[sensor_id];
      const auto &bias = sensor_state.bias[sensor_id];
      return {
          raw.fx - bias.fx, raw.fy - bias.fy, raw.fz - bias.fz,
          raw.mx - bias.mx, raw.my - bias.my, raw.mz - bias.mz};
    };

    ForceSensorData left_arm = read_wrench(SENSOR_LEFT_ARM);
    ForceSensorData left_hand = read_wrench(SENSOR_LEFT_HAND);
    ForceSensorData right_arm = read_wrench(SENSOR_RIGHT_ARM);
    ForceSensorData right_hand = read_wrench(SENSOR_RIGHT_HAND);

    static const std::array<double, 6> left_arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> left_hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> right_arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> right_hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    auto apply_sign = [](ForceSensorData &w, const std::array<double, 6> &sign)
    {
      w.fx *= sign[0];
      w.fy *= sign[1];
      w.fz *= sign[2];
      w.mx *= sign[3];
      w.my *= sign[4];
      w.mz *= sign[5];
    };

    apply_sign(left_arm, left_arm_sign);
    apply_sign(left_hand, left_hand_sign);
    apply_sign(right_arm, right_arm_sign);
    apply_sign(right_hand, right_hand_sign);

    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    auto apply_wrench_dz = [&](ForceSensorData &w)
    {
      w.fx = applyDeadzone(w.fx, force_deadzone);
      w.fy = applyDeadzone(w.fy, force_deadzone);
      w.fz = applyDeadzone(w.fz, force_deadzone);
      w.mx = applyDeadzone(w.mx, torque_deadzone);
      w.my = applyDeadzone(w.my, torque_deadzone);
      w.mz = applyDeadzone(w.mz, torque_deadzone);
    };

    apply_wrench_dz(left_arm);
    apply_wrench_dz(left_hand);
    apply_wrench_dz(right_arm);
    apply_wrench_dz(right_hand);

    applyUpperArmSensorWorldAxisGains(left_arm);
    applyUpperArmSensorWorldAxisGains(right_arm);

    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm
    bool ft_sat = false;
    ft_sat |= clampWrenchWithFlag(left_arm, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(left_hand, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(right_arm, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(right_hand, force_limit, moment_limit);

    // 双臂整体动力学计算
    Eigen::VectorXd tau_grav = pinocchio::computeGeneralizedGravity(model, *data, q);
    applyArmGravityCompensation(tau_grav, left_v_id, LEFT_ARM_GRAVITY_COMPENSATION);
    applyArmGravityCompensation(tau_grav, right_v_id, RIGHT_ARM_GRAVITY_COMPENSATION);

    Eigen::VectorXd tau_fric = Eigen::VectorXd::Zero(model.nv);
    addArmFrictionCompensation(tau_fric, v, left_v_id, LEFT_ARM_FRICTION_COMPENSATION);
    addArmFrictionCompensation(tau_fric, v, right_v_id, RIGHT_ARM_FRICTION_COMPENSATION);

    Eigen::VectorXd g = tau_grav + tau_fric;

    Eigen::MatrixXd J_left_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_left_hand = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_right_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_right_hand = Eigen::MatrixXd::Zero(6, model.nv);

    pinocchio::computeFrameJacobian(model, *data, q, left_frame_id_arm, pinocchio::LOCAL, J_left_arm);
    pinocchio::computeFrameJacobian(model, *data, q, left_frame_id_hand, pinocchio::LOCAL, J_left_hand);
    pinocchio::computeFrameJacobian(model, *data, q, right_frame_id_arm, pinocchio::LOCAL, J_right_arm);
    pinocchio::computeFrameJacobian(model, *data, q, right_frame_id_hand, pinocchio::LOCAL, J_right_hand);

    auto make_force = [](const ForceSensorData &w) -> pinocchio::Force
    {
      pinocchio::Force f = pinocchio::Force::Zero();
      f.linear() << w.fx, w.fy, w.fz;
      f.angular() << w.mx, w.my, w.mz;
      return f;
    };

    Eigen::VectorXd tau_ext =
        J_left_arm.transpose() * make_force(left_arm).toVector() +
        J_left_hand.transpose() * make_force(left_hand).toVector() +
        J_right_arm.transpose() * make_force(right_arm).toVector() +
        J_right_hand.transpose() * make_force(right_hand).toVector();

    // 左右臂独立净粘性阻尼参数（与原程序完全一致）
    constexpr std::array<double, 7> left_net_damping = {
        0.20, 0.40, 0.40, 0.40, 0.40, 0.40, 0.40};
    constexpr std::array<double, 7> right_net_damping = {
        0.20, 0.40, 0.40, 0.40, 0.40, 0.40, 0.40};

    Eigen::VectorXd stabilizing_damping = Eigen::VectorXd::Zero(model.nv);
    for (std::size_t i = 0; i < 7; ++i)
    {
      stabilizing_damping(left_v_id[i]) =
          LEFT_ARM_FRICTION_COMPENSATION.viscous_nm_per_rad_s[i] +
          left_net_damping[i];
      stabilizing_damping(right_v_id[i]) =
          RIGHT_ARM_FRICTION_COMPENSATION.viscous_nm_per_rad_s[i] +
          right_net_damping[i];
    }

    Eigen::VectorXd tau_stabilizing_damping =
        -(stabilizing_damping.asDiagonal() * v);

    Eigen::VectorXd tau = g + tau_ext + tau_stabilizing_damping;

    const std::array<double, 7> torque_limit = {
        50.0, 30.0, 30.0, 30.0, 18.0, 18.0, 18.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      // 左臂扭矩（+1.0）
      double t_l = tau(left_v_id[i]);
      const bool sat_l = clampAbsWithFlag(t_l, torque_limit[i]);
      output.use_torque[left_ctrl_id[i]] = true;
      output.target_torque[left_ctrl_id[i]] = t_l;

      // 右臂扭矩（-1.0 驱动方向修正）
      double t_r = -tau(right_v_id[i]);
      const bool sat_r = clampAbsWithFlag(t_r, torque_limit[i]);
      output.use_torque[right_ctrl_id[i]] = true;
      output.target_torque[right_ctrl_id[i]] = t_r;

      if (diag != nullptr)
      {
        diag->tau_int_nm[left_ctrl_id[i]] = tau_ext(left_v_id[i]);
        diag->tau_grav_nm[left_ctrl_id[i]] = tau_grav(left_v_id[i]);
        diag->tau_fric_nm[left_ctrl_id[i]] = tau_fric(left_v_id[i]);
        diag->saturated[left_ctrl_id[i]] = sat_l;

        diag->tau_int_nm[right_ctrl_id[i]] = tau_ext(right_v_id[i]);
        diag->tau_grav_nm[right_ctrl_id[i]] = tau_grav(right_v_id[i]);
        diag->tau_fric_nm[right_ctrl_id[i]] = tau_fric(right_v_id[i]);
        diag->saturated[right_ctrl_id[i]] = sat_r;
      }
    }

    if (diag != nullptr)
    {
      diag->proc_ft[SENSOR_LEFT_ARM] = left_arm;
      diag->proc_ft[SENSOR_LEFT_HAND] = left_hand;
      diag->proc_ft[SENSOR_RIGHT_ARM] = right_arm;
      diag->proc_ft[SENSOR_RIGHT_HAND] = right_hand;
      diag->ft_saturated = ft_sat;
    }

    // 末端关节固定输出（执行层会按主动轴过滤，保持原结构一致）
    output.use_position[IDX_R_J18] = true;
    output.target_position[IDX_R_J18] = 0.0;
    output.target_velocity[IDX_R_J18] = pulsePerMsToJointSpeedDegPerSecMagnitude(8, 1500.0);
    output.target_acc[IDX_R_J18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(8, 5.0);
    output.target_dec[IDX_R_J18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(8, 0.25);
    output.target_smooth_time[IDX_R_J18] = 25;

    output.use_position[IDX_R_J19] = true;
    output.target_position[IDX_R_J19] = 0.0;
    output.target_velocity[IDX_R_J19] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 1500.0);
    output.target_acc[IDX_R_J19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
    output.target_dec[IDX_R_J19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
    output.target_smooth_time[IDX_R_J19] = 25;

    output.use_position[IDX_R_J20] = true;
    output.target_position[IDX_R_J20] = 0.0;
    output.target_velocity[IDX_R_J20] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 1500.0);
    output.target_acc[IDX_R_J20] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
    output.target_dec[IDX_R_J20] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
    output.target_smooth_time[IDX_R_J20] = 25;

    output.use_position[IDX_L_J8] = true;
    output.target_position[IDX_L_J8] = 0.0;
    output.target_velocity[IDX_L_J8] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[IDX_L_J8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[IDX_L_J8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[IDX_L_J8] = 25;

    output.use_position[IDX_L_J9] = true;
    output.target_position[IDX_L_J9] = 0.0;
    output.target_velocity[IDX_L_J9] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[IDX_L_J9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[IDX_L_J9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[IDX_L_J9] = 25;

    output.use_position[IDX_L_J10] = true;
    output.target_position[IDX_L_J10] = 0.0;
    output.target_velocity[IDX_L_J10] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[IDX_L_J10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[IDX_L_J10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[IDX_L_J10] = 25;

    output.valid = true;
  }

  // ===================== 左臂预设位姿（严禁改动，完全原样保留） =====================
  void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;

    output = ControlOutputCache{};
    output.valid = true;

    // ===================== 左臂 物理轴11 -> 控制层索引10 =====================
    output.use_position[10] = true;
    output.target_position[10] = 0.0;
    output.target_velocity[10] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
    output.target_acc[10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
    output.target_dec[10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
    output.target_smooth_time[10] = 25;

    // ===================== 左臂 物理轴12 -> 控制层索引11 =====================
    output.use_position[11] = true;
    output.target_position[11] = 0.0;
    output.target_velocity[11] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
    output.target_acc[11] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
    output.target_dec[11] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
    output.target_smooth_time[11] = 25;

    // ===================== 左臂 物理轴13 -> 控制层索引12 =====================
    output.use_position[12] = true;
    output.target_position[12] = 0.0;
    output.target_velocity[12] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
    output.target_acc[12] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
    output.target_dec[12] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
    output.target_smooth_time[12] = 25;

    // ===================== 左臂 物理轴14 -> 控制层索引13 =====================
    output.use_position[13] = true;
    output.target_position[13] = 0.0;
    output.target_velocity[13] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1500.0);
    output.target_acc[13] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 5.0);
    output.target_dec[13] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 5.0);
    output.target_smooth_time[13] = 25;

    // ===================== 左臂 物理轴15 -> 控制层索引14 =====================
    output.use_position[14] = true;
    output.target_position[14] = 0.0;
    output.target_velocity[14] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1500.0);
    output.target_acc[14] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 5.0);
    output.target_dec[14] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 5.0);
    output.target_smooth_time[14] = 25;

    // ===================== 左臂 物理轴16 -> 控制层索引15 =====================
    output.use_position[15] = true;
    output.target_position[15] = 0.0;
    output.target_velocity[15] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1500.0);
    output.target_acc[15] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 5.0);
    output.target_dec[15] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 5.0);
    output.target_smooth_time[15] = 25;

    // ===================== 左臂 物理轴17 -> 控制层索引16 =====================
    output.use_position[16] = true;
    output.target_position[16] = 45.0;
    output.target_velocity[16] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1500.0);
    output.target_acc[16] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 5.0);
    output.target_dec[16] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 0.25);
    output.target_smooth_time[16] = 25;

    // ===================== 左臂 物理轴18 -> 控制层索引17 =====================
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

    // ===================== 左臂 物理轴19 -> 控制层索引18 =====================
    output.use_position[18] = true;
    output.target_position[18] = 0.0;
    output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[18] = 25;

    // ===================== 左臂 物理轴20 -> 控制层索引19 =====================
    output.use_position[19] = true;
    output.target_position[19] = 0.0;
    output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[19] = 25;
  }

  // ===================== 右臂预设位姿（严禁改动，完全原样保留） =====================
  void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;

    output = ControlOutputCache{};
    output.valid = true;

    // ===================== 右臂 物理轴1 -> 控制层索引0 =====================
    output.use_position[0] = true;
    output.target_position[0] = 0.0;
    output.target_velocity[0] = pulsePerMsToJointSpeedDegPerSecMagnitude(1, 200.0);
    output.target_acc[0] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(1, 0.5);
    output.target_dec[0] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(1, 0.25);
    output.target_smooth_time[0] = 25;

    // ===================== 右臂 物理轴2 -> 控制层索引1 =====================
    output.use_position[1] = true;
    output.target_position[1] = 0.0;
    output.target_velocity[1] = pulsePerMsToJointSpeedDegPerSecMagnitude(2, 500.0);
    output.target_acc[1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(2, 0.5);
    output.target_dec[1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(2, 0.25);
    output.target_smooth_time[1] = 25;

    // ===================== 右臂 物理轴3 -> 控制层索引2 =====================
    output.use_position[2] = true;
    output.target_position[2] = 0.0;
    output.target_velocity[2] = pulsePerMsToJointSpeedDegPerSecMagnitude(3, 500.0);
    output.target_acc[2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(3, 0.5);
    output.target_dec[2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(3, 0.25);
    output.target_smooth_time[2] = 25;

    // ===================== 右臂 物理轴4 -> 控制层索引3 =====================
    output.use_position[3] = true;
    output.target_position[3] = 0.0;
    output.target_velocity[3] = pulsePerMsToJointSpeedDegPerSecMagnitude(4, 800.0);
    output.target_acc[3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(4, 0.5);
    output.target_dec[3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(4, 0.25);
    output.target_smooth_time[3] = 25;

    // ===================== 右臂 物理轴5 -> 控制层索引4 =====================
    output.use_position[4] = true;
    output.target_position[4] = 0.0;
    output.target_velocity[4] = pulsePerMsToJointSpeedDegPerSecMagnitude(5, 600.0);
    output.target_acc[4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(5, 5.0);
    output.target_dec[4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(5, 0.25);
    output.target_smooth_time[4] = 25;

    // ===================== 右臂 物理轴6 -> 控制层索引5 =====================
    output.use_position[5] = true;
    output.target_position[5] = 0.0;
    output.target_velocity[5] = pulsePerMsToJointSpeedDegPerSecMagnitude(6, 600.0);
    output.target_acc[5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(6, 5.0);
    output.target_dec[5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(6, 0.25);
    output.target_smooth_time[5] = 25;

    // ===================== 右臂 物理轴7 -> 控制层索引6 =====================
    output.use_position[6] = true;
    output.target_position[6] = 45.0;
    output.target_velocity[6] = pulsePerMsToJointSpeedDegPerSecMagnitude(7, 3600.0);
    output.target_acc[6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(7, 5.0);
    output.target_dec[6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(7, 0.25);
    output.target_smooth_time[6] = 25;

    // 控制层索引7 当前未设置，保持默认值

    // ===================== 右臂 物理轴9 -> 控制层索引8 =====================
    output.use_position[8] = true;
    output.target_position[8] = 0.0;
    output.target_velocity[8] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 9000.0);
    output.target_acc[8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
    output.target_dec[8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
    output.target_smooth_time[8] = 25;

    // ===================== 右臂 物理轴10 -> 控制层索引9 =====================
    output.use_position[9] = true;
    output.target_position[9] = 0.0;
    output.target_velocity[9] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 3000.0);
    output.target_acc[9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
    output.target_dec[9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
    output.target_smooth_time[9] = 25;
  }

  // ===================== 左臂正弦被动跟踪模式（PID，MODE_PASSIVE = 5） =====================
  //
  // 防跳变最小改动说明：
  // 1. 状态变量移出函数内 static，并在进入模式时调用 resetPassiveMode()，避免重复进入沿用旧起点；
  // 2. 引入 2.0 s 余弦渐入包络 ramp(t) = 0.5*(1 - cos(pi*u))，消除 J5 phase=pi/2 在 t=0 时的 12° 阶跃跳变；
  // 3. 首帧令 e_prev = e（使首帧 de = 0），消除起步微分尖峰；
  // 4. 统一使用控制环传入的 actual_dt_s（限幅 0.002~0.05 s）。
  //
  void runPassiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    (void)sensor_state;

    output = ControlOutputCache{};
    output.valid = false;

    constexpr std::size_t IDX_J1 = 10;
    constexpr std::size_t IDX_J2 = 11;
    constexpr std::size_t IDX_J3 = 12;
    constexpr std::size_t IDX_J4 = 13;
    constexpr std::size_t IDX_J5 = 14;
    constexpr std::size_t IDX_J6 = 15;
    constexpr std::size_t IDX_J7 = 16;
    const std::array<std::size_t, 7> left_ctrl_id = {
        IDX_J1, IDX_J2, IDX_J3, IDX_J4, IDX_J5, IDX_J6, IDX_J7};

    const double dt = clampControlDt(dt_s);

    if (!passive_pid_initialized)
    {
      for (std::size_t i = 0; i < 7; ++i)
      {
        passive_pid_q0[i] = state.position[left_ctrl_id[i]];
      }
      passive_pid_elapsed_t = 0.0;
      passive_pid_e_int.fill(0.0);
      passive_pid_e_prev.fill(0.0);
      passive_pid_de_filt.fill(0.0);
      passive_pid_first_step = true;
      passive_pid_initialized = true;
    }
    else
    {
      passive_pid_elapsed_t += dt;
    }

    const double t = passive_pid_elapsed_t;

    // 2.0 s 余弦渐入包络：t=0 时 ramp=0，t>=2.0 s 时 ramp=1（与原正弦轨迹完全一致）
    constexpr double RAMP_DURATION_S = 2.0;
    const double u = clampBetween(t / RAMP_DURATION_S, 0.0, 1.0);
    const double ramp = 0.5 * (1.0 - std::cos(M_PI * u));

    // 根据实验参数选择单关节 / 组合关节，以及两档低速
    const int joint_profile = g_exp_params.passive_joint_profile;
    const double speed_scale = getPassiveSpeedScale(g_exp_params.passive_speed_level);

    const std::array<double, 4> base_amp = {8.0, 12.0, 10.0, 15.0}; // J4, J5, J6, J7 (deg)
    const std::array<double, 4> base_freq = {0.10, 0.10, 0.10, 0.12}; // Hz
    const std::array<double, 4> phase = {0.0, M_PI / 2.0, 0.0, 0.0};
    const std::array<double, 4> rel_limit = {15.0, 20.0, 20.0, 25.0}; // 相对 q0 安全限幅 (deg)

    std::array<double, 4> amp = base_amp;
    if (joint_profile == PASSIVE_JOINT_J4_ONLY)
      amp = {base_amp[0], 0.0, 0.0, 0.0};
    else if (joint_profile == PASSIVE_JOINT_J5_ONLY)
      amp = {0.0, base_amp[1], 0.0, 0.0};
    else if (joint_profile == PASSIVE_JOINT_J6_ONLY)
      amp = {0.0, 0.0, base_amp[2], 0.0};
    else if (joint_profile == PASSIVE_JOINT_J7_ONLY)
      amp = {0.0, 0.0, 0.0, base_amp[3]};

    // PID 参数（基于 2026-11-20 实机频谱分析整定）：
    // 位置外环存在约 60~95 ms 内环纯滞后，离散特征方程 1 + Kp*z^(-d) = 0 要求 Kp << 1.0；
    // 原 Kp=0.8 接近临界且 Kp=1.2 会激发 5.3~8.3 Hz 抖动。
    // 现改为低增益顺滑参数：Kp=0.30、Ki=0.05（紧限幅防饱和）、Kd=0.0，配合 smooth_time=25 消除抖动。
    constexpr double Kp = 0.30;
    constexpr double Ki = 0.05;
    constexpr double Kd = 0.00;
    constexpr double DE_LPF_ALPHA = 0.20;

    // 背部 joint1~3 固定在进入模式时的初始角
    for (std::size_t i = 0; i < 3; ++i)
    {
      const std::size_t idx = left_ctrl_id[i];
      output.use_position[idx] = true;
      output.target_position[idx] = passive_pid_q0[i];
      if (diag != nullptr)
      {
        diag->qd_deg[idx] = passive_pid_q0[i];
      }
    }

    // 肩 joint4~6、肘 joint7 正弦被动 PID 跟踪
    for (std::size_t k = 0; k < 4; ++k)
    {
      const std::size_t idx = left_ctrl_id[k + 3];
      const double q0 = passive_pid_q0[k + 3];
      const double q_meas = state.position[idx];
      const double w = 2.0 * M_PI * (base_freq[k] * speed_scale);

      const double q_ref = q0 + ramp * amp[k] * std::sin(w * t + phase[k]);
      const double e = q_ref - q_meas;

      passive_pid_e_int[k] = clampBetween(passive_pid_e_int[k] + e * dt, -2.0, 2.0);
      if (passive_pid_first_step)
      {
        passive_pid_e_prev[k] = e;
        passive_pid_de_filt[k] = 0.0;
      }
      const double de_raw = (e - passive_pid_e_prev[k]) / dt;
      passive_pid_e_prev[k] = e;
      passive_pid_de_filt[k] =
          DE_LPF_ALPHA * de_raw + (1.0 - DE_LPF_ALPHA) * passive_pid_de_filt[k];

      double q_cmd = q_ref + Kp * e + Ki * passive_pid_e_int[k] + Kd * passive_pid_de_filt[k];
      const bool sat = clampBetweenWithFlag(q_cmd, q0 - rel_limit[k], q0 + rel_limit[k]);

      output.use_position[idx] = true;
      output.target_position[idx] = q_cmd;

      if (diag != nullptr)
      {
        diag->qd_deg[idx] = q_ref;
        diag->saturated[idx] = sat;
      }
    }
    passive_pid_first_step = false;

    // 速度/加速度参数（恢复原平滑 S 曲线 smooth_time=25、acc/dec=3.0，保证轨迹顺滑无抖动）
    output.target_velocity[IDX_J1] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
    output.target_acc[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
    output.target_dec[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
    output.target_smooth_time[IDX_J1] = 25;

    output.target_velocity[IDX_J2] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
    output.target_acc[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
    output.target_dec[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
    output.target_smooth_time[IDX_J2] = 25;

    output.target_velocity[IDX_J3] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
    output.target_acc[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
    output.target_dec[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
    output.target_smooth_time[IDX_J3] = 25;

    output.target_velocity[IDX_J4] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1200.0);
    output.target_acc[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_dec[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_smooth_time[IDX_J4] = 25;

    output.target_velocity[IDX_J5] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1200.0);
    output.target_acc[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_dec[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_smooth_time[IDX_J5] = 25;

    output.target_velocity[IDX_J6] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1200.0);
    output.target_acc[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_dec[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_smooth_time[IDX_J6] = 25;

    output.target_velocity[IDX_J7] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1200.0);
    output.target_acc[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_dec[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_smooth_time[IDX_J7] = 25;

    // 末端关节固定
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

    output.use_position[18] = true;
    output.target_position[18] = 0.0;
    output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[18] = 25;

    output.use_position[19] = true;
    output.target_position[19] = 0.0;
    output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[19] = 25;

    output.valid = true;
  }

  // ===================== 左臂正弦被动跟踪模式（PD+前馈，MODE_PASSIVE_SINE_PD_FF = 6） =====================
  //
  // 防跳变最小改动说明：
  // 1. 状态变量移出函数内 static，并在进入模式时调用 resetPassivePDFFMode()；
  // 2. 引入 2.0 s 余弦渐入包络 ramp(t) 及解析导数 dramp_dt，确保 t=0 时位置与速度前馈均从 0 平滑起步；
  // 3. 统一使用控制环传入的 actual_dt_s（限幅 0.002~0.05 s）。
  //
  void runPassivePDFFMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    (void)sensor_state;

    output = ControlOutputCache{};
    output.valid = false;

    constexpr std::size_t IDX_J1 = 10;
    constexpr std::size_t IDX_J2 = 11;
    constexpr std::size_t IDX_J3 = 12;
    constexpr std::size_t IDX_J4 = 13;
    constexpr std::size_t IDX_J5 = 14;
    constexpr std::size_t IDX_J6 = 15;
    constexpr std::size_t IDX_J7 = 16;
    const std::array<std::size_t, 7> left_ctrl_id = {
        IDX_J1, IDX_J2, IDX_J3, IDX_J4, IDX_J5, IDX_J6, IDX_J7};

    const double dt = clampControlDt(dt_s);

    if (!passive_pdff_initialized)
    {
      for (std::size_t i = 0; i < 7; ++i)
      {
        passive_pdff_q0[i] = state.position[left_ctrl_id[i]];
      }
      passive_pdff_elapsed_t = 0.0;
      passive_pdff_initialized = true;
    }
    else
    {
      passive_pdff_elapsed_t += dt;
    }

    const double t = passive_pdff_elapsed_t;

    // 2.0 s 余弦渐入包络及其一阶、二阶解析导数
    constexpr double RAMP_DURATION_S = 2.0;
    const double u = clampBetween(t / RAMP_DURATION_S, 0.0, 1.0);
    const double ramp = 0.5 * (1.0 - std::cos(M_PI * u));
    const double dramp_dt =
        (t < RAMP_DURATION_S)
            ? (0.5 * (M_PI / RAMP_DURATION_S) * std::sin(M_PI * u))
            : 0.0;
    const double ddramp_dt2 =
        (t < RAMP_DURATION_S)
            ? (0.5 * (M_PI / RAMP_DURATION_S) * (M_PI / RAMP_DURATION_S) * std::cos(M_PI * u))
            : 0.0;

    const int joint_profile = g_exp_params.passive_joint_profile;
    const double speed_scale = getPassiveSpeedScale(g_exp_params.passive_speed_level);

    const std::array<double, 4> base_amp = {8.0, 12.0, 10.0, 15.0};
    const std::array<double, 4> base_freq = {0.10, 0.10, 0.10, 0.12};
    const std::array<double, 4> phase = {0.0, M_PI / 2.0, 0.0, 0.0};
    const std::array<double, 4> rel_limit = {15.0, 20.0, 20.0, 25.0};

    std::array<double, 4> amp = base_amp;
    if (joint_profile == PASSIVE_JOINT_J4_ONLY)
      amp = {base_amp[0], 0.0, 0.0, 0.0};
    else if (joint_profile == PASSIVE_JOINT_J5_ONLY)
      amp = {0.0, base_amp[1], 0.0, 0.0};
    else if (joint_profile == PASSIVE_JOINT_J6_ONLY)
      amp = {0.0, 0.0, base_amp[2], 0.0};
    else if (joint_profile == PASSIVE_JOINT_J7_ONLY)
      amp = {0.0, 0.0, 0.0, base_amp[3]};

    // PD + 二阶解析前馈参数（基于 2026-11-20 smooth_time=25 开环对象延迟辨识 qc->q 整定）：
    // 1. 反馈增益 Kp=0.25 远小于纯延迟稳定边界 1.0，Kd=0.0 不引入测速噪声，彻底消除 5~8 Hz 抖动；
    // 2. 用实测 smooth_time=25 开环延迟 Kff_s = {58, 77, 98, 92} ms 做一阶速度 + 二阶加速度解析前馈
    //    q_ref + Kff*dq_ref + 0.5*Kff^2*ddq_ref*ramp（完全开环无噪，不影响闭环稳定性，且 t=0 精确零跳变）；
    // 3. 保持 smooth_time=25、acc/dec=3.0 保证轨迹柔顺平稳。
    constexpr double Kp = 0.25;
    constexpr double Kd = 0.00;
    const std::array<double, 4> Kff_s = {0.058, 0.077, 0.098, 0.092}; // J4(GH1), J5(GH2), J6(GH3), J7(EB1)

    // 背部 joint1~3 固定在进入模式时的初始角
    for (std::size_t i = 0; i < 3; ++i)
    {
      const std::size_t idx = left_ctrl_id[i];
      output.use_position[idx] = true;
      output.target_position[idx] = passive_pdff_q0[i];
      if (diag != nullptr)
      {
        diag->qd_deg[idx] = passive_pdff_q0[i];
      }
    }

    // 肩 joint4~6、肘 joint7 正弦被动 PD+FF 跟踪
    for (std::size_t k = 0; k < 4; ++k)
    {
      const std::size_t idx = left_ctrl_id[k + 3];
      const double q0 = passive_pdff_q0[k + 3];
      const double q_meas = state.position[idx];
      const double dq_meas = state.velocity[idx];
      const double w = 2.0 * M_PI * (base_freq[k] * speed_scale);

      const double s_val = std::sin(w * t + phase[k]);
      const double c_val = std::cos(w * t + phase[k]);

      const double q_ref = q0 + ramp * amp[k] * s_val;
      const double dq_ref = dramp_dt * amp[k] * s_val + ramp * amp[k] * w * c_val;
      const double ddq_ref =
          ddramp_dt2 * amp[k] * s_val +
          2.0 * dramp_dt * amp[k] * w * c_val -
          ramp * amp[k] * w * w * s_val;

      const double e = q_ref - q_meas;
      const double ev = dq_ref - dq_meas;

      const double ff_lead =
          Kff_s[k] * dq_ref + (0.5 * Kff_s[k] * Kff_s[k] * ddq_ref * ramp);
      double q_cmd = q_ref + ff_lead + Kp * e + Kd * ev;
      const bool sat = clampBetweenWithFlag(q_cmd, q0 - rel_limit[k], q0 + rel_limit[k]);

      output.use_position[idx] = true;
      output.target_position[idx] = q_cmd;

      if (diag != nullptr)
      {
        diag->qd_deg[idx] = q_ref;
        diag->saturated[idx] = sat;
      }
    }

    output.target_velocity[IDX_J1] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
    output.target_acc[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
    output.target_dec[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
    output.target_smooth_time[IDX_J1] = 25;

    output.target_velocity[IDX_J2] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
    output.target_acc[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
    output.target_dec[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
    output.target_smooth_time[IDX_J2] = 25;

    output.target_velocity[IDX_J3] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
    output.target_acc[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
    output.target_dec[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
    output.target_smooth_time[IDX_J3] = 25;

    output.target_velocity[IDX_J4] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1200.0);
    output.target_acc[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_dec[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_smooth_time[IDX_J4] = 25;

    output.target_velocity[IDX_J5] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1200.0);
    output.target_acc[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_dec[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_smooth_time[IDX_J5] = 25;

    output.target_velocity[IDX_J6] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1200.0);
    output.target_acc[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_dec[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_smooth_time[IDX_J6] = 25;

    output.target_velocity[IDX_J7] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1200.0);
    output.target_acc[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_dec[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_smooth_time[IDX_J7] = 25;

    // 末端关节固定
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

    output.use_position[18] = true;
    output.target_position[18] = 0.0;
    output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[18] = 25;

    output.use_position[19] = true;
    output.target_position[19] = 0.0;
    output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[19] = 25;

    output.valid = true;
  }

  // ===================== 双侧主动导纳模式（MODE_BILATERAL_ACTIVE = 13） =====================
  void runBilateralActiveMode(
      const MotorStateCache &state,
      const SensorStateCache &sensor_state,
      double dt_s,
      ControlOutputCache &output,
      AlgorithmDiagData *diag)
  {
    output = ControlOutputCache{};
    output.valid = false;

    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / M_PI;
    const double dt = clampControlDt(dt_s);

    const std::array<std::size_t, 7> left_ctrl_id = {10, 11, 12, 13, 14, 15, 16};
    const std::array<std::size_t, 7> right_ctrl_id = {0, 1, 2, 3, 4, 5, 6};

    auto reset_admittance_to_current = [&](const Eigen::VectorXd &q_now)
    {
      bilateral_active_q_adm = q_now;
      bilateral_active_dq_adm = Eigen::VectorXd::Zero(q_now.size());
      bilateral_active_admittance_initialized = true;
    };

    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;

    static pinocchio::FrameIndex left_frame_id_arm = 0;
    static pinocchio::FrameIndex left_frame_id_hand = 0;
    static pinocchio::FrameIndex right_frame_id_arm = 0;
    static pinocchio::FrameIndex right_frame_id_hand = 0;

    static std::array<int, 7> left_q_id = {-1, -1, -1, -1, -1, -1, -1};
    static std::array<int, 7> left_v_id = {-1, -1, -1, -1, -1, -1, -1};
    static std::array<int, 7> right_q_id = {-1, -1, -1, -1, -1, -1, -1};
    static std::array<int, 7> right_v_id = {-1, -1, -1, -1, -1, -1, -1};

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/robot_arms.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        left_frame_id_arm = model.getFrameId("ft_sensor_frame");
        right_frame_id_arm = model.getFrameId("ft_rightsensor_frame");
        left_frame_id_hand = model.getFrameId("ft_sensor_frame_2");
        right_frame_id_hand = model.getFrameId("ft_rightsensor_frame_2");

        auto load_joint_index = [&](const std::string &joint_name, int &q_id, int &v_id) -> bool
        {
          pinocchio::JointIndex jid = model.getJointId(joint_name);
          if (jid >= model.joints.size())
          {
            std::cerr << "[runBilateralActiveMode] 找不到关节: " << joint_name << std::endl;
            return false;
          }
          q_id = model.joints[jid].idx_q();
          v_id = model.joints[jid].idx_v();
          if (q_id < 0 || q_id >= model.nq || v_id < 0 || v_id >= model.nv)
          {
            std::cerr << "[runBilateralActiveMode] 关节索引非法: " << joint_name << std::endl;
            return false;
          }
          return true;
        };

        bool ok = true;
        ok = ok && load_joint_index("joint1", left_q_id[0], left_v_id[0]);
        ok = ok && load_joint_index("joint2", left_q_id[1], left_v_id[1]);
        ok = ok && load_joint_index("joint3", left_q_id[2], left_v_id[2]);
        ok = ok && load_joint_index("joint4", left_q_id[3], left_v_id[3]);
        ok = ok && load_joint_index("joint5", left_q_id[4], left_v_id[4]);
        ok = ok && load_joint_index("joint6", left_q_id[5], left_v_id[5]);
        ok = ok && load_joint_index("joint7", left_q_id[6], left_v_id[6]);

        ok = ok && load_joint_index("joint11", right_q_id[0], right_v_id[0]);
        ok = ok && load_joint_index("joint12", right_q_id[1], right_v_id[1]);
        ok = ok && load_joint_index("joint13", right_q_id[2], right_v_id[2]);
        ok = ok && load_joint_index("joint14", right_q_id[3], right_v_id[3]);
        ok = ok && load_joint_index("joint15", right_q_id[4], right_v_id[4]);
        ok = ok && load_joint_index("joint16", right_q_id[5], right_v_id[5]);
        ok = ok && load_joint_index("joint17", right_q_id[6], right_v_id[6]);

        if (left_frame_id_arm >= model.frames.size() ||
            right_frame_id_arm >= model.frames.size() ||
            left_frame_id_hand >= model.frames.size() ||
            right_frame_id_hand >= model.frames.size() ||
            !ok || model.nq != 14 || model.nv != 14)
        {
          std::cerr << "[runBilateralActiveMode] 双臂整体 Pinocchio 模型检查失败。"
                    << " nq=" << model.nq << " nv=" << model.nv << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout << "[runBilateralActiveMode] 双臂双侧主动导纳初始化成功。"
                    << " urdf_path=" << urdf_path
                    << " nq=" << model.nq << " nv=" << model.nv
                    << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runBilateralActiveMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    // 组装双臂整体 q / v
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      q(left_q_id[i]) = state.position[left_ctrl_id[i]] * DEG2RAD;
      v(left_v_id[i]) = state.velocity[left_ctrl_id[i]] * DEG2RAD;
      q(right_q_id[i]) = state.position[right_ctrl_id[i]] * DEG2RAD;
      v(right_v_id[i]) = state.velocity[right_ctrl_id[i]] * DEG2RAD;
    }

    if (!q.allFinite() || !v.allFinite())
    {
      bilateral_active_fault_counter++;
      bilateral_active_admittance_initialized = false;
      std::cerr << "[runBilateralActiveMode] q/v 出现 NaN 或 Inf，本轮不输出。fault="
                << bilateral_active_fault_counter << std::endl;
      output.valid = false;
      return;
    }

    // q / v 物理合理性检查（与原程序一致）
    const double q_abs_hard_limit_default = 3.8;
    const double v_abs_hard_limit = 12.0;
    Eigen::VectorXd q_abs_limit =
        Eigen::VectorXd::Constant(model.nq, q_abs_hard_limit_default);
    if (right_q_id[0] >= 0 && right_q_id[0] < model.nq)
    {
      q_abs_limit(right_q_id[0]) = 5.2;
    }

    for (int i = 0; i < model.nq; ++i)
    {
      if (!std::isfinite(q(i)) || std::abs(q(i)) > q_abs_limit(i))
      {
        bilateral_active_fault_counter++;
        bilateral_active_admittance_initialized = false;
        std::cerr << "[runBilateralActiveMode] q 超出物理合理范围，index=" << i
                  << " val=" << q(i) << " fault=" << bilateral_active_fault_counter
                  << std::endl;
        output.valid = false;
        return;
      }
    }

    for (int i = 0; i < model.nv; ++i)
    {
      if (!std::isfinite(v(i)) || std::abs(v(i)) > v_abs_hard_limit)
      {
        bilateral_active_fault_counter++;
        bilateral_active_admittance_initialized = false;
        std::cerr << "[runBilateralActiveMode] v 超出物理合理范围，index=" << i
                  << " val=" << v(i) << " fault=" << bilateral_active_fault_counter
                  << std::endl;
        output.valid = false;
        return;
      }
    }

    // 导纳状态初始化
    if (!bilateral_active_admittance_initialized ||
        bilateral_active_q_adm.size() != model.nq ||
        bilateral_active_dq_adm.size() != model.nv)
    {
      reset_admittance_to_current(q);
      std::cout << "[runBilateralActiveMode] 导纳状态已初始化" << std::endl;
    }

    // 读取四个六维力传感器
    auto read_wrench = [&](std::size_t sensor_id) -> ForceSensorData
    {
      const auto &raw = sensor_state.sensor[sensor_id];
      const auto &bias = sensor_state.bias[sensor_id];
      return {
          raw.fx - bias.fx, raw.fy - bias.fy, raw.fz - bias.fz,
          raw.mx - bias.mx, raw.my - bias.my, raw.mz - bias.mz};
    };

    ForceSensorData left_arm = read_wrench(SENSOR_LEFT_ARM);
    ForceSensorData left_hand = read_wrench(SENSOR_LEFT_HAND);
    ForceSensorData right_arm = read_wrench(SENSOR_RIGHT_ARM);
    ForceSensorData right_hand = read_wrench(SENSOR_RIGHT_HAND);

    const double force_deadzone = 1.0;
    const double torque_deadzone = 0.05;
    const double force_limit = 80.0;
    const double moment_limit = 8.0;
    bool ft_sat = false;

    auto process_real_wrench = [&](ForceSensorData &w)
    {
      w.fx = applyDeadzone(w.fx, force_deadzone);
      w.fy = applyDeadzone(w.fy, force_deadzone);
      w.fz = applyDeadzone(w.fz, force_deadzone);
      w.mx = applyDeadzone(w.mx, torque_deadzone);
      w.my = applyDeadzone(w.my, torque_deadzone);
      w.mz = applyDeadzone(w.mz, torque_deadzone);
      ft_sat |= clampWrenchWithFlag(w, force_limit, moment_limit);
    };

    process_real_wrench(left_arm);
    process_real_wrench(left_hand);
    process_real_wrench(right_arm);
    process_real_wrench(right_hand);

    applyUpperArmSensorWorldAxisGains(left_arm);
    applyUpperArmSensorWorldAxisGains(right_arm);
    ft_sat |= clampWrenchWithFlag(left_arm, force_limit, moment_limit);
    ft_sat |= clampWrenchWithFlag(right_arm, force_limit, moment_limit);

    // LOCAL Jacobian 映射
    Eigen::MatrixXd J_left_arm_local = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_left_hand_local = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_right_arm_local = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_right_hand_local = Eigen::MatrixXd::Zero(6, model.nv);

    pinocchio::computeFrameJacobian(model, *data, q, left_frame_id_arm, pinocchio::LOCAL, J_left_arm_local);
    pinocchio::computeFrameJacobian(model, *data, q, left_frame_id_hand, pinocchio::LOCAL, J_left_hand_local);
    pinocchio::computeFrameJacobian(model, *data, q, right_frame_id_arm, pinocchio::LOCAL, J_right_arm_local);
    pinocchio::computeFrameJacobian(model, *data, q, right_frame_id_hand, pinocchio::LOCAL, J_right_hand_local);

    auto make_force = [](const ForceSensorData &w) -> pinocchio::Force
    {
      pinocchio::Force f = pinocchio::Force::Zero();
      f.linear() << w.fx, w.fy, w.fz;
      f.angular() << w.mx, w.my, w.mz;
      return f;
    };

    Eigen::VectorXd tau_ext =
        J_left_arm_local.transpose() * make_force(left_arm).toVector() +
        J_left_hand_local.transpose() * make_force(left_hand).toVector() +
        J_right_arm_local.transpose() * make_force(right_arm).toVector() +
        J_right_hand_local.transpose() * make_force(right_hand).toVector();

    if (!tau_ext.allFinite())
    {
      bilateral_active_fault_counter++;
      reset_admittance_to_current(q);
      std::cerr << "[runBilateralActiveMode] tau_ext 出现 NaN/Inf，已重置导纳，本轮不输出" << std::endl;
      output.valid = false;
      return;
    }

    // 关节空间导纳参数（支持三档阻尼预设）
    const double damping_scale =
        getAdmittanceDampingScale(g_exp_params.admittance_damping_level);

    Eigen::VectorXd M_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd D_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd K_a = Eigen::VectorXd::Zero(model.nv);

    const std::array<double, 7> M_value = {4.0, 4.0, 3.0, 1.0, 1.0, 1.0, 0.5};
    const std::array<double, 7> D_value = {45.0, 45.0, 35.0, 20.0, 20.0, 18.0, 12.0};
    const std::array<double, 7> K_value = {18.0, 18.0, 14.0, 8.0, 8.0, 6.0, 4.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      M_a(left_v_id[i]) = M_value[i];
      D_a(left_v_id[i]) = D_value[i] * damping_scale;
      K_a(left_v_id[i]) = K_value[i];

      M_a(right_v_id[i]) = M_value[i];
      D_a(right_v_id[i]) = D_value[i] * damping_scale;
      K_a(right_v_id[i]) = K_value[i];
    }

    Eigen::VectorXd ddq_adm = Eigen::VectorXd::Zero(model.nv);
    const std::array<double, 7> ddq_limit_value = {5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0};

    for (int i = 0; i < model.nv; ++i)
    {
      if (M_a(i) <= 1e-9)
      {
        ddq_adm(i) = 0.0;
        continue;
      }

      ddq_adm(i) =
          (tau_ext(i) -
           D_a(i) * bilateral_active_dq_adm(i) -
           K_a(i) * (bilateral_active_q_adm(i) - q(i))) /
          M_a(i);
    }

    for (std::size_t i = 0; i < 7; ++i)
    {
      ddq_adm(left_v_id[i]) = clampAbs(ddq_adm(left_v_id[i]), ddq_limit_value[i]);
      ddq_adm(right_v_id[i]) = clampAbs(ddq_adm(right_v_id[i]), ddq_limit_value[i]);
    }

    bilateral_active_dq_adm = bilateral_active_dq_adm + ddq_adm * dt;
    bilateral_active_q_adm = bilateral_active_q_adm + bilateral_active_dq_adm * dt;

    if (!bilateral_active_q_adm.allFinite() || !bilateral_active_dq_adm.allFinite())
    {
      bilateral_active_fault_counter++;
      reset_admittance_to_current(q);
      std::cerr << "[runBilateralActiveMode] 导纳积分出现 NaN/Inf，已重置导纳" << std::endl;
      output.valid = false;
      return;
    }

    const std::array<double, 7> q_min_value = {
        -30.0 * DEG2RAD, -30.0 * DEG2RAD, -30.0 * DEG2RAD,
        -60.0 * DEG2RAD, -60.0 * DEG2RAD, -60.0 * DEG2RAD,
        0.0 * DEG2RAD};
    const std::array<double, 7> q_max_value = {
        30.0 * DEG2RAD, 30.0 * DEG2RAD, 30.0 * DEG2RAD,
        60.0 * DEG2RAD, 60.0 * DEG2RAD, 60.0 * DEG2RAD,
        120.0 * DEG2RAD};
    const std::array<double, 7> dq_limit_value = {1.2, 1.2, 1.2, 2.0, 2.0, 2.0, 2.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      bilateral_active_q_adm(left_q_id[i]) =
          clampBetween(bilateral_active_q_adm(left_q_id[i]), q_min_value[i], q_max_value[i]);
      bilateral_active_q_adm(right_q_id[i]) =
          clampBetween(bilateral_active_q_adm(right_q_id[i]), q_min_value[i], q_max_value[i]);
      bilateral_active_dq_adm(left_v_id[i]) =
          clampAbs(bilateral_active_dq_adm(left_v_id[i]), dq_limit_value[i]);
      bilateral_active_dq_adm(right_v_id[i]) =
          clampAbs(bilateral_active_dq_adm(right_v_id[i]), dq_limit_value[i]);
    }

    // 重力与摩擦补偿
    Eigen::VectorXd tau_grav = pinocchio::computeGeneralizedGravity(model, *data, q);
    applyArmGravityCompensation(tau_grav, left_v_id, LEFT_ARM_GRAVITY_COMPENSATION);
    applyArmGravityCompensation(tau_grav, right_v_id, RIGHT_ARM_GRAVITY_COMPENSATION);

    Eigen::VectorXd tau_fric = Eigen::VectorXd::Zero(model.nv);
    addArmFrictionCompensation(tau_fric, v, left_v_id, LEFT_ARM_FRICTION_COMPENSATION);
    addArmFrictionCompensation(tau_fric, v, right_v_id, RIGHT_ARM_FRICTION_COMPENSATION);

    Eigen::VectorXd g = tau_grav + tau_fric;

    if (!g.allFinite())
    {
      bilateral_active_fault_counter++;
      reset_admittance_to_current(q);
      std::cerr << "[runBilateralActiveMode] 重力补偿出现 NaN/Inf，已重置导纳" << std::endl;
      output.valid = false;
      return;
    }

    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd Kv = Eigen::VectorXd::Zero(model.nv);

    const std::array<double, 7> Kp_value = {20.0, 20.0, 16.0, 15.0, 15.0, 13.0, 8.0};
    const std::array<double, 7> Kv_value = {4.0, 4.0, 3.0, 2.5, 2.5, 2.0, 1.5};

    for (std::size_t i = 0; i < 7; ++i)
    {
      Kp(left_v_id[i]) = Kp_value[i];
      Kv(left_v_id[i]) = Kv_value[i];
      Kp(right_v_id[i]) = Kp_value[i];
      Kv(right_v_id[i]) = Kv_value[i];
    }

    Eigen::VectorXd tau_cmd =
        g +
        Kp.asDiagonal() * (bilateral_active_q_adm - q) +
        Kv.asDiagonal() * (bilateral_active_dq_adm - v);

    if (!tau_cmd.allFinite())
    {
      bilateral_active_fault_counter++;
      reset_admittance_to_current(q);
      std::cerr << "[runBilateralActiveMode] tau_cmd 出现 NaN/Inf，已重置导纳" << std::endl;
      output.valid = false;
      return;
    }

    const std::array<double, 7> torque_limit = {
        50.0, 30.0, 22.0, 20.0, 16.0, 16.0, 10.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      // 左臂扭矩（+1.0）
      double t_l = tau_cmd(left_v_id[i]);
      const bool sat_l = clampAbsWithFlag(t_l, torque_limit[i]);
      output.use_torque[left_ctrl_id[i]] = true;
      output.target_torque[left_ctrl_id[i]] = t_l;

      // 右臂扭矩（-1.0 驱动方向修正）
      double t_r = -tau_cmd(right_v_id[i]);
      const bool sat_r = clampAbsWithFlag(t_r, torque_limit[i]);
      output.use_torque[right_ctrl_id[i]] = true;
      output.target_torque[right_ctrl_id[i]] = t_r;

      if (diag != nullptr)
      {
        diag->qd_deg[left_ctrl_id[i]] = bilateral_active_q_adm(left_q_id[i]) * RAD2DEG;
        diag->qa_deg[left_ctrl_id[i]] = bilateral_active_q_adm(left_q_id[i]) * RAD2DEG;
        diag->dqa_deg_s[left_ctrl_id[i]] = bilateral_active_dq_adm(left_v_id[i]) * RAD2DEG;
        diag->ddqa_deg_s2[left_ctrl_id[i]] = ddq_adm(left_v_id[i]) * RAD2DEG;
        diag->tau_int_nm[left_ctrl_id[i]] = tau_ext(left_v_id[i]);
        diag->tau_grav_nm[left_ctrl_id[i]] = tau_grav(left_v_id[i]);
        diag->tau_fric_nm[left_ctrl_id[i]] = tau_fric(left_v_id[i]);
        diag->saturated[left_ctrl_id[i]] = sat_l;

        diag->qd_deg[right_ctrl_id[i]] = bilateral_active_q_adm(right_q_id[i]) * RAD2DEG;
        diag->qa_deg[right_ctrl_id[i]] = bilateral_active_q_adm(right_q_id[i]) * RAD2DEG;
        diag->dqa_deg_s[right_ctrl_id[i]] = bilateral_active_dq_adm(right_v_id[i]) * RAD2DEG;
        diag->ddqa_deg_s2[right_ctrl_id[i]] = ddq_adm(right_v_id[i]) * RAD2DEG;
        diag->tau_int_nm[right_ctrl_id[i]] = tau_ext(right_v_id[i]);
        diag->tau_grav_nm[right_ctrl_id[i]] = tau_grav(right_v_id[i]);
        diag->tau_fric_nm[right_ctrl_id[i]] = tau_fric(right_v_id[i]);
        diag->saturated[right_ctrl_id[i]] = sat_r;
      }
    }

    if (diag != nullptr)
    {
      diag->proc_ft[SENSOR_LEFT_ARM] = left_arm;
      diag->proc_ft[SENSOR_LEFT_HAND] = left_hand;
      diag->proc_ft[SENSOR_RIGHT_ARM] = right_arm;
      diag->proc_ft[SENSOR_RIGHT_HAND] = right_hand;
      diag->ft_saturated = ft_sat;
    }

    if ((bilateral_active_debug_counter++ % 100) == 0)
    {
      std::cout << "[runBilateralActiveMode] damping_scale=" << damping_scale
                << " dt=" << dt
                << " fault=" << bilateral_active_fault_counter
                << std::endl;
    }

    output.valid = true;
  }

} // namespace upperlimb_robot
