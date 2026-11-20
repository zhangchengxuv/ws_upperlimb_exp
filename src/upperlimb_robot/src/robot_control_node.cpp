// Copyright 2024 Zhang Chenxu
// Licensed under the Apache License, Version 2.0

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/msg/experiment_config.hpp"
#include "upperlimb_robot/msg/logger_command.hpp"
#include "upperlimb_robot/msg/robot_state.hpp"
#include "upperlimb_robot/msg/system_state.hpp"

#include "upperlimb_robot/async_csv_logger.hpp"
#include "upperlimb_robot/control_algorithms.hpp"
#include "upperlimb_robot/gxn.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

// ===================== GTN 底层库调用串行化 =====================
//
// 所有 GTN_* 底层调用统一经由 RobotControlNode::callGtn() 加锁串行访问，
// 防止状态采集线程与控制线程并发调用控制卡库引发模式/同步异常。
//
#define GTN_Open(...) callGtn([&]() { return ::GTN_Open(__VA_ARGS__); })
#define GTN_Close(...) callGtn([&]() { return ::GTN_Close(__VA_ARGS__); })
#define GTN_LoadConfig(...) callGtn([&]() { return ::GTN_LoadConfig(__VA_ARGS__); })
#define GTN_ClrSts(...) callGtn([&]() { return ::GTN_ClrSts(__VA_ARGS__); })
#define GTN_InitEcatComm(...) callGtn([&]() { return ::GTN_InitEcatComm(__VA_ARGS__); })
#define GTN_IsEcatReady(...) callGtn([&]() { return ::GTN_IsEcatReady(__VA_ARGS__); })
#define GTN_StartEcatComm(...) callGtn([&]() { return ::GTN_StartEcatComm(__VA_ARGS__); })
#define GTN_TerminateEcatComm(...) callGtn([&]() { return ::GTN_TerminateEcatComm(__VA_ARGS__); })
#define GTN_AxisOn(...) callGtn([&]() { return ::GTN_AxisOn(__VA_ARGS__); })
#define GTN_AxisOff(...) callGtn([&]() { return ::GTN_AxisOff(__VA_ARGS__); })
#define GTN_GetSts(...) callGtn([&]() { return ::GTN_GetSts(__VA_ARGS__); })
#define GTN_GetEcatEncPos(...) callGtn([&]() { return ::GTN_GetEcatEncPos(__VA_ARGS__); })
#define GTN_SetPrfPos(...) callGtn([&]() { return ::GTN_SetPrfPos(__VA_ARGS__); })
#define GTN_SetEncPos(...) callGtn([&]() { return ::GTN_SetEncPos(__VA_ARGS__); })
#define GTN_SynchAxisPos(...) callGtn([&]() { return ::GTN_SynchAxisPos(__VA_ARGS__); })
#define GTN_GetEcatRawData(...) callGtn([&]() { return ::GTN_GetEcatRawData(__VA_ARGS__); })
#define GTN_GetEcatAxisAtlTorque(...) callGtn([&]() { return ::GTN_GetEcatAxisAtlTorque(__VA_ARGS__); })
#define GTN_GetEncVel(...) callGtn([&]() { return ::GTN_GetEncVel(__VA_ARGS__); })
#define GTN_Stop(...) callGtn([&]() { return ::GTN_Stop(__VA_ARGS__); })
#define GTN_SetEcatAxisPT(...) callGtn([&]() { return ::GTN_SetEcatAxisPT(__VA_ARGS__); })
#define GTN_SetEcatAxisMode(...) callGtn([&]() { return ::GTN_SetEcatAxisMode(__VA_ARGS__); })
#define GTN_GetEcatAxisMode(...) callGtn([&]() { return ::GTN_GetEcatAxisMode(__VA_ARGS__); })
#define GTN_PrfTrap(...) callGtn([&]() { return ::GTN_PrfTrap(__VA_ARGS__); })
#define GTN_SetTrapPrm(...) callGtn([&]() { return ::GTN_SetTrapPrm(__VA_ARGS__); })
#define GTN_SetVel(...) callGtn([&]() { return ::GTN_SetVel(__VA_ARGS__); })
#define GTN_SetPos(...) callGtn([&]() { return ::GTN_SetPos(__VA_ARGS__); })
#define GTN_Update(...) callGtn([&]() { return ::GTN_Update(__VA_ARGS__); })

class RobotControlNode : public rclcpp::Node
{
public:
  RobotControlNode() : Node("robot_control_node")
  {
    RCLCPP_INFO(this->get_logger(), "机器人硬件控制节点启动（中期基础实验版）");

    // ===================== 启动参数（默认 100 Hz，可一键切回 50 Hz） =====================
    const int param_control_hz = this->declare_parameter<int>("control_hz", 100);
    const int param_state_hz = this->declare_parameter<int>("state_hz", param_control_hz);
    publish_divider_ = std::max(1, static_cast<int>(this->declare_parameter<int>("publish_divider", 5)));
    status_read_divider_ = std::max(1, static_cast<int>(this->declare_parameter<int>("status_read_divider", 20)));
    ecat_config_path_param_ = this->declare_parameter<std::string>("ecat_config_path", "");
    log_output_dir_ = this->declare_parameter<std::string>("log_output_dir", "data");

    state_hz_.store(std::clamp(param_state_hz, 10, 500));
    control_hz_.store(std::clamp(param_control_hz, 10, 500));

    RCLCPP_INFO(
        this->get_logger(),
        "运行频率配置: state_hz=%d Hz, control_hz=%d Hz, publish_divider=%d, log_output_dir=%s",
        state_hz_.load(),
        control_hz_.load(),
        publish_divider_,
        log_output_dir_.c_str());

    // ===================== ROS2 通信接口 =====================
    control_command_sub_ =
        this->create_subscription<upperlimb_robot::msg::ControlCommand>(
            "control_command",
            10,
            std::bind(&RobotControlNode::controlCommandCallback, this, std::placeholders::_1));

    experiment_config_sub_ =
        this->create_subscription<upperlimb_robot::msg::ExperimentConfig>(
            "experiment_config",
            10,
            std::bind(&RobotControlNode::experimentConfigCallback, this, std::placeholders::_1));

    logger_command_sub_ =
        this->create_subscription<upperlimb_robot::msg::LoggerCommand>(
            "logger_command",
            10,
            std::bind(&RobotControlNode::loggerCommandCallback, this, std::placeholders::_1));

    system_pub_ =
        this->create_publisher<upperlimb_robot::msg::SystemState>("system_state", 10);

    robot_state_pub_ =
        this->create_publisher<upperlimb_robot::msg::RobotState>("robot_state", 10);

    joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    state_timer_ = this->create_wall_timer(
        500ms, std::bind(&RobotControlNode::publishSystemState, this));

    // ===================== 硬件初始化 =====================
    if (!initController())
    {
      RCLCPP_ERROR(this->get_logger(), "控制器初始化失败");
      publishSystemState();
      return;
    }
    controller_init_ = true;
    publishSystemState();

    if (!initMotors())
    {
      RCLCPP_ERROR(this->get_logger(), "电机初始化失败");
      publishSystemState();
      return;
    }
    joint_init_ = true;

    system_ready_ = controller_init_ && joint_init_;
    publishSystemState();

    RCLCPP_INFO(this->get_logger(), "硬件初始化完成，系统准备就绪");

    running_.store(true);

    state_read_thread_ = std::thread(&RobotControlNode::stateReadLoop, this);
    RCLCPP_INFO(this->get_logger(), "状态采集线程已启动");

    control_thread_ = std::thread(&RobotControlNode::controlLoop, this);
    RCLCPP_INFO(this->get_logger(), "控制线程已启动");
  }

  ~RobotControlNode() override
  {
    shutdownHardware();
  }

  void shutdownHardware()
  {
    if (shutdown_done_)
    {
      return;
    }
    shutdown_done_ = true;

    running_.store(false);

    if (state_read_thread_.joinable())
    {
      state_read_thread_.join();
    }

    if (control_thread_.joinable())
    {
      control_thread_.join();
    }

    // 优先刷盘并关闭异步 CSV 日志器，避免数据丢失
    csv_logger_.stopSession();

    fprintf(stderr, "正在安全关闭机器人系统...\n");
    fflush(stderr);

    clearAllTorqueCommands();
    restoreAllAxesToPositionMode();
    const bool axes_ok = stopAllAxesRaw();

    if (axes_ok)
    {
      fprintf(stderr, "所有电机已安全关闭。\n");
    }
    else
    {
      fprintf(stderr, "部分电机关闭失败，请检查上面的错误信息。\n");
    }
    fflush(stderr);

    if (ecat_started_)
    {
      const short rtn = GTN_TerminateEcatComm(core_id_);
      if (rtn == 0)
      {
        fprintf(stderr, "EtherCAT通信已关闭。\n");
      }
      else
      {
        fprintf(stderr, "EtherCAT通信关闭失败，错误码=%d\n", rtn);
      }
      fflush(stderr);
      ecat_started_ = false;
    }

    if (controller_opened_)
    {
      const short rtn = GTN_Close();
      if (rtn == 0)
      {
        fprintf(stderr, "控制器已关闭。\n");
      }
      else
      {
        fprintf(stderr, "控制器关闭失败，错误码=%d\n", rtn);
      }
      fflush(stderr);
      controller_opened_ = false;
    }
  }

private:
  // ===================== GTN 调用互斥封装 =====================
  template <typename Fn>
  auto callGtn(Fn &&fn) -> decltype(fn())
  {
    std::lock_guard<std::mutex> lock(gtn_mutex_);
    return fn();
  }

  void requestModeEnterFallback(int fallback_mode)
  {
    mode_enter_failed_ = true;
    mode_enter_fallback_mode_ = fallback_mode;
  }

  // ===================== 主动控制轴过滤 =====================
  // 仅控制 14 个主动关节：右臂 0~6（joint11~17）、左臂 10~16（joint1~7）
  bool isControlledAxisIndex(std::size_t i) const
  {
    return (i <= 6) || (i >= 10 && i <= 16);
  }

  // ===================== 控制命令回调 =====================
  void controlCommandCallback(const upperlimb_robot::msg::ControlCommand::SharedPtr msg)
  {
    current_mode_.store(msg->command);
    RCLCPP_INFO(this->get_logger(), "收到控制命令=%d", msg->command);
  }

  // ===================== 实验参数回调（含运行中安全互锁） =====================
  void experimentConfigCallback(const upperlimb_robot::msg::ExperimentConfig::SharedPtr msg)
  {
    const int active_mode = applied_mode_atomic_.load();
    if (active_mode != upperlimb_robot::MODE_IDLE)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "安全互锁：当前处于运行模式(%d)，禁止在运行中修改实验参数！请先切回 MODE_IDLE(0)。",
          active_mode);
      return;
    }

    upperlimb_robot::ExperimentRuntimeParams params{};
    params.passive_joint_profile =
        std::clamp(static_cast<int>(msg->passive_joint_profile),
                   static_cast<int>(upperlimb_robot::PASSIVE_JOINTS_COMBINED),
                   static_cast<int>(upperlimb_robot::PASSIVE_JOINT_J7_ONLY));
    params.passive_speed_level =
        std::clamp(static_cast<int>(msg->passive_speed_level),
                   static_cast<int>(upperlimb_robot::PASSIVE_SPEED_SLOW),
                   static_cast<int>(upperlimb_robot::PASSIVE_SPEED_STANDARD));
    params.admittance_damping_level =
        std::clamp(static_cast<int>(msg->admittance_damping_level),
                   static_cast<int>(upperlimb_robot::ADMITTANCE_DAMPING_LOW),
                   static_cast<int>(upperlimb_robot::ADMITTANCE_DAMPING_HIGH));

    {
      std::lock_guard<std::mutex> lock(exp_param_mutex_);
      pending_exp_params_ = params;
      exp_params_dirty_ = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "更新实验参数: passive_joint_profile=%d, passive_speed_level=%d, admittance_damping_level=%d",
        params.passive_joint_profile,
        params.passive_speed_level,
        params.admittance_damping_level);
  }

  // ===================== CSV 日志控制与事件打标回调 =====================
  void loggerCommandCallback(const upperlimb_robot::msg::LoggerCommand::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(logger_meta_mutex_);
      if (!msg->subject_id.empty())
      {
        log_subject_id_ = msg->subject_id;
      }
      if (!msg->condition.empty())
      {
        log_condition_ = msg->condition;
      }
      if (!msg->trial_id.empty())
      {
        log_trial_id_ = msg->trial_id;
      }
    }

    if (msg->action == 1)
    {
      std::string subj, cond, trial;
      {
        std::lock_guard<std::mutex> lock(logger_meta_mutex_);
        subj = log_subject_id_;
        cond = log_condition_;
        trial = log_trial_id_;
        pending_event_tag_ = "REC_START";
      }
      log_tick_.store(0);
      log_start_time_ = std::chrono::steady_clock::now();
      const bool ok = csv_logger_.startSession(log_output_dir_, subj, cond, trial);
      if (ok)
      {
        RCLCPP_INFO(this->get_logger(), "已开始 CSV 记录: %s",
                    csv_logger_.currentFilePath().c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "启动 CSV 记录失败");
      }
      publishSystemState();
    }
    else if (msg->action == 2)
    {
      csv_logger_.stopSession();
      RCLCPP_INFO(this->get_logger(), "已停止 CSV 记录");
      publishSystemState();
    }
    else if (msg->action == 3)
    {
      std::lock_guard<std::mutex> lock(logger_meta_mutex_);
      pending_event_tag_ = msg->event_tag.empty() ? "MARK" : msg->event_tag;
      RCLCPP_INFO(this->get_logger(), "收到事件打标: %s", pending_event_tag_.c_str());
    }
  }

  // ===================== 模式状态机更新 =====================
  void updateModeStateMachine()
  {
    // 仅在空闲状态下把新实验参数同步到算法层（双重互锁）
    if (applied_mode_ == upperlimb_robot::MODE_IDLE)
    {
      std::lock_guard<std::mutex> lock(exp_param_mutex_);
      if (exp_params_dirty_)
      {
        upperlimb_robot::setExperimentRuntimeParams(pending_exp_params_);
        active_exp_params_ = pending_exp_params_;
        exp_params_dirty_ = false;
      }
    }

    const int requested_mode = current_mode_.load();

    if (requested_mode == applied_mode_)
    {
      return;
    }

    const int old_mode = applied_mode_;

    onModeExit(old_mode, requested_mode);

    mode_enter_failed_ = false;
    mode_enter_fallback_mode_ = requested_mode;

    onModeEnter(old_mode, requested_mode);

    if (mode_enter_failed_)
    {
      applied_mode_ = mode_enter_fallback_mode_;
      applied_mode_atomic_.store(applied_mode_);
      current_mode_.store(mode_enter_fallback_mode_);

      RCLCPP_WARN(
          this->get_logger(),
          "模式进入失败: old=%d requested=%d fallback=%d",
          old_mode,
          requested_mode,
          mode_enter_fallback_mode_);
      return;
    }

    applied_mode_ = requested_mode;
    applied_mode_atomic_.store(applied_mode_);

    RCLCPP_INFO(
        this->get_logger(),
        "模式切换完成: old=%d -> new=%d",
        old_mode,
        requested_mode);
  }

  // ===================== 模式退出动作 =====================
  void onModeExit(int old_mode, int new_mode)
  {
    (void)new_mode;

    switch (old_mode)
    {
    // 退出扭矩相关模式：先清零 PT，停规划，再恢复位置模式
    case upperlimb_robot::MODE_ZERO_FORCE:
    case upperlimb_robot::MODE_RIGHT_ZERO_FORCE:
    case upperlimb_robot::MODE_BILATERAL_ZERO_FORCE:
    case upperlimb_robot::MODE_ACTIVE:
    case upperlimb_robot::MODE_RIGHT_ACTIVE:
    case upperlimb_robot::MODE_BILATERAL_ACTIVE:
      clearAllTorqueCommands();
      stopAllAxesMotionOnly();
      restoreAllAxesToPositionMode();
      invalidateAllPositionCommandCache();
      break;

    // 退出位置相关模式：停止当前残余位置运动，并清除位置命令缓存
    case upperlimb_robot::MODE_PASSIVE:
    case upperlimb_robot::MODE_PASSIVE_SINE_PD_FF:
    case upperlimb_robot::MODE_LEFT_PRESET_POSITION:
    case upperlimb_robot::MODE_RIGHT_PRESET_POSITION:
      stopAllAxesMotionOnly();
      invalidateAllPositionCommandCache();
      break;

    case upperlimb_robot::MODE_IDLE:
    default:
      break;
    }
  }

  // ===================== 模式进入动作 =====================
  void onModeEnter(int old_mode, int new_mode)
  {
    // 从 Idle 进入其他模式时，先确保 14 个主动轴全部使能
    if (old_mode == upperlimb_robot::MODE_IDLE &&
        new_mode != upperlimb_robot::MODE_IDLE)
    {
      for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
      {
        if (!isControlledAxisIndex(i))
        {
          axis_enabled_[i] = false;
          continue;
        }

        const short axis = axis_ids_[i];
        const short rtn = GTN_AxisOn(core_id_, axis);

        if (rtn != 0)
        {
          RCLCPP_ERROR(
              this->get_logger(),
              "GTN_AxisOn 失败，axis=%d，错误码=%d",
              axis,
              rtn);
        }
        else
        {
          axis_enabled_[i] = true;
        }
      }

      RCLCPP_INFO(this->get_logger(), "退出空闲模式：已重新使能 14 个主动轴，末端 6 个关节不参与控制");
    }

    switch (new_mode)
    {
    case upperlimb_robot::MODE_ZERO_FORCE:
    {
      const bool ok = captureForceSensorBias();
      if (ok)
      {
        RCLCPP_INFO(this->get_logger(), "进入左臂零力模式，已记录六维力零偏");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "进入左臂零力模式失败：当前传感器数据无效，未记录零偏");
      }
      break;
    }

    case upperlimb_robot::MODE_RIGHT_ZERO_FORCE:
    {
      const bool ok = captureForceSensorBias();
      if (ok)
      {
        RCLCPP_INFO(this->get_logger(), "进入右臂零力模式，已记录六维力零偏");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "进入右臂零力模式失败：当前传感器数据无效，未记录零偏");
      }
      break;
    }

    case upperlimb_robot::MODE_BILATERAL_ZERO_FORCE:
    {
      // 完整保留原实机验证通过的“预计算 -> 逐轴切 CST 并立即下发首条重力补偿 PT”交接机制
      upperlimb_robot::ControlOutputCache initial_torque_output{};
      const bool bias_ok = captureForceSensorBias();
      const bool precompute_ok =
          bias_ok && precomputeBilateralZeroForceBeforeSwitch(initial_torque_output);
      const bool torque_ok =
          precompute_ok && enterDualArmTorqueModeOnce(&initial_torque_output);

      if (torque_ok)
      {
        RCLCPP_INFO(this->get_logger(),
                    "进入双臂零力模式：预计算有效，14轴已逐轴切换并分别下发首条补偿PT");
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "进入双臂零力模式失败：bias_ok=%d, precompute_ok=%d, torque_ok=%d；退回 Idle",
            static_cast<int>(bias_ok),
            static_cast<int>(precompute_ok),
            static_cast<int>(torque_ok));

        if (precompute_ok)
        {
          clearDualArmTorqueCommandsNoModeSwitch();
          stopAllAxesMotionOnly();
          restoreAllAxesToPositionMode();
        }
        requestModeEnterFallback(upperlimb_robot::MODE_IDLE);
      }
      break;
    }

    case upperlimb_robot::MODE_ACTIVE:
    {
      const bool ok = captureForceSensorBias();
      upperlimb_robot::resetActiveMode();
      if (ok)
      {
        RCLCPP_INFO(this->get_logger(), "进入左臂主动导纳模式，已记录六维力零偏并重置导纳状态");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "进入左臂主动导纳模式失败：当前传感器数据无效，未记录零偏");
      }
      break;
    }

    case upperlimb_robot::MODE_RIGHT_ACTIVE:
    {
      const bool ok = captureForceSensorBias();
      upperlimb_robot::resetRightActiveMode();
      if (ok)
      {
        RCLCPP_INFO(this->get_logger(), "进入右臂主动导纳模式，已记录六维力零偏并重置右臂导纳状态");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "进入右臂主动导纳模式失败：当前传感器数据无效，未记录零偏");
      }
      break;
    }

    case upperlimb_robot::MODE_BILATERAL_ACTIVE:
    {
      const bool bias_ok = captureForceSensorBias();
      upperlimb_robot::resetBilateralActiveMode();

      const bool torque_ok = enterDualArmTorqueModeOnce();

      if (bias_ok && torque_ok)
      {
        RCLCPP_INFO(this->get_logger(), "进入双侧主动导纳模式：已记录六维力零偏，14 个主动轴已切到扭矩模式");
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "进入双侧主动导纳模式：bias_ok=%d, torque_ok=%d，请检查传感器有效性和轴模式切换",
            static_cast<int>(bias_ok),
            static_cast<int>(torque_ok));
      }
      break;
    }

    case upperlimb_robot::MODE_LEFT_PRESET_POSITION:
      RCLCPP_INFO(this->get_logger(), "进入左臂预设位置模式");
      break;

    case upperlimb_robot::MODE_RIGHT_PRESET_POSITION:
      RCLCPP_INFO(this->get_logger(), "进入右臂预设位置模式");
      break;

    case upperlimb_robot::MODE_PASSIVE:
      upperlimb_robot::resetPassiveMode();
      RCLCPP_INFO(this->get_logger(), "进入左臂正弦被动模式（PID跟踪），已重置起始角与平滑起步状态");
      break;

    case upperlimb_robot::MODE_PASSIVE_SINE_PD_FF:
      upperlimb_robot::resetPassivePDFFMode();
      RCLCPP_INFO(this->get_logger(), "进入左臂正弦被动模式（PD+前馈跟踪），已重置起始角与平滑起步状态");
      break;

    case upperlimb_robot::MODE_IDLE:
      RCLCPP_INFO(this->get_logger(), "进入安全停止 / 空闲模式（保持位置模式伺服使能）");
      break;

    default:
      break;
    }
  }

  // ===================== 双臂扭矩模式辅助函数 =====================
  bool isDualArmTorqueMode(int mode) const
  {
    return mode == upperlimb_robot::MODE_BILATERAL_ZERO_FORCE ||
           mode == upperlimb_robot::MODE_BILATERAL_ACTIVE;
  }

  std::array<std::size_t, 14> dualArmActiveTorqueIndices() const
  {
    return {
        0, 1, 2, 3, 4, 5, 6,       // 右臂 joint11~joint17 -> 0~6
        10, 11, 12, 13, 14, 15, 16 // 左臂 joint1~joint7   -> 10~16
    };
  }

  // ===================== 双臂零力：切换扭矩模式前预计算（原样保留） =====================
  bool precomputeBilateralZeroForceBeforeSwitch(
      upperlimb_robot::ControlOutputCache &precomputed)
  {
    precomputed = upperlimb_robot::ControlOutputCache{};
    if (!state_ready_.load())
    {
      RCLCPP_ERROR(this->get_logger(), "双臂零力预计算失败：关节状态尚未准备完成");
      return false;
    }

    upperlimb_robot::MotorStateCache state;
    upperlimb_robot::SensorStateCache sensor_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = latest_motor_state_;
    }
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      sensor_state = latest_sensor_state_;
    }

    if (!sensor_state.valid)
    {
      RCLCPP_ERROR(this->get_logger(), "双臂零力预计算失败：六维力传感器数据无效");
      return false;
    }

    for (const std::size_t i : dualArmActiveTorqueIndices())
    {
      if (!std::isfinite(state.position[i]) ||
          !std::isfinite(state.velocity[i]))
      {
        RCLCPP_ERROR(this->get_logger(),
                     "双臂零力预计算失败：控制索引%zu关节位置或速度非有限值", i);
        return false;
      }
    }

    const auto t_begin = std::chrono::steady_clock::now();
    try
    {
      upperlimb_robot::runBilateralZeroForceMode(
          state, sensor_state, precomputed);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "双臂零力预计算异常：%s", e.what());
      return false;
    }

    if (!precomputed.valid)
    {
      RCLCPP_ERROR(this->get_logger(), "双臂零力预计算失败：Pinocchio初始化或控制输出无效");
      return false;
    }

    for (const std::size_t i : dualArmActiveTorqueIndices())
    {
      if (!precomputed.use_torque[i] ||
          !std::isfinite(precomputed.target_torque[i]))
      {
        RCLCPP_ERROR(this->get_logger(),
                     "双臂零力预计算失败：控制索引%zu输出力矩无效", i);
        return false;
      }
    }

    const auto precompute_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now() - t_begin)
                                   .count();
    RCLCPP_INFO(this->get_logger(),
                "双臂零力预计算通过：用时=%lld us，右1轴=%.3f Nm，左11轴=%.3f Nm；尚未切换模式或下发PT",
                static_cast<long long>(precompute_us),
                precomputed.target_torque[0],
                precomputed.target_torque[10]);

    return true;
  }

  // ===================== 14轴扭矩模式切换和首次PT力矩交接（原样保留） =====================
  bool enterDualArmTorqueModeOnce(
      const upperlimb_robot::ControlOutputCache *initial_output = nullptr)
  {
    const auto indices = dualArmActiveTorqueIndices();
    std::array<short, 14> initial_pt_raw{};

    if (initial_output != nullptr)
    {
      for (std::size_t k = 0; k < indices.size(); ++k)
      {
        const std::size_t i = indices[k];
        if (i >= upperlimb_robot::AXIS_NUM ||
            !isControlledAxisIndex(i) ||
            !initial_output->valid ||
            !initial_output->use_torque[i] ||
            !std::isfinite(initial_output->target_torque[i]))
        {
          RCLCPP_ERROR(this->get_logger(),
                       "双臂零力PT预检查失败：索引%zu的初始力矩无效", i);
          return false;
        }

        const int axis = axis_ids_[i];
        const double raw = upperlimb_robot::torqueNmToRaw(
            axis, initial_output->target_torque[i]);
        const double raw_per_nm = upperlimb_robot::torqueNmToRaw(axis, 1.0);
        if (!std::isfinite(raw) || !std::isfinite(raw_per_nm) ||
            raw_per_nm == 0.0 ||
            raw < static_cast<double>(std::numeric_limits<short>::min()) ||
            raw > static_cast<double>(std::numeric_limits<short>::max()))
        {
          RCLCPP_ERROR(this->get_logger(),
                       "双臂零力PT预检查失败：物理轴%d Nm/raw换算错误或超short范围", axis);
          return false;
        }
        initial_pt_raw[k] = static_cast<short>(std::lround(raw));
      }
    }

    using Clock = std::chrono::steady_clock;
    struct FirstAxisTrace
    {
      int axis{0};
      double target_nm{0.0};
      short raw{0};
      long long mode_us{0};
      long long pt_us{0};
      long long total_us{0};
      bool recorded{false};
    };
    std::array<FirstAxisTrace, 2> traces{};
    const auto start_time = Clock::now();
    bool all_ok = true;

    for (std::size_t k = 0; k < indices.size(); ++k)
    {
      const std::size_t i = indices[k];
      if (i >= upperlimb_robot::AXIS_NUM)
      {
        all_ok = false;
        if (initial_output != nullptr)
          return false;
        continue;
      }

      const auto before_mode = Clock::now();
      if (!ensureAxisInTorqueMode(i))
      {
        all_ok = false;
        if (initial_output != nullptr)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "双臂零力逐轴交接中止：物理轴%d切入扭矩模式失败", axis_ids_[i]);
          return false;
        }
        continue;
      }
      const auto mode_ready = Clock::now();

      if (initial_output != nullptr)
      {
        const int axis = axis_ids_[i];
        const short rtn_pt = GTN_SetEcatAxisPT(core_id_, axis, initial_pt_raw[k]);
        const auto pt_done = Clock::now();
        if (rtn_pt != 0)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "双臂零力逐轴交接中止：物理轴%d首条PT发送失败，错误码=%d，目标=%.3f Nm，raw=%d",
                       axis, rtn_pt, initial_output->target_torque[i],
                       static_cast<int>(initial_pt_raw[k]));
          return false;
        }

        last_target_torque_[i] = initial_output->target_torque[i];

        if (axis == 1 || axis == 11)
        {
          FirstAxisTrace &trace = traces[axis == 1 ? 0 : 1];
          trace.axis = axis;
          trace.target_nm = initial_output->target_torque[i];
          trace.raw = initial_pt_raw[k];
          trace.mode_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              mode_ready - before_mode)
                              .count();
          trace.pt_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            pt_done - mode_ready)
                            .count();
          trace.total_us = std::chrono::duration_cast<std::chrono::microseconds>(
                               pt_done - start_time)
                               .count();
          trace.recorded = true;
        }
      }
      else
      {
        last_target_torque_[i] = 0.0;
      }

      command_sent_[i] = false;
      position_profile_configured_[i] = false;
    }

    if (initial_output != nullptr && all_ok)
    {
      for (const FirstAxisTrace &trace : traces)
      {
        if (!trace.recorded)
          continue;
        RCLCPP_INFO(this->get_logger(),
                    "双臂零力首条PT交接：物理轴%d，目标=%.3f Nm，raw=%d，单轴切模式=%lld us，模式确认至PT返回=%lld us，从开始切轴至PT返回=%lld us",
                    trace.axis, trace.target_nm, static_cast<int>(trace.raw),
                    trace.mode_us, trace.pt_us, trace.total_us);
      }
      const auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                Clock::now() - start_time)
                                .count();
      RCLCPP_INFO(this->get_logger(),
                  "双臂零力14轴逐轴PT交接完成：合计=%lld us",
                  static_cast<long long>(total_us));
    }

    return all_ok;
  }

  void clearDualArmTorqueCommandsNoModeSwitch()
  {
    for (const std::size_t i : dualArmActiveTorqueIndices())
    {
      if (i >= upperlimb_robot::AXIS_NUM)
      {
        continue;
      }

      const int axis = axis_ids_[i];
      const short rtn = GTN_SetEcatAxisPT(core_id_, axis, 0);
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "双臂扭矩模式清零 PT 失败，轴%d，错误码=%d", axis, rtn);
      }

      last_target_torque_[i] = 0.0;
    }
  }

  // ===================== 控制器初始化 =====================
  bool initController()
  {
    short rtn = GTN_Open();
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "GTN_Open 打开控制器失败，错误码=%d", rtn);
      return false;
    }
    controller_opened_ = true;
    RCLCPP_INFO(this->get_logger(), "GTN_Open 成功");

    std::string config_path = ecat_config_path_param_;
    if (config_path.empty())
    {
      try
      {
        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("upperlimb_robot");
        config_path = pkg_share + "/config/Gecat.eni";
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "获取包配置目录失败: %s", e.what());
        return false;
      }
    }

    rtn = GTN_LoadConfig(core_id_, const_cast<char *>(config_path.c_str()));
    if (rtn != 0)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "GTN_LoadConfig 加载配置文件失败，错误码=%d, 路径=%s",
          rtn, config_path.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "GTN_LoadConfig 成功 (%s)", config_path.c_str());

    rtn = GTN_ClrSts(core_id_, 1, static_cast<short>(upperlimb_robot::AXIS_NUM));
    if (rtn != 0)
    {
      RCLCPP_WARN(this->get_logger(), "GTN_ClrSts 更新配置状态失败，错误码=%d", rtn);
    }

    rtn = GTN_InitEcatComm(core_id_);
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "EtherCAT初始化失败，错误码=%d", rtn);
      return false;
    }

    short ready = 0;
    int retry = 0;
    constexpr int max_retry = 100;

    do
    {
      rtn = GTN_IsEcatReady(core_id_, &ready);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "检测EtherCAT状态失败，错误码=%d", rtn);
        return false;
      }

      if (ready == 1)
      {
        break;
      }

      rclcpp::sleep_for(50ms);
      retry++;
    } while (retry < max_retry);

    if (ready != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "EtherCAT启动超时");
      return false;
    }

    rtn = GTN_StartEcatComm(core_id_);
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "启动EtherCAT通信失败，错误码=%d", rtn);
      return false;
    }

    ecat_started_ = true;
    RCLCPP_INFO(this->get_logger(), "EtherCAT通信启动成功");

    return true;
  }

  // ===================== 电机初始化 =====================
  bool initMotors()
  {
    short rtn = 0;

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      if (!isControlledAxisIndex(i))
      {
        enc_pos_[i] = 0;
        axis_enabled_[i] = false;
        RCLCPP_INFO(
            this->get_logger(),
            "轴%d 为末端非控制关节，初始化阶段跳过使能、同步和模式控制",
            axis);
        continue;
      }

      rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "读取轴%d编码器位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_SetPrfPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "设置轴%d规划位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_SetEncPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "设置轴%d编码器位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_SynchAxisPos(core_id_, static_cast<long>(1L << (axis - 1)));
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "同步轴%d位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_AxisOn(core_id_, axis);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "轴%d使能失败，错误码=%d", axis, rtn);
        return false;
      }

      axis_enabled_[i] = true;

      RCLCPP_INFO(this->get_logger(), "轴%d初始化完成，编码器位置=%d", axis, enc_pos_[i]);
    }

    return true;
  }

  // ===================== 系统状态发布 =====================
  void publishSystemState()
  {
    upperlimb_robot::msg::SystemState msg;
    msg.joint_init = joint_init_;
    msg.controller_init = controller_init_;
    msg.system_ready = system_ready_;
    msg.applied_mode = applied_mode_atomic_.load();
    msg.logging_active = csv_logger_.isActive();
    msg.current_log_file = csv_logger_.currentFilePath();
    msg.dropped_frames = csv_logger_.droppedFrames();
    system_pub_->publish(msg);
  }

  // ===================== 读取 EtherCAT 原始 float 数据 =====================
  bool readFloatRawData(int offset, double &value)
  {
    int raw = 0;
    const short rtn =
        GTN_GetEcatRawData(core_id_, offset, 4, reinterpret_cast<unsigned char *>(&raw));

    if (rtn != 0)
    {
      value = 0.0;
      return false;
    }

    float temp = 0.0f;
    std::memcpy(&temp, &raw, sizeof(float));
    value = static_cast<double>(temp);
    return true;
  }

  bool readSingleForceSensor(int base_offset, upperlimb_robot::ForceSensorData &data)
  {
    bool ok = true;
    ok &= readFloatRawData(base_offset + 0, data.fx);
    ok &= readFloatRawData(base_offset + 4, data.fy);
    ok &= readFloatRawData(base_offset + 8, data.fz);
    ok &= readFloatRawData(base_offset + 12, data.mx);
    ok &= readFloatRawData(base_offset + 16, data.my);
    ok &= readFloatRawData(base_offset + 20, data.mz);
    return ok;
  }

  bool readAllForceSensors(upperlimb_robot::SensorStateCache &sensor_state)
  {
    bool ok = true;
    ok &= readSingleForceSensor(8, sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_HAND]);
    ok &= readSingleForceSensor(40, sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_ARM]);
    ok &= readSingleForceSensor(72, sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_HAND]);
    ok &= readSingleForceSensor(104, sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_ARM]);
    sensor_state.valid = ok;
    return ok;
  }

  bool captureForceSensorBias()
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!latest_sensor_state_.valid)
    {
      return false;
    }
    latest_sensor_state_.bias = latest_sensor_state_.sensor;
    return true;
  }

  upperlimb_robot::ForceSensorData subtractBias(
      const upperlimb_robot::ForceSensorData &raw,
      const upperlimb_robot::ForceSensorData &bias) const
  {
    upperlimb_robot::ForceSensorData out;
    out.fx = raw.fx - bias.fx;
    out.fy = raw.fy - bias.fy;
    out.fz = raw.fz - bias.fz;
    out.mx = raw.mx - bias.mx;
    out.my = raw.my - bias.my;
    out.mz = raw.mz - bias.mz;
    return out;
  }

  // ===================== 传感器状态转成 24 维数组 =====================
  std::vector<double> buildForceSensorArray(
      const upperlimb_robot::SensorStateCache &sensor_state) const
  {
    std::vector<double> out;
    out.reserve(24);

    const auto right_hand = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_HAND],
        sensor_state.bias[upperlimb_robot::SENSOR_RIGHT_HAND]);
    const auto right_arm = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_ARM],
        sensor_state.bias[upperlimb_robot::SENSOR_RIGHT_ARM]);
    const auto left_hand = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_HAND],
        sensor_state.bias[upperlimb_robot::SENSOR_LEFT_HAND]);
    const auto left_arm = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_ARM],
        sensor_state.bias[upperlimb_robot::SENSOR_LEFT_ARM]);

    auto fill_one = [&](const upperlimb_robot::ForceSensorData &s)
    {
      out.push_back(s.fx);
      out.push_back(s.fy);
      out.push_back(s.fz);
      out.push_back(s.mx);
      out.push_back(s.my);
      out.push_back(s.mz);
    };

    fill_one(right_hand);
    fill_one(right_arm);
    fill_one(left_hand);
    fill_one(left_arm);

    return out;
  }

  // ===================== 发布 RobotState =====================
  void publishRobotState(
      const upperlimb_robot::MotorStateCache &state,
      const upperlimb_robot::SensorStateCache &sensor_state)
  {
    upperlimb_robot::msg::RobotState msg;

    msg.position.assign(state.position.begin(), state.position.end());
    msg.velocity.assign(state.velocity.begin(), state.velocity.end());
    msg.torque.assign(state.torque.begin(), state.torque.end());
    msg.enabled.assign(state.enable.begin(), state.enable.end());

    const auto force_array = buildForceSensorArray(sensor_state);
    msg.force_sensor.assign(force_array.begin(), force_array.end());

    msg.actual_dt_ms = last_actual_dt_ms_.load();
    msg.state_elapsed_ms = last_state_elapsed_ms_.load();
    msg.control_elapsed_ms = last_control_elapsed_ms_.load();
    msg.applied_mode = applied_mode_atomic_.load();

    robot_state_pub_->publish(msg);
    joint_state_pub_->publish(convertToJointStateMsg(state));
  }

  // ===================== 状态采集线程 =====================
  void stateReadLoop()
  {
    std::uint64_t timeout_count = 0;

    while (running_.load())
    {
      const int state_hz = state_hz_.load();
      if (state_hz <= 0)
      {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const auto period = std::chrono::microseconds(1000000 / state_hz);
      const auto cycle_start = std::chrono::steady_clock::now();

      upperlimb_robot::MotorStateCache local_state;
      upperlimb_robot::SensorStateCache local_sensor_state;

      const bool do_read_status =
          (++status_read_divider_count_ >= status_read_divider_);
      if (do_read_status)
      {
        status_read_divider_count_ = 0;
      }

      for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
      {
        const int axis = axis_ids_[i];

        if (!isControlledAxisIndex(i))
        {
          local_state.enable[i] = false;
          local_state.position[i] = 0.0;
          local_state.velocity[i] = 0.0;
          local_state.torque[i] = 0.0;
          continue;
        }

        short rtn = 0;
        int32_t sts = 0;
        int32_t enc_pos = 0;
        double vel = 0.0;
        short torque_raw = 0;

        if (do_read_status)
        {
          rtn = GTN_GetSts(core_id_, axis, &sts, 1);
          local_state.enable[i] = (rtn == 0) ? (sts != 0) : axis_enabled_[i];
        }
        else
        {
          local_state.enable[i] = axis_enabled_[i];
        }

        rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos);
        local_state.position[i] =
            (rtn == 0)
                ? upperlimb_robot::pulseToJointDeg(axis, static_cast<double>(enc_pos))
                : 0.0;

        rtn = GTN_GetEcatAxisAtlTorque(core_id_, axis, &torque_raw);
        local_state.torque[i] =
            (rtn == 0)
                ? upperlimb_robot::torqueRawToNm(axis, static_cast<double>(torque_raw))
                : 0.0;

        rtn = GTN_GetEncVel(core_id_, axis, &vel, 1);
        local_state.velocity[i] =
            (rtn == 0)
                ? upperlimb_robot::pulsePerMsToJointDegPerSec(axis, vel)
                : 0.0;
      }

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        local_sensor_state.bias = latest_sensor_state_.bias;
      }

      readAllForceSensors(local_sensor_state);

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_motor_state_ = local_state;
        state_ready_.store(true);
      }

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_sensor_state_.sensor = local_sensor_state.sensor;
        latest_sensor_state_.bias = local_sensor_state.bias;
        latest_sensor_state_.valid = local_sensor_state.valid;
      }

      const auto work_end = std::chrono::steady_clock::now();
      const double state_ms =
          std::chrono::duration<double, std::milli>(work_end - cycle_start).count();
      last_state_elapsed_ms_.store(state_ms);

      publish_divider_count_++;
      if (publish_divider_count_ >= publish_divider_)
      {
        publish_divider_count_ = 0;
        publishRobotState(local_state, local_sensor_state);
      }

      const auto cycle_end = std::chrono::steady_clock::now();
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);

      if (elapsed < period)
      {
        std::this_thread::sleep_for(period - elapsed);
      }
      else
      {
        timeout_count++;
        if (timeout_count % 100 == 0)
        {
          const double actual_hz = 1000000.0 / static_cast<double>(elapsed.count());
          RCLCPP_WARN(
              this->get_logger(),
              "状态采集循环超时，目标频率=%d Hz，实际频率=%.1f Hz，耗时=%ld us",
              state_hz_.load(),
              actual_hz,
              elapsed.count());
        }
      }
    }
  }

  // ===================== 控制线程 =====================
  void controlLoop()
  {
    std::uint64_t timeout_count = 0;

    while (running_.load())
    {
      const int control_hz = control_hz_.load();
      if (control_hz <= 0)
      {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const auto period = std::chrono::microseconds(1000000 / control_hz);
      const auto cycle_start = std::chrono::steady_clock::now();

      if (!system_ready_ || !state_ready_.load())
      {
        std::this_thread::sleep_for(period);
        continue;
      }

      upperlimb_robot::MotorStateCache local_state;
      upperlimb_robot::SensorStateCache local_sensor_state;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_state = latest_motor_state_;
      }

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        local_sensor_state = latest_sensor_state_;
      }

      const int previous_mode = applied_mode_;
      updateModeStateMachine();

      // 双臂零力首帧读取切换后最新关节状态与新记录零偏
      if (previous_mode != applied_mode_ &&
          applied_mode_ == upperlimb_robot::MODE_BILATERAL_ZERO_FORCE)
      {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          local_state = latest_motor_state_;
        }
        {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          local_sensor_state = latest_sensor_state_;
        }
      }

      const int mode = applied_mode_;
      double actual_dt_s = 1.0 / static_cast<double>(control_hz);
      if (last_control_time_valid_)
      {
        actual_dt_s =
            std::chrono::duration<double>(cycle_start - last_control_time_).count();
      }
      last_control_time_ = cycle_start;
      last_control_time_valid_ = true;
      last_actual_dt_ms_.store(actual_dt_s * 1000.0);

      upperlimb_robot::AlgorithmDiagData diag{};
      upperlimb_robot::ControlOutputCache output =
          computeControlOutput(mode, local_state, local_sensor_state, actual_dt_s, diag);

      if (output.valid)
      {
        applyControlOutput(output);
      }
      else
      {
        if (isDualArmTorqueMode(applied_mode_))
        {
          clearDualArmTorqueCommandsNoModeSwitch();
        }
        else
        {
          clearAllTorqueCommands();
          restoreAllAxesToPositionMode();
        }
      }

      const auto control_done = std::chrono::steady_clock::now();
      const double control_ms =
          std::chrono::duration<double, std::milli>(control_done - cycle_start).count();
      last_control_elapsed_ms_.store(control_ms);

      // ===================== 同频异步 CSV 日志入队（非阻塞） =====================
      if (csv_logger_.isActive())
      {
        upperlimb_robot::LogRecord rec{};
        {
          std::lock_guard<std::mutex> lock(logger_meta_mutex_);
          upperlimb_robot::copySanitizedTag(rec.subject_id, sizeof(rec.subject_id), log_subject_id_);
          upperlimb_robot::copySanitizedTag(rec.condition, sizeof(rec.condition), log_condition_);
          upperlimb_robot::copySanitizedTag(rec.trial_id, sizeof(rec.trial_id), log_trial_id_);
          upperlimb_robot::copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), pending_event_tag_);
          pending_event_tag_.clear();
        }

        rec.mode = mode;
        upperlimb_robot::copySanitizedTag(
            rec.param_preset,
            sizeof(rec.param_preset),
            upperlimb_robot::formatParamPreset(mode, active_exp_params_));
        rec.passive_joint_profile = active_exp_params_.passive_joint_profile;
        rec.passive_speed_level = active_exp_params_.passive_speed_level;
        rec.admittance_damping_level = active_exp_params_.admittance_damping_level;

        rec.tick = log_tick_.fetch_add(1) + 1;
        rec.monotonic_time_s =
            std::chrono::duration<double>(cycle_start - log_start_time_).count();
        rec.actual_dt_ms = actual_dt_s * 1000.0;
        rec.state_elapsed_ms = last_state_elapsed_ms_.load();
        rec.control_elapsed_ms = control_ms;
        rec.dropped_frames = csv_logger_.droppedFrames();

        upperlimb_robot::fillLogRecordFromStates(
            rec, local_state, local_sensor_state, output, diag);

        (void)csv_logger_.push(rec);
      }

      const auto cycle_end = std::chrono::steady_clock::now();
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);

      if (elapsed < period)
      {
        std::this_thread::sleep_for(period - elapsed);
      }
      else
      {
        timeout_count++;
        if (timeout_count % 100 == 0)
        {
          const double actual_hz = 1000000.0 / static_cast<double>(elapsed.count());
          RCLCPP_WARN(
              this->get_logger(),
              "控制循环超时，目标频率=%d Hz，实际频率=%.1f Hz，耗时=%ld us",
              control_hz_.load(),
              actual_hz,
              elapsed.count());
        }
      }
    }
  }

  // ===================== 控制算法总入口 =====================
  upperlimb_robot::ControlOutputCache computeControlOutput(
      int mode,
      const upperlimb_robot::MotorStateCache &state,
      const upperlimb_robot::SensorStateCache &sensor_state,
      double actual_dt_s,
      upperlimb_robot::AlgorithmDiagData &diag)
  {
    upperlimb_robot::ControlOutputCache output{};

    switch (mode)
    {
    case upperlimb_robot::MODE_LEFT_PRESET_POSITION:
      upperlimb_robot::runLeftPresetPosition(state, output);
      for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
      {
        if (output.use_position[i])
        {
          diag.qd_deg[i] = output.target_position[i];
        }
      }
      break;

    case upperlimb_robot::MODE_RIGHT_PRESET_POSITION:
      upperlimb_robot::runRightPresetPosition(state, output);
      for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
      {
        if (output.use_position[i])
        {
          diag.qd_deg[i] = output.target_position[i];
        }
      }
      break;

    case upperlimb_robot::MODE_ZERO_FORCE:
      upperlimb_robot::runZeroForceMode(state, sensor_state, output, &diag);
      break;

    case upperlimb_robot::MODE_RIGHT_ZERO_FORCE:
      upperlimb_robot::runRightZeroForceMode(state, sensor_state, output, &diag);
      break;

    case upperlimb_robot::MODE_BILATERAL_ZERO_FORCE:
      upperlimb_robot::runBilateralZeroForceMode(state, sensor_state, output, &diag);
      break;

    case upperlimb_robot::MODE_ACTIVE:
      upperlimb_robot::runActiveMode(state, sensor_state, actual_dt_s, output, &diag);
      break;

    case upperlimb_robot::MODE_RIGHT_ACTIVE:
      upperlimb_robot::runRightActiveMode(state, sensor_state, actual_dt_s, output, &diag);
      break;

    case upperlimb_robot::MODE_BILATERAL_ACTIVE:
      upperlimb_robot::runBilateralActiveMode(state, sensor_state, actual_dt_s, output, &diag);
      break;

    case upperlimb_robot::MODE_PASSIVE:
      upperlimb_robot::runPassiveMode(state, sensor_state, actual_dt_s, output, &diag);
      break;

    case upperlimb_robot::MODE_PASSIVE_SINE_PD_FF:
      upperlimb_robot::runPassivePDFFMode(state, sensor_state, actual_dt_s, output, &diag);
      break;

    case upperlimb_robot::MODE_IDLE:
    default:
      upperlimb_robot::runIdle(state, output);
      break;
    }

    // 不可控末端关节输出清理（双重保护）
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (isControlledAxisIndex(i))
      {
        continue;
      }

      output.use_position[i] = false;
      output.use_velocity[i] = false;
      output.use_torque[i] = false;
      output.target_position[i] = 0.0;
      output.target_velocity[i] = 0.0;
      output.target_torque[i] = 0.0;
      output.target_acc[i] = 0.0;
      output.target_dec[i] = 0.0;
      output.target_smooth_time[i] = 0;
    }

    return output;
  }

  // ===================== 执行控制输出 =====================
  void applyControlOutput(const upperlimb_robot::ControlOutputCache &output)
  {
    applyPositionOutput(output);
    applyTorqueOutput(output);
  }

  // ===================== 单轴切到位置模式（模式 8） =====================
  bool ensureAxisInPositionMode(std::size_t i)
  {
    if (i >= upperlimb_robot::AXIS_NUM || !isControlledAxisIndex(i))
    {
      return false;
    }

    const int axis = axis_ids_[i];

    const short rtn_stop =
        GTN_Stop(core_id_, static_cast<long>(1L << (axis - 1)), 0);
    if (rtn_stop != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "切回位置模式前停止轴%d失败，错误码=%d", axis, rtn_stop);
      return false;
    }

    const short rtn_zero = GTN_SetEcatAxisPT(core_id_, axis, 0);
    if (rtn_zero != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "切回位置模式前清零 PT 失败，轴%d，错误码=%d", axis, rtn_zero);
      return false;
    }

    const short rtn_mode = GTN_SetEcatAxisMode(core_id_, axis, 8);
    if (rtn_mode != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "GTN_SetEcatAxisMode 设置位置模式失败，轴%d，错误码=%d",
          axis, rtn_mode);
      return false;
    }

    unsigned short actual_mode = static_cast<unsigned short>(-1);
    short rtn_get_mode = 0;
    bool mode_ok = false;

    for (int retry = 0; retry < 8; ++retry)
    {
      rtn_get_mode = GTN_GetEcatAxisMode(core_id_, axis, &actual_mode);
      if (rtn_get_mode == 0 && actual_mode == 8)
      {
        mode_ok = true;
        break;
      }
      std::this_thread::sleep_for(5ms);
    }

    if (!mode_ok)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "轴%d 期望切到位置模式(8)，实际=%d，错误码=%d", axis, actual_mode, rtn_get_mode);
      return false;
    }

    int32_t enc_pos = 0;
    const short rtn_get_enc = GTN_GetEcatEncPos(core_id_, axis, &enc_pos);
    if (rtn_get_enc != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "读取轴%d编码器位置失败，错误码=%d", axis, rtn_get_enc);
      return false;
    }

    const short rtn_set_prf = GTN_SetPrfPos(core_id_, axis, enc_pos);
    if (rtn_set_prf != 0)
    {
      return false;
    }

    const short rtn_set_enc = GTN_SetEncPos(core_id_, axis, enc_pos);
    if (rtn_set_enc != 0)
    {
      return false;
    }

    const short rtn_sync =
        GTN_SynchAxisPos(core_id_, static_cast<long>(1L << (axis - 1)));
    if (rtn_sync != 0)
    {
      return false;
    }

    axis_in_torque_mode_[i] = false;
    last_target_torque_[i] = 0.0;
    command_sent_[i] = false;
    position_profile_configured_[i] = false;
    return true;
  }

  // ===================== 单轴切到扭矩模式（模式 10） =====================
  bool ensureAxisInTorqueMode(std::size_t i)
  {
    if (i >= upperlimb_robot::AXIS_NUM || !isControlledAxisIndex(i))
    {
      return false;
    }

    const int axis = axis_ids_[i];

    const short rtn_stop =
        GTN_Stop(core_id_, static_cast<long>(1L << (axis - 1)), 0);
    if (rtn_stop != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "切到扭矩模式前停止轴%d失败，错误码=%d", axis, rtn_stop);
      return false;
    }

    const short rtn_mode = GTN_SetEcatAxisMode(core_id_, axis, 10);
    if (rtn_mode != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "GTN_SetEcatAxisMode 设置扭矩模式失败，轴%d，错误码=%d",
          axis, rtn_mode);
      return false;
    }

    unsigned short actual_mode = static_cast<unsigned short>(-1);
    short rtn_get_mode = 0;
    bool mode_ok = false;

    for (int retry = 0; retry < 8; ++retry)
    {
      rtn_get_mode = GTN_GetEcatAxisMode(core_id_, axis, &actual_mode);
      if (rtn_get_mode == 0 && actual_mode == 10)
      {
        mode_ok = true;
        break;
      }
      std::this_thread::sleep_for(5ms);
    }

    if (!mode_ok)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "轴%d 期望切到扭矩模式(10)，实际=%d，错误码=%d", axis, actual_mode, rtn_get_mode);
      return false;
    }

    axis_in_torque_mode_[i] = true;
    command_sent_[i] = false;
    position_profile_configured_[i] = false;
    return true;
  }

  void invalidateAllPositionCommandCache()
  {
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      command_sent_[i] = false;
      position_profile_configured_[i] = false;
    }
  }

  void stopAllAxesMotionOnly()
  {
    long mask = 0;
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!isControlledAxisIndex(i) || !axis_enabled_[i])
      {
        continue;
      }
      const int axis = axis_ids_[i];
      mask |= (1L << (axis - 1));
    }

    if (mask == 0)
    {
      return;
    }

    const short rtn = GTN_Stop(core_id_, mask, 0);
    if (rtn != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "GTN_Stop 停止当前运动失败，mask=0x%lx，错误码=%d", mask, rtn);
    }
  }

  void restoreAllAxesToPositionMode()
  {
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!isControlledAxisIndex(i))
      {
        axis_in_torque_mode_[i] = false;
        command_sent_[i] = false;
        position_profile_configured_[i] = false;
        continue;
      }

      const int axis = axis_ids_[i];
      unsigned short actual_mode = static_cast<unsigned short>(-1);
      const short rtn_get_mode = GTN_GetEcatAxisMode(core_id_, axis, &actual_mode);
      if (rtn_get_mode == 0)
      {
        axis_in_torque_mode_[i] = (actual_mode == 10);
      }

      if (!axis_in_torque_mode_[i])
      {
        continue;
      }

      (void)ensureAxisInPositionMode(i);
    }

    invalidateAllPositionCommandCache();
  }

  // ===================== 执行位置输出 =====================
  void applyPositionOutput(const upperlimb_robot::ControlOutputCache &output)
  {
    long update_mask = 0;
    const bool dual_arm_torque_mode = isDualArmTorqueMode(applied_mode_);

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!isControlledAxisIndex(i))
      {
        command_sent_[i] = false;
        position_profile_configured_[i] = false;
        last_target_torque_[i] = 0.0;
        continue;
      }

      if (dual_arm_torque_mode && (output.use_torque[i] || !output.use_position[i]))
      {
        command_sent_[i] = false;
        position_profile_configured_[i] = false;
        continue;
      }

      const int axis = axis_ids_[i];

      unsigned short actual_mode = static_cast<unsigned short>(-1);
      const short rtn_get_mode = GTN_GetEcatAxisMode(core_id_, axis, &actual_mode);
      if (rtn_get_mode == 0)
      {
        axis_in_torque_mode_[i] = (actual_mode == 10);
      }

      if (output.use_torque[i])
      {
        command_sent_[i] = false;
        position_profile_configured_[i] = false;
        continue;
      }

      if (!output.use_position[i])
      {
        command_sent_[i] = false;
        position_profile_configured_[i] = false;
        if (axis_in_torque_mode_[i])
        {
          (void)ensureAxisInPositionMode(i);
        }
        continue;
      }

      if (axis_in_torque_mode_[i])
      {
        if (!ensureAxisInPositionMode(i))
        {
          continue;
        }
      }

      const double target_deg = output.target_position[i];
      const double target_pulse = upperlimb_robot::jointDegToPulse(axis, target_deg);

      const double vel_deg = output.target_velocity[i];
      const double acc_deg = output.target_acc[i];
      const double dec_deg = output.target_dec[i];
      const short smooth_time = output.target_smooth_time[i];

      bool need_reconfigure_profile = !position_profile_configured_[i];
      if (!need_reconfigure_profile)
      {
        if (std::abs(vel_deg - last_profile_velocity_[i]) > 1e-6 ||
            std::abs(acc_deg - last_profile_acc_[i]) > 1e-6 ||
            std::abs(dec_deg - last_profile_dec_[i]) > 1e-6 ||
            smooth_time != last_profile_smooth_time_[i])
        {
          need_reconfigure_profile = true;
        }
      }

      if (need_reconfigure_profile)
      {
        int32_t enc_pos = 0;
        short rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos);
        if (rtn != 0)
        {
          continue;
        }

        rtn = GTN_SetPrfPos(core_id_, axis, enc_pos);
        if (rtn != 0)
        {
          continue;
        }

        rtn = GTN_SetEncPos(core_id_, axis, enc_pos);
        if (rtn != 0)
        {
          continue;
        }

        rtn = GTN_SynchAxisPos(core_id_, static_cast<long>(1L << (axis - 1)));
        if (rtn != 0)
        {
          continue;
        }

        rtn = GTN_PrfTrap(core_id_, axis);
        if (rtn != 0)
        {
          continue;
        }

        TTrapPrm trap{};
        trap.acc = upperlimb_robot::jointAccelDegPerSec2ToPulsePerMs2(axis, acc_deg);
        trap.dec = upperlimb_robot::jointAccelDegPerSec2ToPulsePerMs2(axis, dec_deg);
        trap.smoothTime = smooth_time;
        trap.velStart = 0;

        rtn = GTN_SetTrapPrm(core_id_, axis, &trap);
        if (rtn != 0)
        {
          continue;
        }

        rtn = GTN_SetVel(
            core_id_,
            axis,
            upperlimb_robot::jointSpeedDegPerSecToPulsePerMs(axis, vel_deg));
        if (rtn != 0)
        {
          continue;
        }

        position_profile_configured_[i] = true;
        last_profile_velocity_[i] = vel_deg;
        last_profile_acc_[i] = acc_deg;
        last_profile_dec_[i] = dec_deg;
        last_profile_smooth_time_[i] = smooth_time;
      }

      if (command_sent_[i] &&
          std::abs(target_deg - last_target_position_[i]) < position_epsilon_)
      {
        continue;
      }

      const short rtn_set_pos =
          GTN_SetPos(core_id_, axis, static_cast<int32_t>(target_pulse));
      if (rtn_set_pos != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_SetPos 轴%d失败，错误码=%d", axis, rtn_set_pos);
        continue;
      }

      update_mask |= (1L << (axis - 1));
      last_target_position_[i] = target_deg;
      command_sent_[i] = true;
    }

    if (update_mask != 0)
    {
      const short rtn = GTN_Update(core_id_, update_mask);
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_Update 失败，mask=0x%lx，错误码=%d", update_mask, rtn);
      }
    }
  }

  // ===================== 执行扭矩输出 =====================
  void applyTorqueOutput(const upperlimb_robot::ControlOutputCache &output)
  {
    const bool dual_arm_torque_mode = isDualArmTorqueMode(applied_mode_);

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!isControlledAxisIndex(i))
      {
        axis_in_torque_mode_[i] = false;
        last_target_torque_[i] = 0.0;
        continue;
      }

      if (dual_arm_torque_mode && !output.use_torque[i])
      {
        last_target_torque_[i] = 0.0;
        continue;
      }

      const int axis = axis_ids_[i];

      unsigned short actual_mode = static_cast<unsigned short>(-1);
      const short rtn_get_mode = GTN_GetEcatAxisMode(core_id_, axis, &actual_mode);
      if (rtn_get_mode == 0)
      {
        axis_in_torque_mode_[i] = (actual_mode == 10);
      }

      if (!output.use_torque[i])
      {
        if (axis_in_torque_mode_[i])
        {
          (void)GTN_SetEcatAxisPT(core_id_, axis, 0);
          if (!dual_arm_torque_mode)
          {
            (void)ensureAxisInPositionMode(i);
          }
        }
        last_target_torque_[i] = 0.0;
        continue;
      }

      const double test_raw = upperlimb_robot::torqueNmToRaw(axis, 1.0);
      if (test_raw == 0.0)
      {
        continue;
      }

      const double target_torque_nm = output.target_torque[i];
      if (!std::isfinite(target_torque_nm))
      {
        continue;
      }

      bool switched_to_torque_mode = false;
      if (!axis_in_torque_mode_[i])
      {
        if (dual_arm_torque_mode)
        {
          RCLCPP_ERROR_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "双臂扭矩模式下轴%d 未处于扭矩模式，本轮跳过 PT", axis);
          continue;
        }

        if (!ensureAxisInTorqueMode(i))
        {
          continue;
        }
        switched_to_torque_mode = true;
      }

      if (!switched_to_torque_mode &&
          std::abs(target_torque_nm - last_target_torque_[i]) < torque_epsilon_)
      {
        continue;
      }

      const double torque_raw_double =
          upperlimb_robot::torqueNmToRaw(axis, target_torque_nm);
      if (!std::isfinite(torque_raw_double))
      {
        continue;
      }

      const short torque_raw = static_cast<short>(std::lround(torque_raw_double));
      const short rtn_pt = GTN_SetEcatAxisPT(core_id_, axis, torque_raw);
      if (rtn_pt != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_SetEcatAxisPT 发送扭矩失败，轴%d，目标扭矩=%.4f Nm，raw=%d，错误码=%d",
            axis, target_torque_nm, torque_raw, rtn_pt);
        continue;
      }

      last_target_torque_[i] = target_torque_nm;
    }
  }

  void clearAllTorqueCommands()
  {
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!isControlledAxisIndex(i))
      {
        axis_in_torque_mode_[i] = false;
        last_target_torque_[i] = 0.0;
        continue;
      }

      const int axis = axis_ids_[i];
      unsigned short actual_mode = static_cast<unsigned short>(-1);
      const short rtn_get_mode = GTN_GetEcatAxisMode(core_id_, axis, &actual_mode);
      if (rtn_get_mode == 0)
      {
        axis_in_torque_mode_[i] = (actual_mode == 10);
      }

      if (!axis_in_torque_mode_[i])
      {
        continue;
      }

      (void)GTN_SetEcatAxisPT(core_id_, axis, 0);
      last_target_torque_[i] = 0.0;
    }
  }

  bool stopAllAxesRaw()
  {
    if (!controller_opened_)
    {
      return true;
    }

    bool all_ok = true;
    int failed_axes = 0;

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!isControlledAxisIndex(i) || !axis_enabled_[i])
      {
        continue;
      }

      const int axis = axis_ids_[i];
      short rtn_stop = GTN_Stop(core_id_, static_cast<long>(1L << (axis - 1)), 0);
      if (rtn_stop != 0)
      {
        fprintf(stderr, "停止轴%d失败，错误码=%d\n", axis, rtn_stop);
        fflush(stderr);
        all_ok = false;
        failed_axes++;
      }

      short rtn_off = GTN_AxisOff(core_id_, axis);
      if (rtn_off != 0)
      {
        fprintf(stderr, "关闭轴%d使能失败，错误码=%d\n", axis, rtn_off);
        fflush(stderr);
        all_ok = false;
        failed_axes++;
      }
      else
      {
        axis_enabled_[i] = false;
      }
    }

    if (!all_ok)
    {
      fprintf(stderr, "共有 %d 个轴关闭过程出现错误。\n", failed_axes);
      fflush(stderr);
    }

    return all_ok;
  }

  // ===================== JointState 转换 =====================
  sensor_msgs::msg::JointState convertToJointStateMsg(
      const upperlimb_robot::MotorStateCache &state)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    msg.name = {
        "joint1", "joint2", "joint3", "joint4", "joint5",
        "joint6", "joint7", "joint8", "joint9", "joint10",
        "joint11", "joint12", "joint13", "joint14", "joint15",
        "joint16", "joint17", "joint18", "joint19", "joint20"};

    msg.position.resize(20);
    msg.velocity.resize(20);
    msg.effort.resize(20);

    constexpr double deg_to_rad = M_PI / 180.0;

    for (std::size_t src = 10, dst = 0; src < 20; ++src, ++dst)
    {
      msg.position[dst] = state.position[src] * deg_to_rad;
      msg.velocity[dst] = state.velocity[src] * deg_to_rad;
      msg.effort[dst] = state.torque[src];
    }

    for (std::size_t src = 0, dst = 10; src < 10; ++src, ++dst)
    {
      msg.position[dst] = state.position[src] * deg_to_rad;
      msg.velocity[dst] = state.velocity[src] * deg_to_rad;
      msg.effort[dst] = state.torque[src];
    }

    return msg;
  }

private:
  rclcpp::Subscription<upperlimb_robot::msg::ControlCommand>::SharedPtr control_command_sub_;
  rclcpp::Subscription<upperlimb_robot::msg::ExperimentConfig>::SharedPtr experiment_config_sub_;
  rclcpp::Subscription<upperlimb_robot::msg::LoggerCommand>::SharedPtr logger_command_sub_;

  rclcpp::Publisher<upperlimb_robot::msg::SystemState>::SharedPtr system_pub_;
  rclcpp::Publisher<upperlimb_robot::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  bool joint_init_ = false;
  bool controller_init_ = false;
  bool system_ready_ = false;

  bool controller_opened_ = false;
  bool ecat_started_ = false;
  bool shutdown_done_ = false;

  std::atomic<bool> running_{false};
  std::atomic<bool> state_ready_{false};

  std::thread state_read_thread_;
  std::thread control_thread_;

  std::chrono::steady_clock::time_point last_control_time_{};
  bool last_control_time_valid_ = false;

  std::atomic<int> current_mode_{upperlimb_robot::MODE_IDLE};
  int applied_mode_{upperlimb_robot::MODE_IDLE};
  std::atomic<int> applied_mode_atomic_{upperlimb_robot::MODE_IDLE};

  bool mode_enter_failed_ = false;
  int mode_enter_fallback_mode_ = upperlimb_robot::MODE_IDLE;

  // 默认 100 Hz，可通过启动参数一键切回 50 Hz
  std::atomic<int> state_hz_{100};
  std::atomic<int> control_hz_{100};

  // 实时性体检指标（单位：ms）
  std::atomic<double> last_actual_dt_ms_{0.0};
  std::atomic<double> last_state_elapsed_ms_{0.0};
  std::atomic<double> last_control_elapsed_ms_{0.0};

  std::string ecat_config_path_param_;
  std::string log_output_dir_{"data"};

  // 实验运行参数（受 MODE_IDLE 安全互锁保护）
  std::mutex exp_param_mutex_;
  upperlimb_robot::ExperimentRuntimeParams pending_exp_params_{};
  upperlimb_robot::ExperimentRuntimeParams active_exp_params_{};
  bool exp_params_dirty_ = false;

  // 异步 CSV 日志器与试验元信息
  upperlimb_robot::AsyncCsvLogger csv_logger_{4096};
  std::mutex logger_meta_mutex_;
  std::string log_subject_id_{"S01"};
  std::string log_condition_{"baseline"};
  std::string log_trial_id_{"T01"};
  std::string pending_event_tag_;
  std::atomic<std::uint64_t> log_tick_{0};
  std::chrono::steady_clock::time_point log_start_time_{};

  int status_read_divider_ = 20;
  int status_read_divider_count_ = 0;

  std::mutex state_mutex_;
  upperlimb_robot::MotorStateCache latest_motor_state_;

  std::mutex sensor_mutex_;
  upperlimb_robot::SensorStateCache latest_sensor_state_;

  std::mutex gtn_mutex_;

  int publish_divider_ = 5;
  int publish_divider_count_ = 0;

  int core_id_ = 1;

  std::array<int, upperlimb_robot::AXIS_NUM> axis_ids_ = {
      1, 2, 3, 4, 5,
      6, 7, 8, 9, 10,
      11, 12, 13, 14, 15,
      16, 17, 18, 19, 20};

  std::array<int32_t, upperlimb_robot::AXIS_NUM> enc_pos_{};
  std::array<bool, upperlimb_robot::AXIS_NUM> axis_enabled_{};

  std::array<double, upperlimb_robot::AXIS_NUM> last_target_position_{};
  std::array<bool, upperlimb_robot::AXIS_NUM> command_sent_{};

  std::array<bool, upperlimb_robot::AXIS_NUM> axis_in_torque_mode_{};
  std::array<double, upperlimb_robot::AXIS_NUM> last_target_torque_{};

  std::array<bool, upperlimb_robot::AXIS_NUM> position_profile_configured_{};
  std::array<double, upperlimb_robot::AXIS_NUM> last_profile_velocity_{};
  std::array<double, upperlimb_robot::AXIS_NUM> last_profile_acc_{};
  std::array<double, upperlimb_robot::AXIS_NUM> last_profile_dec_{};
  std::array<short, upperlimb_robot::AXIS_NUM> last_profile_smooth_time_{};

  // 100 Hz 连续位置轨迹每步增量约 0.01~0.10 deg；0.01 deg 兼顾顺滑流式更新与抑制微噪触发。
  double position_epsilon_ = 0.01;
  double torque_epsilon_ = 0.01;
};

static std::shared_ptr<RobotControlNode> g_node = nullptr;

void signal_handler(int signum)
{
  fprintf(stderr, "\n收到退出信号(%d)，开始安全关闭...\n", signum);
  fflush(stderr);

  if (g_node)
  {
    g_node->shutdownHardware();
  }

  rclcpp::shutdown();
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_node = std::make_shared<RobotControlNode>();

  rclcpp::spin(g_node);

  if (g_node)
  {
    g_node->shutdownHardware();
    g_node.reset();
  }

  return 0;
}
