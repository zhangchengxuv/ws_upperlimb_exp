#include "upperlimb_robot/async_csv_logger.hpp"
#include "upperlimb_robot/control_algorithms.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
  bool containsColumn(const std::vector<std::string> &cols, const std::string &name)
  {
    return std::find(cols.begin(), cols.end(), name) != cols.end();
  }

  std::vector<std::string> splitCsvLine(const std::string &line)
  {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ','))
    {
      out.push_back(item);
    }
    return out;
  }

  double maxAbsRange(const std::array<double, upperlimb_robot::AXIS_NUM> &arr,
                     std::size_t begin_idx,
                     std::size_t end_idx)
  {
    double m = 0.0;
    for (std::size_t i = begin_idx; i <= end_idx; ++i)
    {
      m = std::max(m, std::abs(arr[i]));
    }
    return m;
  }
} // namespace

int main(int argc, char **argv)
{
  bool keep_csv = false;
  bool show_sample = false;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--keep-csv")
      keep_csv = true;
    else if (arg == "--show-sample")
      show_sample = true;
  }

  using namespace upperlimb_robot;

  // ===================== 1. 核查预设位姿与被动正弦防跳变 =====================
  {
    MotorStateCache st{};
    ControlOutputCache out{};
    runLeftPresetPosition(st, out);
    if (!out.valid || std::abs(out.target_position[16] - 45.0) > 1e-6 || !out.use_position[10])
    {
      std::cerr << "[FAIL] runLeftPresetPosition 核查失败\n";
      return 1;
    }
    runRightPresetPosition(st, out);
    if (!out.valid || std::abs(out.target_position[6] - 45.0) > 1e-6 || !out.use_position[0])
    {
      std::cerr << "[FAIL] runRightPresetPosition 核查失败\n";
      return 1;
    }
    runBilateralPresetPosition(st, out);
    if (!out.valid ||
        !out.use_position[0] || !out.use_position[10] ||
        std::abs(out.target_position[6] - 45.0) > 1e-6 ||
        std::abs(out.target_position[16] - 45.0) > 1e-6)
    {
      std::cerr << "[FAIL] runBilateralPresetPosition(MODE=20) 核查失败\n";
      return 1;
    }

    st.position[13] = 5.0;  // L_GH1
    st.position[14] = -8.0; // L_GH2
    st.position[15] = 3.0;  // L_GH3
    st.position[16] = 45.0; // L_EB1
    st.velocity[16] = 1.5;

    SensorStateCache sens{};
    AlgorithmDiagData diag{};

    ExperimentRuntimeParams params{};
    params.passive_joint_profile = PASSIVE_JOINTS_COMBINED;
    params.passive_speed_level = PASSIVE_SPEED_STANDARD;
    setExperimentRuntimeParams(params);

    resetPassiveMode();
    runPassiveMode(st, sens, 0.01, out, &diag);
    if (!out.valid || std::abs(out.target_position[14] - (-8.0)) > 1e-6)
    {
      std::cerr << "[FAIL] runPassiveMode 首帧存在跳变\n";
      return 1;
    }
    // 运行 250 帧（2.5 s 过渐入期）后，构造跟踪误差验证 qd != qc
    for (int k = 0; k < 250; ++k)
    {
      runPassiveMode(st, sens, 0.01, out, &diag);
    }
    if (std::abs(diag.qd_deg[16] - out.target_position[16]) < 1e-4)
    {
      std::cerr << "[FAIL] MODE_PASSIVE(5) qd 与 qc 未分离\n";
      return 1;
    }

    // 重复进入防跳变验证
    st.position[14] = 12.0;
    resetPassiveMode();
    runPassiveMode(st, sens, 0.01, out, &diag);
    if (std::abs(out.target_position[14] - 12.0) > 1e-6)
    {
      std::cerr << "[FAIL] runPassiveMode 重复进入存在跳变\n";
      return 1;
    }

    // MODE_PASSIVE_SINE_PD_FF(6) 起步与 qd != qc 验证
    resetPassivePDFFMode();
    runPassivePDFFMode(st, sens, 0.01, out, &diag);
    if (!out.valid || std::abs(out.target_position[14] - 12.0) > 1e-6)
    {
      std::cerr << "[FAIL] runPassivePDFFMode 首帧存在跳变\n";
      return 1;
    }
    for (int k = 0; k < 250; ++k)
    {
      runPassivePDFFMode(st, sens, 0.01, out, &diag);
    }
    if (std::abs(diag.qd_deg[16] - out.target_position[16]) < 1e-4)
    {
      std::cerr << "[FAIL] MODE_PASSIVE_SINE_PD_FF(6) qd 与 qc 未分离\n";
      return 1;
    }

    std::cout << "[PASS] 预设位姿 + MODE_PASSIVE(5)/MODE_PASSIVE_SINE_PD_FF(6) 防跳变与 qd/qc 分离校验通过\n";
  }

  // ===================== 2. 全部 6 个力控/导纳模式真实 Pinocchio 逐字段校验 =====================
  {
    MotorStateCache st{};
    for (std::size_t i = 0; i <= 6; ++i)
    {
      st.enable[i] = true;
      st.position[i] = 5.0;
      st.velocity[i] = 6.0;
      st.torque[i] = -1.2;
    }
    for (std::size_t i = 10; i <= 16; ++i)
    {
      st.enable[i] = true;
      st.position[i] = 5.0;
      st.velocity[i] = 6.0;
      st.torque[i] = 1.2;
    }
    st.position[6] = 45.0;
    st.position[16] = 45.0;

    SensorStateCache sens{};
    sens.valid = true;
    for (std::size_t s = 0; s < FORCE_SENSOR_NUM; ++s)
    {
      sens.sensor[s] = {12.0, -5.0, 9.0, 0.40, -0.20, 0.30};
      sens.bias[s] = {2.0, -1.0, 1.0, 0.10, -0.05, 0.05};
    }

    // 2a. MODE_ZERO_FORCE(4)
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      runZeroForceMode(st, sens, out, &diag);
      if (!out.valid ||
          maxAbsRange(diag.tau_int_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 10, 16) < 1e-4 ||
          maxAbsRange(out.target_torque, 10, 16) < 1e-4 ||
          std::abs(diag.proc_ft[SENSOR_LEFT_ARM].fz - 8.4) > 1e-4)
      {
        std::cerr << "[FAIL] MODE_ZERO_FORCE(4) 字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_ZERO_FORCE(4) 真实 Pinocchio 字段校验通过\n";
    }

    // 2b. MODE_RIGHT_ZERO_FORCE(8)
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      runRightZeroForceMode(st, sens, out, &diag);
      if (!out.valid ||
          maxAbsRange(diag.tau_int_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 0, 6) < 1e-4 ||
          maxAbsRange(out.target_torque, 0, 6) < 1e-4 ||
          std::abs(diag.proc_ft[SENSOR_RIGHT_ARM].fz - 8.4) > 1e-4)
      {
        std::cerr << "[FAIL] MODE_RIGHT_ZERO_FORCE(8) 字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_RIGHT_ZERO_FORCE(8) 真实 Pinocchio 字段校验通过\n";
    }

    // 2c. MODE_BILATERAL_ZERO_FORCE(9)
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      runBilateralZeroForceMode(st, sens, out, &diag);
      if (!out.valid ||
          maxAbsRange(diag.tau_int_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 10, 16) < 1e-4 ||
          maxAbsRange(out.target_torque, 0, 6) < 1e-4 ||
          maxAbsRange(out.target_torque, 10, 16) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_BILATERAL_ZERO_FORCE(9) 字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_BILATERAL_ZERO_FORCE(9) 真实 Pinocchio 字段校验通过\n";
    }

    // 2d. MODE_ACTIVE(3)
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      resetActiveMode();
      runActiveMode(st, sens, 0.01, out, &diag);
      if (!out.valid ||
          maxAbsRange(diag.qa_deg, 10, 16) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 10, 16) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 10, 16) < 1e-4 ||
          maxAbsRange(out.target_torque, 10, 16) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_ACTIVE(3) 导纳 qa/dqa/ddqa 或力矩字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_ACTIVE(3) 真实 Pinocchio + qa/dqa/ddqa 字段校验通过\n";
    }

    // 2e. MODE_RIGHT_ACTIVE(14)
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      resetRightActiveMode();
      runRightActiveMode(st, sens, 0.01, out, &diag);
      if (!out.valid ||
          maxAbsRange(diag.qa_deg, 0, 6) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 0, 6) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 0, 6) < 1e-4 ||
          maxAbsRange(out.target_torque, 0, 6) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_RIGHT_ACTIVE(14) 导纳 qa/dqa/ddqa 或力矩字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_RIGHT_ACTIVE(14) 真实 Pinocchio + qa/dqa/ddqa 字段校验通过\n";
    }

    // 2f. MODE_BILATERAL_ACTIVE(13)
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      resetBilateralActiveMode();
      runBilateralActiveMode(st, sens, 0.01, out, &diag);
      if (!out.valid ||
          maxAbsRange(diag.qa_deg, 0, 6) < 1e-4 ||
          maxAbsRange(diag.qa_deg, 10, 16) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 0, 6) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 10, 16) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 0, 6) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 10, 16) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_BILATERAL_ACTIVE(13) 导纳 qa/dqa/ddqa 或力矩字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_BILATERAL_ACTIVE(13) 真实 Pinocchio + 双侧 qa/dqa/ddqa 字段校验通过\n";
    }

    // 2g. MODE_LEFT_ADAPTIVE_ACTIVE(21) - 创新 IVDA 左臂
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      resetLeftAdaptiveActiveMode();
      for (int k = 0; k < 20; ++k)
      {
        runLeftAdaptiveActiveMode(st, sens, 0.01, out, &diag);
      }
      if (!out.valid ||
          maxAbsRange(diag.qa_deg, 10, 16) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 10, 16) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 10, 16) < 1e-4 ||
          maxAbsRange(out.target_torque, 10, 16) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_LEFT_ADAPTIVE_ACTIVE(21) 字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_LEFT_ADAPTIVE_ACTIVE(21) IVDA 创新模型 + qa/dqa/ddqa 校验通过\n";
    }

    // 2h. MODE_RIGHT_ADAPTIVE_ACTIVE(22) - 创新 IVDA 右臂
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      resetRightAdaptiveActiveMode();
      for (int k = 0; k < 20; ++k)
      {
        runRightAdaptiveActiveMode(st, sens, 0.01, out, &diag);
      }
      if (!out.valid ||
          maxAbsRange(diag.qa_deg, 0, 6) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 0, 6) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_grav_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_fric_nm, 0, 6) < 1e-4 ||
          maxAbsRange(out.target_torque, 0, 6) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_RIGHT_ADAPTIVE_ACTIVE(22) 字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_RIGHT_ADAPTIVE_ACTIVE(22) IVDA 创新模型 + qa/dqa/ddqa 校验通过\n";
    }

    // 2i. MODE_BILATERAL_ADAPTIVE_ACTIVE(23) - 创新 IVDA 双侧
    {
      ControlOutputCache out{};
      AlgorithmDiagData diag{};
      resetBilateralAdaptiveActiveMode();
      for (int k = 0; k < 20; ++k)
      {
        runBilateralAdaptiveActiveMode(st, sens, 0.01, out, &diag);
      }
      if (!out.valid ||
          maxAbsRange(diag.qa_deg, 0, 6) < 1e-4 ||
          maxAbsRange(diag.qa_deg, 10, 16) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 0, 6) < 1e-4 ||
          maxAbsRange(diag.dqa_deg_s, 10, 16) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 0, 6) < 1e-4 ||
          maxAbsRange(diag.ddqa_deg_s2, 10, 16) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 0, 6) < 1e-4 ||
          maxAbsRange(diag.tau_int_nm, 10, 16) < 1e-4 ||
          maxAbsRange(out.target_torque, 0, 6) < 1e-4 ||
          maxAbsRange(out.target_torque, 10, 16) < 1e-4)
      {
        std::cerr << "[FAIL] MODE_BILATERAL_ADAPTIVE_ACTIVE(23) 字段校验失败\n";
        return 1;
      }
      std::cout << "[PASS] MODE_BILATERAL_ADAPTIVE_ACTIVE(23) IVDA 创新模型 + 双侧 qa/dqa/ddqa 校验通过\n";
    }
  }

  // ===================== 3. 异步 CSV 日志器 100 Hz × 10 s 真实算法模拟测试 =====================
  const std::string test_dir = "/tmp/upperlimb_exp_logger_test";
  AsyncCsvLogger logger(4096);

  if (!logger.startSession(test_dir, "S_TEST", "bilateral_adm_std", "T01"))
  {
    std::cerr << "[FAIL] 无法启动异步 CSV 日志会话\n";
    return 1;
  }

  const std::string csv_path = logger.currentFilePath();
  constexpr int TARGET_HZ = 100;
  constexpr int DURATION_S = 10;
  constexpr int TOTAL_FRAMES = TARGET_HZ * DURATION_S; // 1000 帧
  const auto period = std::chrono::microseconds(1000000 / TARGET_HZ);

  ExperimentRuntimeParams exp_params{};
  exp_params.passive_joint_profile = PASSIVE_JOINT_J7_ONLY;
  exp_params.passive_speed_level = PASSIVE_SPEED_SLOW;
  exp_params.admittance_damping_level = ADMITTANCE_DAMPING_STANDARD;
  setExperimentRuntimeParams(exp_params);
  resetBilateralActiveMode();

  std::vector<double> push_us_samples;
  push_us_samples.reserve(TOTAL_FRAMES);

  const auto test_start = std::chrono::steady_clock::now();
  auto next_wake = test_start;
  auto prev_tick_time = test_start;

  for (int i = 0; i < TOTAL_FRAMES; ++i)
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt_s =
        (i == 0) ? 0.01 : std::chrono::duration<double>(now - prev_tick_time).count();
    prev_tick_time = now;
    const double t_s = std::chrono::duration<double>(now - test_start).count();

    MotorStateCache motor{};
    SensorStateCache sensor{};
    ControlOutputCache output{};
    AlgorithmDiagData diag{};

    for (std::size_t idx = 0; idx <= 6; ++idx)
    {
      motor.enable[idx] = true;
      motor.position[idx] = (idx == 6) ? 45.0 : 5.0;
      motor.velocity[idx] = 3.0;
      motor.torque[idx] = -1.5;
    }
    for (std::size_t idx = 10; idx <= 16; ++idx)
    {
      motor.enable[idx] = true;
      motor.position[idx] = (idx == 16) ? 45.0 : 5.0;
      motor.velocity[idx] = 3.0;
      motor.torque[idx] = 1.5;
    }

    sensor.valid = true;
    // raw_ft 与 bias 明显不同，且大臂 Fz 会受 ARM_FORCE_GAIN_WORLD_X=1.20 放大
    sensor.sensor[SENSOR_RIGHT_HAND] = {5.5, -2.2, 4.8, 0.25, -0.10, 0.15};
    sensor.bias[SENSOR_RIGHT_HAND] = {0.5, -0.2, 0.8, 0.05, -0.02, 0.05};
    sensor.sensor[SENSOR_RIGHT_ARM] = {15.5, -4.2, 8.8, 0.35, -0.12, 0.25};
    sensor.bias[SENSOR_RIGHT_ARM] = {1.5, -0.2, 0.8, 0.05, -0.02, 0.05};
    sensor.sensor[SENSOR_LEFT_HAND] = {6.5, -2.5, 5.2, 0.20, -0.08, 0.12};
    sensor.bias[SENSOR_LEFT_HAND] = {0.5, -0.5, 0.2, 0.02, -0.01, 0.02};
    sensor.sensor[SENSOR_LEFT_ARM] = {15.5, -4.2, 8.8, 0.35, -0.12, 0.25};
    sensor.bias[SENSOR_LEFT_ARM] = {1.5, -0.2, 0.8, 0.05, -0.02, 0.05};

    // 真实调用 Pinocchio 双侧主动导纳算法填充全部诊断字段
    runBilateralActiveMode(motor, sensor, dt_s, output, &diag);

    LogRecord rec{};
    copySanitizedTag(rec.subject_id, sizeof(rec.subject_id), "S_TEST");
    copySanitizedTag(rec.condition, sizeof(rec.condition), "bilateral_adm_std");
    copySanitizedTag(rec.trial_id, sizeof(rec.trial_id), "T01");
    copySanitizedTag(rec.param_preset, sizeof(rec.param_preset),
                     formatParamPreset(MODE_BILATERAL_ACTIVE, exp_params));
    if (i == 0)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "START");
    else if (i == 300)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "HOLD");
    else if (i == 600)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "RELEASE");
    else if (i == TOTAL_FRAMES - 1)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "END");

    rec.mode = MODE_BILATERAL_ACTIVE;
    rec.passive_joint_profile = exp_params.passive_joint_profile;
    rec.passive_speed_level = exp_params.passive_speed_level;
    rec.admittance_damping_level = exp_params.admittance_damping_level;
    rec.tick = static_cast<std::uint64_t>(i + 1);
    rec.monotonic_time_s = t_s;
    rec.actual_dt_ms = dt_s * 1000.0;
    rec.state_elapsed_ms = 1.2;
    rec.control_elapsed_ms = 0.6;
    rec.dropped_frames = logger.droppedFrames();

    fillLogRecordFromStates(rec, motor, sensor, output, diag);

    const auto t_push_0 = std::chrono::steady_clock::now();
    const bool pushed = logger.push(rec);
    const auto t_push_1 = std::chrono::steady_clock::now();

    if (!pushed)
    {
      std::cerr << "[FAIL] 第 " << i << " 帧入队失败\n";
      return 1;
    }

    const double push_us =
        std::chrono::duration<double, std::micro>(t_push_1 - t_push_0).count();
    push_us_samples.push_back(push_us);

    next_wake += period;
    std::this_thread::sleep_until(next_wake);
  }

  logger.stopSession();

  const std::uint64_t dropped = logger.droppedFrames();
  const std::uint64_t written = logger.writtenFrames();

  std::sort(push_us_samples.begin(), push_us_samples.end());
  const double avg_us =
      std::accumulate(push_us_samples.begin(), push_us_samples.end(), 0.0) /
      static_cast<double>(push_us_samples.size());
  const double p99_us = push_us_samples[static_cast<std::size_t>(0.99 * push_us_samples.size())];
  const double max_us = push_us_samples.back();

  std::cout << "[INFO] 100 Hz x 10 s 写入完成: written=" << written
            << " dropped=" << dropped
            << " push_avg=" << avg_us << " us"
            << " push_p99=" << p99_us << " us"
            << " push_max=" << max_us << " us\n";

  if (dropped != 0 || written != static_cast<std::uint64_t>(TOTAL_FRAMES))
  {
    std::cerr << "[FAIL] 丢帧或帧数不匹配! written=" << written
              << " dropped=" << dropped << "\n";
    return 1;
  }

  // ===================== 4. 核查带物理单位的 CSV 表头与内容 =====================
  std::ifstream ifs(csv_path);
  if (!ifs.is_open())
  {
    std::cerr << "[FAIL] 无法打开生成的 CSV 文件: " << csv_path << "\n";
    return 1;
  }

  std::string header_line;
  std::getline(ifs, header_line);
  const auto cols = splitCsvLine(header_line);

  const std::vector<std::string> required_cols = {
      "subject_id", "condition", "trial_id", "mode", "param_preset", "event_tag",
      "tick", "monotonic_time_s", "actual_dt_ms", "state_elapsed_ms",
      "control_elapsed_ms", "dropped_frames",
      "qd_L_EB1_deg", "qc_L_EB1_deg", "q_L_SG1_deg", "q_R_SG1_deg",
      "dq_L_EB1_deg_s", "en_L_EB1", "sat_L_EB1",
      "raw_ft_rh_fx_N", "raw_ft_la_fz_N", "ft_bias_la_fz_N", "proc_ft_la_fz_N",
      "raw_ft_la_mx_Nm", "proc_ft_la_mx_Nm",
      "tau_int_L_EB1_Nm", "qa_L_EB1_deg", "dqa_L_EB1_deg_s", "ddqa_L_EB1_deg_s2",
      "tau_grav_L_EB1_Nm", "tau_fric_L_EB1_Nm", "tau_cmd_L_EB1_Nm", "tau_meas_L_EB1_Nm"};

  for (const auto &rc : required_cols)
  {
    if (!containsColumn(cols, rc))
    {
      std::cerr << "[FAIL] CSV 表头缺少必需带单位列: " << rc << "\n";
      return 1;
    }
  }

  auto colIndex = [&](const std::string &name) -> std::size_t
  {
    return static_cast<std::size_t>(
        std::find(cols.begin(), cols.end(), name) - cols.begin());
  };

  const std::size_t idx_event = colIndex("event_tag");
  const std::size_t idx_raw_fz = colIndex("raw_ft_la_fz_N");
  const std::size_t idx_bias_fz = colIndex("ft_bias_la_fz_N");
  const std::size_t idx_proc_fz = colIndex("proc_ft_la_fz_N");
  const std::size_t idx_ddqa = colIndex("ddqa_L_EB1_deg_s2");

  int data_rows = 0;
  bool seen_start = false;
  bool seen_hold = false;
  bool seen_release = false;
  bool seen_end = false;
  bool ft_separated_ok = true;
  bool ddqa_nonzero = false;

  std::vector<std::string> first_two_rows;
  std::string line;
  while (std::getline(ifs, line))
  {
    if (line.empty())
      continue;
    if (first_two_rows.size() < 2)
    {
      first_two_rows.push_back(line);
    }
    const auto vals = splitCsvLine(line);
    if (vals.size() != cols.size())
    {
      std::cerr << "[FAIL] 第 " << data_rows << " 行列数不匹配: "
                << vals.size() << " vs " << cols.size() << "\n";
      return 1;
    }

    if (vals[idx_event] == "START")
      seen_start = true;
    if (vals[idx_event] == "HOLD")
      seen_hold = true;
    if (vals[idx_event] == "RELEASE")
      seen_release = true;
    if (vals[idx_event] == "END")
      seen_end = true;

    // 验证 raw_ft_la_fz_N=8.8, ft_bias_la_fz_N=0.8, 连续软死区 dz=1.0 后 proc_ft_la_fz_N=(8.8-0.8-1.0)*1.20=8.4
    const double raw_v = std::stod(vals[idx_raw_fz]);
    const double bias_v = std::stod(vals[idx_bias_fz]);
    const double proc_v = std::stod(vals[idx_proc_fz]);
    if (std::abs(raw_v - 8.8) > 1e-4 ||
        std::abs(bias_v - 0.8) > 1e-4 ||
        std::abs(proc_v - 8.4) > 1e-4)
    {
      ft_separated_ok = false;
    }

    if (std::abs(std::stod(vals[idx_ddqa])) > 1e-4)
    {
      ddqa_nonzero = true;
    }

    data_rows++;
  }
  ifs.close();

  if (data_rows != TOTAL_FRAMES || !ft_separated_ok || !ddqa_nonzero ||
      !seen_start || !seen_hold || !seen_release || !seen_end)
  {
    std::cerr << "[FAIL] CSV 内容核查失败: rows=" << data_rows
              << " ft_sep=" << ft_separated_ok
              << " ddqa_nonzero=" << ddqa_nonzero
              << " tags=(" << seen_start << "," << seen_hold << ","
              << seen_release << "," << seen_end << ")\n";
    return 1;
  }

  std::cout << "[PASS] CSV 表头完整（共 " << cols.size() << " 列，全部物理量带单位），共 "
            << data_rows << " 行，raw_ft/ft_bias/proc_ft 严格分离，ddqa 非零，事件打标完整！\n";

  if (show_sample)
  {
    std::cout << "\n=== CSV_HEADER_LINE_1 ===\n" << header_line << "\n";
    if (first_two_rows.size() >= 1)
      std::cout << "\n=== CSV_DATA_LINE_2 ===\n" << first_two_rows[0] << "\n";
    if (first_two_rows.size() >= 2)
      std::cout << "\n=== CSV_DATA_LINE_3 ===\n" << first_two_rows[1] << "\n";
  }

  if (!keep_csv)
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir, ec);
    std::cout << "[INFO] 已清理临时测试 CSV 目录: " << test_dir << "\n";
  }

  return 0;
}
