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
} // namespace

int main(int argc, char **argv)
{
  bool keep_csv = false;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--keep-csv")
    {
      keep_csv = true;
    }
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

    // 检查被动正弦（PID 与 PD+FF）在起步和重复进入时是否存在跳变
    st.position[13] = 5.0;  // L_GH1
    st.position[14] = -8.0; // L_GH2（原 phase=pi/2，最易跳变）
    st.position[15] = 3.0;  // L_GH3
    st.position[16] = 45.0; // L_EB1

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
      std::cerr << "[FAIL] runPassiveMode 首帧存在跳变! target_j5="
                << out.target_position[14] << "\n";
      return 1;
    }

    // 模拟运行 3 秒后更换起点并重复进入，验证无跳变
    for (int k = 0; k < 300; ++k)
    {
      runPassiveMode(st, sens, 0.01, out, &diag);
    }
    st.position[14] = 12.0;
    resetPassiveMode();
    runPassiveMode(st, sens, 0.01, out, &diag);
    if (std::abs(out.target_position[14] - 12.0) > 1e-6)
    {
      std::cerr << "[FAIL] runPassiveMode 重复进入存在跳变! target_j5="
                << out.target_position[14] << "\n";
      return 1;
    }

    resetPassivePDFFMode();
    runPassivePDFFMode(st, sens, 0.01, out, &diag);
    if (!out.valid || std::abs(out.target_position[14] - 12.0) > 1e-6)
    {
      std::cerr << "[FAIL] runPassivePDFFMode 首帧存在跳变! target_j5="
                << out.target_position[14] << "\n";
      return 1;
    }

    std::cout << "[PASS] 预设位姿原样校验 & 被动正弦(PID / PD+FF)起步/重入防跳变校验通过\n";
  }

  // ===================== 2. 异步 CSV 日志器 100 Hz × 10 s 模拟测试 =====================
  const std::string test_dir = "/tmp/upperlimb_exp_logger_test";
  AsyncCsvLogger logger(4096);

  if (!logger.startSession(test_dir, "S_TEST", "passive_eb1_slow", "T01"))
  {
    std::cerr << "[FAIL] 无法启动异步 CSV 日志会话\n";
    return 1;
  }

  const std::string csv_path = logger.currentFilePath();
  constexpr int TARGET_HZ = 100;
  constexpr int DURATION_S = 10;
  constexpr int TOTAL_FRAMES = TARGET_HZ * DURATION_S; // 1000 帧
  const auto period = std::chrono::microseconds(1000000 / TARGET_HZ);

  std::vector<double> push_us_samples;
  push_us_samples.reserve(TOTAL_FRAMES);

  const auto test_start = std::chrono::steady_clock::now();
  auto next_wake = test_start;
  auto prev_tick_time = test_start;

  for (int i = 0; i < TOTAL_FRAMES; ++i)
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt_ms =
        (i == 0) ? 10.0 : std::chrono::duration<double, std::milli>(now - prev_tick_time).count();
    prev_tick_time = now;
    const double t_s = std::chrono::duration<double>(now - test_start).count();

    MotorStateCache motor{};
    SensorStateCache sensor{};
    ControlOutputCache output{};
    AlgorithmDiagData diag{};

    sensor.valid = true;
    // 故意让 raw_ft 与 bias / proc_ft 数值明显不同，验证严格分离
    sensor.sensor[SENSOR_LEFT_ARM] = {15.5, -4.2, 8.8, 0.35, -0.12, 0.25};
    sensor.bias[SENSOR_LEFT_ARM] = {1.5, -0.2, 0.8, 0.05, -0.02, 0.05};
    diag.proc_ft[SENSOR_LEFT_ARM] = {14.0, -4.0, 9.6, 0.30, -0.10, 0.20};

    motor.position[16] = 45.0 + 10.0 * std::sin(2.0 * M_PI * 0.1 * t_s);
    motor.velocity[16] = 2.0 * M_PI * 0.1 * 10.0 * std::cos(2.0 * M_PI * 0.1 * t_s);
    motor.torque[16] = 2.1;
    motor.enable[16] = true;

    output.use_position[16] = true;
    output.target_position[16] = motor.position[16] + 0.2;
    diag.qd_deg[16] = motor.position[16] + 0.1;
    diag.qa_deg[16] = motor.position[16];
    diag.dqa_deg_s[16] = motor.velocity[16];
    diag.tau_int_nm[16] = 1.25;
    diag.tau_grav_nm[16] = 1.85;
    diag.tau_fric_nm[16] = 0.25;

    LogRecord rec{};
    copySanitizedTag(rec.subject_id, sizeof(rec.subject_id), "S_TEST");
    copySanitizedTag(rec.condition, sizeof(rec.condition), "passive_eb1_slow");
    copySanitizedTag(rec.trial_id, sizeof(rec.trial_id), "T01");
    if (i == 0)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "START");
    else if (i == 300)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "HOLD");
    else if (i == 600)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "RELEASE");
    else if (i == TOTAL_FRAMES - 1)
      copySanitizedTag(rec.event_tag, sizeof(rec.event_tag), "END");

    rec.mode = MODE_PASSIVE;
    rec.passive_joint_profile = PASSIVE_JOINT_J7_ONLY;
    rec.passive_speed_level = PASSIVE_SPEED_SLOW;
    rec.admittance_damping_level = ADMITTANCE_DAMPING_STANDARD;
    rec.tick = static_cast<std::uint64_t>(i + 1);
    rec.monotonic_time_s = t_s;
    rec.actual_dt_ms = dt_ms;
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

  // ===================== 3. 核查 CSV 表头与内容 =====================
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
      "subject_id", "condition", "trial_id", "mode", "event_tag",
      "tick", "monotonic_time_s", "actual_dt_ms", "state_elapsed_ms",
      "control_elapsed_ms", "dropped_frames",
      "qd_L_EB1", "qc_L_EB1", "q_L_SG1", "q_R_SG1", "dq_L_EB1", "en_L_EB1", "sat_L_EB1",
      "raw_ft_rh_fx", "raw_ft_la_fx", "ft_bias_la_fx", "proc_ft_la_fx",
      "tau_int_L_EB1", "qa_L_EB1", "dqa_L_EB1",
      "tau_grav_L_EB1", "tau_fric_L_EB1", "tau_cmd_L_EB1", "tau_fb_L_EB1"};

  for (const auto &rc : required_cols)
  {
    if (!containsColumn(cols, rc))
    {
      std::cerr << "[FAIL] CSV 表头缺少必需列: " << rc << "\n";
      return 1;
    }
  }

  auto colIndex = [&](const std::string &name) -> std::size_t
  {
    return static_cast<std::size_t>(
        std::find(cols.begin(), cols.end(), name) - cols.begin());
  };

  const std::size_t idx_event = colIndex("event_tag");
  const std::size_t idx_raw_fx = colIndex("raw_ft_la_fx");
  const std::size_t idx_bias_fx = colIndex("ft_bias_la_fx");
  const std::size_t idx_proc_fx = colIndex("proc_ft_la_fx");

  int data_rows = 0;
  bool seen_start = false;
  bool seen_hold = false;
  bool seen_release = false;
  bool seen_end = false;
  bool ft_separated_ok = true;

  std::string line;
  while (std::getline(ifs, line))
  {
    if (line.empty())
      continue;
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

    const double raw_v = std::stod(vals[idx_raw_fx]);
    const double bias_v = std::stod(vals[idx_bias_fx]);
    const double proc_v = std::stod(vals[idx_proc_fx]);
    if (std::abs(raw_v - 15.5) > 1e-4 ||
        std::abs(bias_v - 1.5) > 1e-4 ||
        std::abs(proc_v - 14.0) > 1e-4)
    {
      ft_separated_ok = false;
    }

    data_rows++;
  }
  ifs.close();

  if (data_rows != TOTAL_FRAMES || !ft_separated_ok ||
      !seen_start || !seen_hold || !seen_release || !seen_end)
  {
    std::cerr << "[FAIL] CSV 内容核查失败: rows=" << data_rows
              << " ft_sep=" << ft_separated_ok
              << " tags=(" << seen_start << "," << seen_hold << ","
              << seen_release << "," << seen_end << ")\n";
    return 1;
  }

  std::cout << "[PASS] CSV 表头完整（共 " << cols.size() << " 列），共 "
            << data_rows << " 行，raw_ft/ft_bias/proc_ft 严格分离，事件打标完整！\n";

  if (!keep_csv)
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir, ec);
    std::cout << "[INFO] 已清理临时测试 CSV 目录: " << test_dir << "\n";
  }

  return 0;
}
