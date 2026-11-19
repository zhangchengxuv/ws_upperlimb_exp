#include "upperlimb_robot/async_csv_logger.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace upperlimb_robot
{

  namespace
  {
    // 14 个主动关节在 20 轴控制数组中的映射索引
    constexpr std::array<std::size_t, LogRecord::ACTIVE_DOF> ACTIVE_CTRL_INDICES = {
        0, 1, 2, 3, 4, 5, 6,       // 右臂 R1~R7 (joint11~joint17)
        10, 11, 12, 13, 14, 15, 16 // 左臂 L1~L7 (joint1~joint7)
    };

    const std::array<const char *, LogRecord::ACTIVE_DOF> ACTIVE_JOINT_NAMES = {
        "R_SG1", "R_SG2", "R_SG3", "R_GH1", "R_GH2", "R_GH3", "R_EB1",
        "L_SG1", "L_SG2", "L_SG3", "L_GH1", "L_GH2", "L_GH3", "L_EB1"};

    const std::array<const char *, FORCE_SENSOR_NUM> SENSOR_NAMES = {
        "rh", "ra", "lh", "la"};

    // 清理文件名中的非法字符
    std::string sanitizeFilenamePart(const std::string &src, const std::string &fallback)
    {
      std::string out;
      out.reserve(src.size());
      for (char ch : src)
      {
        if ((ch >= 'a' && ch <= 'z') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '_' || ch == '-')
        {
          out.push_back(ch);
        }
        else if (ch == ' ' || ch == '.' || ch == '/')
        {
          out.push_back('_');
        }
      }
      return out.empty() ? fallback : out;
    }

    std::string makeTimestampString()
    {
      const auto now = std::chrono::system_clock::now();
      const std::time_t now_t = std::chrono::system_clock::to_time_t(now);
      std::tm local_tm{};
      localtime_r(&now_t, &local_tm);

      char buf[32]{};
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &local_tm);
      return std::string(buf);
    }

    void appendJointColumns(std::ostringstream &oss, const char *prefix)
    {
      for (const char *name : ACTIVE_JOINT_NAMES)
      {
        oss << "," << prefix << "_" << name;
      }
    }

    void appendForceColumns(std::ostringstream &oss, const char *prefix)
    {
      for (const char *sname : SENSOR_NAMES)
      {
        oss << "," << prefix << "_" << sname << "_fx"
            << "," << prefix << "_" << sname << "_fy"
            << "," << prefix << "_" << sname << "_fz"
            << "," << prefix << "_" << sname << "_mx"
            << "," << prefix << "_" << sname << "_my"
            << "," << prefix << "_" << sname << "_mz";
      }
    }

  } // namespace

  void copySanitizedTag(char *dst, std::size_t dst_size, const std::string &src)
  {
    if (dst == nullptr || dst_size == 0)
    {
      return;
    }
    std::size_t out_idx = 0;
    for (std::size_t i = 0; i < src.size() && out_idx + 1 < dst_size; ++i)
    {
      char c = src[i];
      if (c == ',' || c == '\n' || c == '\r')
      {
        c = '_';
      }
      dst[out_idx++] = c;
    }
    dst[out_idx] = '\0';
  }

  void fillLogRecordFromStates(
      LogRecord &rec,
      const MotorStateCache &motor_state,
      const SensorStateCache &sensor_state,
      const ControlOutputCache &output,
      const AlgorithmDiagData &diag)
  {
    for (std::size_t k = 0; k < LogRecord::ACTIVE_DOF; ++k)
    {
      const std::size_t idx = ACTIVE_CTRL_INDICES[k];
      rec.qd[k] = diag.qd_deg[idx];
      rec.qc[k] = output.use_position[idx] ? output.target_position[idx] : 0.0;
      rec.q[k] = motor_state.position[idx];
      rec.dq[k] = motor_state.velocity[idx];
      rec.en[k] = motor_state.enable[idx] ? 1 : 0;
      rec.sat[k] = diag.saturated[idx] ? 1 : 0;

      rec.tau_int[k] = diag.tau_int_nm[idx];
      rec.qa[k] = diag.qa_deg[idx];
      rec.dqa[k] = diag.dqa_deg_s[idx];
      rec.tau_grav[k] = diag.tau_grav_nm[idx];
      rec.tau_fric[k] = diag.tau_fric_nm[idx];
      rec.tau_cmd[k] = output.use_torque[idx] ? output.target_torque[idx] : 0.0;
      rec.tau_fb[k] = motor_state.torque[idx];
    }

    rec.sat_ft_any = diag.ft_saturated ? 1 : 0;

    for (std::size_t s = 0; s < FORCE_SENSOR_NUM; ++s)
    {
      rec.raw_ft[s] = sensor_state.sensor[s];
      rec.ft_bias[s] = sensor_state.bias[s];
      rec.proc_ft[s] = diag.proc_ft[s];
    }
  }

  std::string AsyncCsvLogger::buildCsvHeader()
  {
    std::ostringstream oss;

    // 1. 试验信息
    oss << "subject_id,condition,trial_id,mode,event_tag,"
        << "passive_joint_profile,passive_speed_level,admittance_damping_level";

    // 2. 实时性体检
    oss << ",tick,monotonic_time_s,actual_dt_ms,state_elapsed_ms,"
        << "control_elapsed_ms,dropped_frames";

    // 3. 轨迹与状态（14 主动关节）
    appendJointColumns(oss, "qd");
    appendJointColumns(oss, "qc");
    appendJointColumns(oss, "q");
    appendJointColumns(oss, "dq");
    appendJointColumns(oss, "en");
    appendJointColumns(oss, "sat");
    oss << ",sat_ft_any";

    // 4. 六维力（原始值 raw_ft_*、零偏 ft_bias_*、处理值 proc_ft_* 严格分开命名）
    appendForceColumns(oss, "raw_ft");
    appendForceColumns(oss, "ft_bias");
    appendForceColumns(oss, "proc_ft");
    appendJointColumns(oss, "tau_int");

    // 5. 导纳与力矩分解
    appendJointColumns(oss, "qa");
    appendJointColumns(oss, "dqa");
    appendJointColumns(oss, "tau_grav");
    appendJointColumns(oss, "tau_fric");
    appendJointColumns(oss, "tau_cmd");
    appendJointColumns(oss, "tau_fb");

    return oss.str();
  }

  AsyncCsvLogger::AsyncCsvLogger(std::size_t queue_capacity)
      : capacity_(queue_capacity < 16 ? 16 : queue_capacity),
        ring_(capacity_)
  {
    writer_thread_ = std::thread(&AsyncCsvLogger::writerLoop, this);
  }

  AsyncCsvLogger::~AsyncCsvLogger()
  {
    stopSession();
    worker_stop_.store(true, std::memory_order_release);
    if (writer_thread_.joinable())
    {
      writer_thread_.join();
    }
  }

  bool AsyncCsvLogger::startSession(
      const std::string &output_dir,
      const std::string &subject_id,
      const std::string &condition,
      const std::string &trial_id)
  {
    // 若之前仍在记录，先安全停止并刷盘
    stopSession();

    std::lock_guard<std::mutex> lock(session_mutex_);

    std::string dir = output_dir.empty() ? "data" : output_dir;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec)
    {
      std::cerr << "[AsyncCsvLogger] 无法创建日志目录: " << dir
                << " err=" << ec.message() << std::endl;
      return false;
    }

    const std::string filename =
        sanitizeFilenamePart(subject_id, "S00") + "_" +
        sanitizeFilenamePart(condition, "cond") + "_" +
        sanitizeFilenamePart(trial_id, "T00") + "_" +
        makeTimestampString() + ".csv";

    const std::filesystem::path full_path = std::filesystem::path(dir) / filename;

    file_.open(full_path, std::ios::out | std::ios::trunc);
    if (!file_.is_open())
    {
      std::cerr << "[AsyncCsvLogger] 无法打开 CSV 文件: "
                << full_path.string() << std::endl;
      return false;
    }

    file_ << buildCsvHeader() << "\n";
    file_.flush();

    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
    dropped_frames_.store(0, std::memory_order_relaxed);
    written_frames_.store(0, std::memory_order_relaxed);
    current_file_path_ = full_path.string();

    logging_active_.store(true, std::memory_order_release);
    std::cout << "[AsyncCsvLogger] 开始记录 CSV: " << current_file_path_ << std::endl;
    return true;
  }

  void AsyncCsvLogger::stopSession()
  {
    // 先通知生产者停止入队
    logging_active_.store(false, std::memory_order_release);

    // 给正在执行的最后一次 push() 留出极短退出窗口
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    std::lock_guard<std::mutex> lock(session_mutex_);
    if (file_.is_open())
    {
      drainQueueToStream();
      file_.flush();
      file_.close();
      std::cout << "[AsyncCsvLogger] 停止记录 CSV: " << current_file_path_
                << " written=" << written_frames_.load(std::memory_order_relaxed)
                << " dropped=" << dropped_frames_.load(std::memory_order_relaxed)
                << std::endl;
    }
  }

  bool AsyncCsvLogger::push(const LogRecord &record)
  {
    if (!logging_active_.load(std::memory_order_acquire))
    {
      return false;
    }

    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % capacity_;
    const std::size_t tail = tail_.load(std::memory_order_acquire);

    if (next == tail)
    {
      // 队列已满：丢帧并计数，绝不阻塞实时控制线程
      dropped_frames_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    ring_[head] = record;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool AsyncCsvLogger::isActive() const
  {
    return logging_active_.load(std::memory_order_acquire);
  }

  std::uint64_t AsyncCsvLogger::droppedFrames() const
  {
    return dropped_frames_.load(std::memory_order_relaxed);
  }

  std::uint64_t AsyncCsvLogger::writtenFrames() const
  {
    return written_frames_.load(std::memory_order_relaxed);
  }

  std::string AsyncCsvLogger::currentFilePath() const
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    return current_file_path_;
  }

  void AsyncCsvLogger::writerLoop()
  {
    int flush_tick = 0;
    while (!worker_stop_.load(std::memory_order_acquire))
    {
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (file_.is_open())
        {
          drainQueueToStream();
          if (++flush_tick >= 20) // 约每 200 ms 刷盘一次
          {
            file_.flush();
            flush_tick = 0;
          }
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 退出前确保剩余数据写完
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (file_.is_open())
    {
      drainQueueToStream();
      file_.flush();
      file_.close();
    }
  }

  void AsyncCsvLogger::drainQueueToStream()
  {
    std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);

    while (tail != head)
    {
      writeRecordLine(ring_[tail]);
      tail = (tail + 1) % capacity_;
      written_frames_.fetch_add(1, std::memory_order_relaxed);
    }

    tail_.store(tail, std::memory_order_release);
  }

  void AsyncCsvLogger::writeRecordLine(const LogRecord &rec)
  {
    // 后台写盘线程中执行格式化，不影响实时控制周期
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5);

    oss << rec.subject_id << ","
        << rec.condition << ","
        << rec.trial_id << ","
        << rec.mode << ","
        << rec.event_tag << ","
        << rec.passive_joint_profile << ","
        << rec.passive_speed_level << ","
        << rec.admittance_damping_level << ","
        << rec.tick << ","
        << std::setprecision(6) << rec.monotonic_time_s << ","
        << std::setprecision(4) << rec.actual_dt_ms << ","
        << rec.state_elapsed_ms << ","
        << rec.control_elapsed_ms << ","
        << rec.dropped_frames;

    oss << std::setprecision(5);

    auto write_doubles = [&](const std::array<double, LogRecord::ACTIVE_DOF> &arr)
    {
      for (std::size_t i = 0; i < LogRecord::ACTIVE_DOF; ++i)
      {
        oss << "," << arr[i];
      }
    };

    auto write_flags = [&](const std::array<std::uint8_t, LogRecord::ACTIVE_DOF> &arr)
    {
      for (std::size_t i = 0; i < LogRecord::ACTIVE_DOF; ++i)
      {
        oss << "," << static_cast<int>(arr[i]);
      }
    };

    auto write_forces = [&](const std::array<ForceSensorData, FORCE_SENSOR_NUM> &arr)
    {
      for (std::size_t s = 0; s < FORCE_SENSOR_NUM; ++s)
      {
        oss << "," << arr[s].fx
            << "," << arr[s].fy
            << "," << arr[s].fz
            << "," << arr[s].mx
            << "," << arr[s].my
            << "," << arr[s].mz;
      }
    };

    write_doubles(rec.qd);
    write_doubles(rec.qc);
    write_doubles(rec.q);
    write_doubles(rec.dq);
    write_flags(rec.en);
    write_flags(rec.sat);
    oss << "," << static_cast<int>(rec.sat_ft_any);

    write_forces(rec.raw_ft);
    write_forces(rec.ft_bias);
    write_forces(rec.proc_ft);
    write_doubles(rec.tau_int);

    write_doubles(rec.qa);
    write_doubles(rec.dqa);
    write_doubles(rec.tau_grav);
    write_doubles(rec.tau_fric);
    write_doubles(rec.tau_cmd);
    write_doubles(rec.tau_fb);

    oss << "\n";
    file_ << oss.str();
  }

} // namespace upperlimb_robot
