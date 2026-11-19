#pragma once

#include "upperlimb_robot/control_algorithms.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace upperlimb_robot
{

  // ===================== 单帧控制日志结构 =====================
  //
  // 全部使用定长数组，避免在实时控制环中发生堆内存分配。
  // 14 个主动关节顺序固定：
  //   0~6  : 右臂 R1~R7 (joint11~joint17，控制索引 0~6)
  //   7~13 : 左臂 L1~L7 (joint1~joint7， 控制索引 10~16)
  //
  struct LogRecord
  {
    static constexpr std::size_t ACTIVE_DOF = 14;

    // 试验信息（定长字符数组，避免 std::string 分配）
    char subject_id[32]{};
    char condition[48]{};
    char trial_id[32]{};
    char event_tag[32]{};

    int mode{0};
    int passive_joint_profile{0};
    int passive_speed_level{0};
    int admittance_damping_level{0};

    // 实时性体检
    std::uint64_t tick{0};
    double monotonic_time_s{0.0};
    double actual_dt_ms{0.0};
    double state_elapsed_ms{0.0};
    double control_elapsed_ms{0.0};
    std::uint64_t dropped_frames{0};

    // 轨迹与状态（deg / deg/s）
    std::array<double, ACTIVE_DOF> qd{};
    std::array<double, ACTIVE_DOF> qc{};
    std::array<double, ACTIVE_DOF> q{};
    std::array<double, ACTIVE_DOF> dq{};
    std::array<std::uint8_t, ACTIVE_DOF> en{};
    std::array<std::uint8_t, ACTIVE_DOF> sat{};
    std::uint8_t sat_ft_any{0};

    // 六维力：原始值、零偏、处理值（顺序：0=RH, 1=RA, 2=LH, 3=LA）
    std::array<ForceSensorData, FORCE_SENSOR_NUM> raw_ft{};
    std::array<ForceSensorData, FORCE_SENSOR_NUM> ft_bias{};
    std::array<ForceSensorData, FORCE_SENSOR_NUM> proc_ft{};

    // 映射交互力矩 J^T F（Nm）
    std::array<double, ACTIVE_DOF> tau_int{};

    // 导纳参考与力矩分解（deg / deg/s / Nm）
    std::array<double, ACTIVE_DOF> qa{};
    std::array<double, ACTIVE_DOF> dqa{};
    std::array<double, ACTIVE_DOF> tau_grav{};
    std::array<double, ACTIVE_DOF> tau_fric{};
    std::array<double, ACTIVE_DOF> tau_cmd{};
    std::array<double, ACTIVE_DOF> tau_fb{};
  };

  // ===================== 辅助填充函数 =====================
  // 将控制层 20 轴缓存与算法诊断量打包为 14 主动关节 LogRecord
  void fillLogRecordFromStates(
      LogRecord &rec,
      const MotorStateCache &motor_state,
      const SensorStateCache &sensor_state,
      const ControlOutputCache &output,
      const AlgorithmDiagData &diag);

  // 将安全字符串写入定长 char 数组（自动替换逗号和换行，防止破坏 CSV 列结构）
  void copySanitizedTag(char *dst, std::size_t dst_size, const std::string &src);

  // ===================== 异步 CSV 日志器 =====================
  //
  // 设计要点：
  // 1. 有界无锁单生产者-单消费者（SPSC）环形队列，构造时预分配全部内存；
  // 2. 控制线程调用 push() 时只做原子下标检查与结构体拷贝，绝不阻塞、绝不写盘；
  // 3. 队列满时丢帧并累加 dropped_frames_，保障实时控制周期稳定；
  // 4. 独立后台线程定期批量格式化并写入磁盘。
  //
  class AsyncCsvLogger
  {
  public:
    explicit AsyncCsvLogger(std::size_t queue_capacity = 4096);
    ~AsyncCsvLogger();

    AsyncCsvLogger(const AsyncCsvLogger &) = delete;
    AsyncCsvLogger &operator=(const AsyncCsvLogger &) = delete;

    // 启动新一轮记录会话：在 output_dir 下创建带时间戳的 CSV 文件并写入表头
    bool startSession(
        const std::string &output_dir,
        const std::string &subject_id,
        const std::string &condition,
        const std::string &trial_id);

    // 停止当前记录会话：等待队列中剩余帧全部落盘后再关闭文件
    void stopSession();

    // 实时控制线程调用：非阻塞入队；未开启记录或队列满时立即返回 false
    bool push(const LogRecord &record);

    bool isActive() const;
    std::uint64_t droppedFrames() const;
    std::uint64_t writtenFrames() const;
    std::string currentFilePath() const;

    // 返回 CSV 表头（供单测核查）
    static std::string buildCsvHeader();

  private:
    void writerLoop();
    void drainQueueToStream();
    void writeRecordLine(const LogRecord &rec);

    std::size_t capacity_{4096};
    std::vector<LogRecord> ring_;

    // SPSC 环形队列读写指针
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};

    std::atomic<bool> logging_active_{false};
    std::atomic<bool> worker_stop_{false};
    std::atomic<std::uint64_t> dropped_frames_{0};
    std::atomic<std::uint64_t> written_frames_{0};

    mutable std::mutex session_mutex_;
    std::ofstream file_;
    std::string current_file_path_;
    std::thread writer_thread_;
  };

} // namespace upperlimb_robot
