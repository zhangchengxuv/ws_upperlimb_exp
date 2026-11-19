# SystemState.msg
# 定义机器人系统状态
bool joint_init
bool controller_init
# 初始化完成后才允许控制算法开始运行
bool system_ready

# 控制端当前已生效的模式编号（与 ControlMode 枚举一致）
int32 applied_mode

# 异步 CSV 日志器当前状态
bool logging_active
string current_log_file
uint64 dropped_frames
