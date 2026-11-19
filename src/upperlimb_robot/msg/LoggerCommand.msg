# LoggerCommand.msg
# 控制异步 CSV 日志器的启停与事件打标

# 动作类型：
# 0 = 仅更新试验元信息
# 1 = 开始记录（自动生成带时间戳的新 CSV 文件）
# 2 = 停止记录（刷盘并关闭当前 CSV 文件）
# 3 = 事件打标（记录一次性事件标签，如 START / HOLD / RELEASE / END）
int32 action

string subject_id
string condition
string trial_id
string event_tag
