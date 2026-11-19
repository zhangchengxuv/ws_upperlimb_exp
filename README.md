# ws_upperlimb_exp — 上肢康复外骨骼中期基础实验工作空间（ROS 2 Humble）

本工作空间将控制端（`upperlimb_robot`）、URDF 模型（`robot_urdf`）与 C++ Qt5 控制台（`ros_gui`）整合在同一 ROS 2 Humble 工作空间内，专门用于完成以下基础安全功能与三项真机验证实验：

1. **恢复初始位置与安全停止（原样保留）**：
   - `MODE_IDLE = 0`（安全停止 / 空闲，保持位置模式伺服使能）
   - `MODE_LEFT_PRESET_POSITION = 1`（`runLeftPresetPosition`）
   - `MODE_RIGHT_PRESET_POSITION = 2`（`runRightPresetPosition`）
2. **被动正弦轨迹跟踪**：`MODE_PASSIVE = 5`（PID）、`MODE_PASSIVE_SINE_PD_FF = 6`（PD+前馈），支持单关节（`EB1`/`GH1`/`GH3`/`GH2`）与组合联动（`GH1~GH3 + EB1`，`SG1~SG3` 保持起始位）以及两档低速切换（0.5x / 1.0x）。
3. **零力拖动交互响应**：`MODE_ZERO_FORCE = 4`（左臂）、`MODE_RIGHT_ZERO_FORCE = 8`（右臂）、`MODE_BILATERAL_ZERO_FORCE = 9`（双臂，保留首帧重力预补偿平滑交接）。
4. **主动导纳交互响应**：`MODE_ACTIVE = 3`（左臂）、`MODE_RIGHT_ACTIVE = 14`（右臂）、`MODE_BILATERAL_ACTIVE = 13`（双侧），支持三档预设阻尼切换（0.70x / 1.00x / 1.50x）与运行中参数安全互锁。

---

## 一、 关节功能与编号对照（严禁混淆）

- **控制/URDF 逻辑顺序**：左臂 `joint1~joint10`，右臂 `joint11~joint20`
- **底层 EtherCAT 物理轴号**：右臂物理轴 `1~10`（控制索引 `0~9`），左臂物理轴 `11~20`（控制索引 `10~19`）

| 序号 | 论文代号 | 左/右 URDF 关节 | 运动功能 | 状态 |
|---|---|---|---|---|
| 1 | `SG1` | `joint1` / `joint11` | 肩带抬升与下降 | 活动控制（被动正弦保持起始位） |
| 2 | `SG2` | `joint2` / `joint12` | 肩带前后运动 | 活动控制（被动正弦保持起始位） |
| 3 | `SG3` | `joint3` / `joint13` | 肩带姿态调整 | 活动控制（被动正弦保持起始位） |
| 4 | `GH1` | `joint4` / `joint14` | 盂肱关节 外展与内收 | 活动控制 |
| 5 | `GH2` | `joint5` / `joint15` | 盂肱关节 内旋与外旋 | 活动控制 |
| 6 | `GH3` | `joint6` / `joint16` | 盂肱关节 前屈与后伸 | 活动控制 |
| 7 | `EB1` | `joint7` / `joint17` | 肘关节 屈曲与伸展 | 活动控制 |
| 8 | `WR1` | `joint8` / `joint18` | 前臂旋前与旋后 | 当前未启用 |
| 9 | `WR2` | `joint9` / `joint19` | 腕关节尺偏与桡偏 | 当前未启用 |
| 10 | `WR3` | `joint10` / `joint20` | 腕关节屈曲与伸展 | 当前未启用 |

CSV 表头使用 `R_SG1..R_EB1` 与 `L_SG1..L_EB1` 命名，并将原始六维力 `raw_ft_*`、零偏 `ft_bias_*` 与处理后六维力 `proc_ft_*` 严格分开列记录。

---

## 二、 编译步骤（严格避开 Anaconda，使用 ROS 2 Humble）

> **重要**：若本机 `~/.bashrc` 自动激活了 Anaconda，切勿在 Conda 环境下执行 `colcon build` 或运行节点。

```bash
# 1. 退出 Conda 并清除 Conda 对 PATH / PYTHONPATH / LD_LIBRARY_PATH 的影响
conda deactivate 2>/dev/null || true
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONPATH LD_LIBRARY_PATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL

# 2. 加载 ROS 2 Humble 环境
source /opt/ros/humble/setup.bash

# 3. 验证使用系统原生 Python 3.10
which python3  # 应为 /usr/bin/python3

# 4. 编译整个工作空间
cd ws_upperlimb_exp
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

---

## 三、 本机无硬件离线验证命令

完成上述环境清理并 `source install/setup.bash` 后：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. 运行纯软件异步 CSV 日志单测（100 Hz x 10 s，自动校验表头/分离/丢帧并清理临时文件）
ros2 run upperlimb_robot test_async_csv_logger

# 2. 启动离线模拟节点 + Qt5 GUI 联调界面
ros2 launch upperlimb_robot offline_mock_sim.launch.py control_hz:=100
```

---

## 四、 工控机部署与 100 Hz / 50 Hz 启动命令

```bash
# 1. 克隆并编译（先退出 Conda，再 source /opt/ros/humble/setup.bash）
git clone https://github.com/zhangchengxuv/ws_upperlimb_exp.git
cd ws_upperlimb_exp
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 2. 上电前赋予 PCIe 运动控制卡设备权限（如工控机需要）
./start_motor_driver.sh

# 3. 加载工作空间环境
source install/setup.bash

# 4a. 默认 100 Hz 独立启动控制节点（推荐先单开终端看日志）
ros2 launch upperlimb_robot exp_control.launch.py control_hz:=100

# 4b. 如需一键切回 50 Hz（state_hz 默认自动与 control_hz 同步）
ros2 launch upperlimb_robot exp_control.launch.py control_hz:=50

# 5. 另开终端启动 Qt5 控制台
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upperlimb_robot exp_gui.launch.py

# （或一键同时启动控制端 + GUI）
# ros2 launch upperlimb_robot exp_all.launch.py control_hz:=100
```
