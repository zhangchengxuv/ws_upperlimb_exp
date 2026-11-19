#!/usr/bin/python3
"""
离线模拟控制节点（无 EtherCAT 硬件时与 ros_gui_node 联调测试用）。
严格模拟控制端话题接口：
- 发布: system_state, robot_state
- 订阅: control_command, experiment_config, logger_command
"""

import math
import time

import rclpy
from rclpy.node import Node

from upperlimb_robot.msg import (
    ControlCommand,
    ExperimentConfig,
    LoggerCommand,
    RobotState,
    SystemState,
)


MODE_IDLE = 0
MODE_LEFT_PRESET_POSITION = 1
MODE_RIGHT_PRESET_POSITION = 2
MODE_ACTIVE = 3
MODE_ZERO_FORCE = 4
MODE_PASSIVE = 5
MODE_PASSIVE_SINE_PD_FF = 6
MODE_RIGHT_ZERO_FORCE = 8
MODE_BILATERAL_ZERO_FORCE = 9
MODE_BILATERAL_ACTIVE = 13
MODE_RIGHT_ACTIVE = 14


class MockRobotNode(Node):
    def __init__(self):
        super().__init__('mock_robot_node')

        self.control_hz = int(self.declare_parameter('control_hz', 100).value)
        self.publish_hz = int(self.declare_parameter('publish_hz', 20).value)

        self.applied_mode = MODE_IDLE
        self.mode_enter_time = time.monotonic()

        # 实验参数（受 MODE_IDLE 安全互锁保护）
        self.passive_joint_profile = 0
        self.passive_speed_level = 1
        self.admittance_damping_level = 1

        # 模拟日志状态
        self.logging_active = False
        self.current_log_file = ''
        self.subject_id = 'S01'
        self.condition = 'baseline'
        self.trial_id = 'T01'
        self.last_event_tag = ''
        self.dropped_frames = 0

        # 20 轴状态（右臂 0~6 活动、7~9 未用；左臂 10~16 活动、17~19 未用）
        self.position = [0.0] * 20
        self.velocity = [0.0] * 20
        self.torque = [0.0] * 20
        self.enabled = [False] * 20
        for i in list(range(0, 7)) + list(range(10, 17)):
            self.enabled[i] = True
        self.position[6] = 45.0   # R_EB1
        self.position[16] = 45.0  # L_EB1
        self.passive_q0 = list(self.position)

        self.force_sensor = [0.0] * 24

        self.cmd_sub = self.create_subscription(
            ControlCommand, 'control_command', self.on_control_command, 10
        )
        self.cfg_sub = self.create_subscription(
            ExperimentConfig, 'experiment_config', self.on_experiment_config, 10
        )
        self.log_sub = self.create_subscription(
            LoggerCommand, 'logger_command', self.on_logger_command, 10
        )

        self.sys_pub = self.create_publisher(SystemState, 'system_state', 10)
        self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)

        self.last_step_t = time.monotonic()
        self.sys_timer = self.create_timer(0.5, self.publish_system_state)
        self.state_timer = self.create_timer(1.0 / max(1, self.publish_hz), self.step_and_publish)

        self.get_logger().info(
            f'离线模拟控制节点启动 (模拟控制频率={self.control_hz} Hz, 发布={self.publish_hz} Hz)'
        )

    def on_control_command(self, msg: ControlCommand):
        new_mode = int(msg.command)
        valid_modes = {
            MODE_IDLE,
            MODE_LEFT_PRESET_POSITION,
            MODE_RIGHT_PRESET_POSITION,
            MODE_ACTIVE,
            MODE_ZERO_FORCE,
            MODE_PASSIVE,
            MODE_PASSIVE_SINE_PD_FF,
            MODE_RIGHT_ZERO_FORCE,
            MODE_BILATERAL_ZERO_FORCE,
            MODE_BILATERAL_ACTIVE,
            MODE_RIGHT_ACTIVE,
        }
        if new_mode not in valid_modes:
            self.get_logger().warn(f'收到不支持的模式编号: {new_mode}，保持当前模式 {self.applied_mode}')
            return

        old_mode = self.applied_mode
        self.applied_mode = new_mode
        self.mode_enter_time = time.monotonic()
        if new_mode in (MODE_PASSIVE, MODE_PASSIVE_SINE_PD_FF):
            self.passive_q0 = list(self.position)
        self.get_logger().info(f'[Mock] 模式切换: {old_mode} -> {new_mode}')
        self.publish_system_state()

    def on_experiment_config(self, msg: ExperimentConfig):
        if self.applied_mode != MODE_IDLE:
            self.get_logger().warn(
                f'[Mock] 安全互锁：当前运行模式({self.applied_mode})禁止修改实验参数！'
            )
            return
        self.passive_joint_profile = int(msg.passive_joint_profile)
        self.passive_speed_level = int(msg.passive_speed_level)
        self.admittance_damping_level = int(msg.admittance_damping_level)
        self.get_logger().info(
            f'[Mock] 更新实验参数: joint_profile={self.passive_joint_profile}, '
            f'speed={self.passive_speed_level}, damping={self.admittance_damping_level}'
        )

    def on_logger_command(self, msg: LoggerCommand):
        if msg.subject_id:
            self.subject_id = msg.subject_id
        if msg.condition:
            self.condition = msg.condition
        if msg.trial_id:
            self.trial_id = msg.trial_id

        if msg.action == 1:
            self.logging_active = True
            ts = time.strftime('%Y%m%d_%H%M%S')
            self.current_log_file = (
                f'data/{self.subject_id}_{self.condition}_{self.trial_id}_{ts}_mock.csv'
            )
            self.get_logger().info(f'[Mock] 开始记录: {self.current_log_file}')
        elif msg.action == 2:
            self.logging_active = False
            self.get_logger().info('[Mock] 停止记录')
        elif msg.action == 3:
            self.last_event_tag = msg.event_tag
            self.get_logger().info(f'[Mock] 收到事件打标: {self.last_event_tag}')
        self.publish_system_state()

    def publish_system_state(self):
        msg = SystemState()
        msg.controller_init = True
        msg.joint_init = True
        msg.system_ready = True
        msg.applied_mode = self.applied_mode
        msg.logging_active = self.logging_active
        msg.current_log_file = self.current_log_file
        msg.dropped_frames = self.dropped_frames
        self.sys_pub.publish(msg)

    def step_and_publish(self):
        now = time.monotonic()
        dt = max(0.005, min(0.1, now - self.last_step_t))
        self.last_step_t = now
        t_mode = now - self.mode_enter_time

        prev_pos = list(self.position)

        if self.applied_mode == MODE_LEFT_PRESET_POSITION:
            targets = {10: 0.0, 11: 0.0, 12: 0.0, 13: 0.0, 14: 0.0, 15: 0.0, 16: 45.0}
            for idx, tgt in targets.items():
                self.position[idx] += (tgt - self.position[idx]) * min(1.0, 2.5 * dt)
        elif self.applied_mode == MODE_RIGHT_PRESET_POSITION:
            targets = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 45.0}
            for idx, tgt in targets.items():
                self.position[idx] += (tgt - self.position[idx]) * min(1.0, 2.5 * dt)
        elif self.applied_mode in (MODE_PASSIVE, MODE_PASSIVE_SINE_PD_FF):
            u = min(1.0, t_mode / 2.0)
            ramp = 0.5 * (1.0 - math.cos(math.pi * u))
            speed_scale = 0.5 if self.passive_speed_level == 0 else 1.0
            amps = [8.0, 12.0, 10.0, 15.0]
            freqs = [0.10, 0.10, 0.10, 0.12]
            phases = [0.0, math.pi / 2.0, 0.0, 0.0]
            if self.passive_joint_profile == 1:
                amps = [8.0, 0.0, 0.0, 0.0]
            elif self.passive_joint_profile == 2:
                amps = [0.0, 12.0, 0.0, 0.0]
            elif self.passive_joint_profile == 3:
                amps = [0.0, 0.0, 10.0, 0.0]
            elif self.passive_joint_profile == 4:
                amps = [0.0, 0.0, 0.0, 15.0]
            for k in range(4):
                idx = 13 + k
                w = 2.0 * math.pi * freqs[k] * speed_scale
                self.position[idx] = self.passive_q0[idx] + ramp * amps[k] * math.sin(w * t_mode + phases[k])
        elif self.applied_mode in (
            MODE_ZERO_FORCE,
            MODE_RIGHT_ZERO_FORCE,
            MODE_BILATERAL_ZERO_FORCE,
            MODE_ACTIVE,
            MODE_RIGHT_ACTIVE,
            MODE_BILATERAL_ACTIVE,
        ):
            # 模拟轻微拖动响应与传感器力信号
            damping_scale = [0.7, 1.0, 1.5][max(0, min(2, self.admittance_damping_level))]
            wave = math.sin(2.0 * math.pi * 0.2 * t_mode)
            if self.applied_mode in (MODE_ZERO_FORCE, MODE_BILATERAL_ZERO_FORCE, MODE_ACTIVE, MODE_BILATERAL_ACTIVE):
                self.position[16] = 45.0 + (8.0 / damping_scale) * wave
                self.torque[16] = 1.8 + 0.6 * wave
            if self.applied_mode in (MODE_RIGHT_ZERO_FORCE, MODE_BILATERAL_ZERO_FORCE, MODE_RIGHT_ACTIVE, MODE_BILATERAL_ACTIVE):
                self.position[6] = 45.0 + (8.0 / damping_scale) * wave
                self.torque[6] = -1.8 - 0.6 * wave

        for i in range(20):
            self.velocity[i] = (self.position[i] - prev_pos[i]) / dt
            if self.applied_mode == MODE_IDLE:
                self.torque[i] = 0.0

        # 模拟 4 处六维力微幅变化
        s = math.sin(now * 1.5)
        self.force_sensor = [
            0.3 * s, -0.2 * s, 0.5 * s, 0.02 * s, -0.01 * s, 0.01 * s,
            1.2 * s, -0.6 * s, 2.0 * s, 0.08 * s, -0.04 * s, 0.05 * s,
            0.4 * s, -0.3 * s, 0.6 * s, 0.02 * s, -0.02 * s, 0.01 * s,
            1.5 * s, -0.8 * s, 2.4 * s, 0.09 * s, -0.05 * s, 0.06 * s,
        ]

        msg = RobotState()
        msg.position = list(self.position)
        msg.velocity = list(self.velocity)
        msg.torque = list(self.torque)
        msg.enabled = list(self.enabled)
        msg.force_sensor = list(self.force_sensor)
        msg.actual_dt_ms = 1000.0 / max(1, self.control_hz) + 0.08 * math.sin(now * 3.0)
        msg.state_elapsed_ms = 1.65 + 0.12 * math.cos(now * 2.0)
        msg.control_elapsed_ms = 0.85 + 0.09 * math.sin(now * 2.5)
        msg.applied_mode = self.applied_mode
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
