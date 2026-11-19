# ExperimentConfig.msg
# 实验参数配置（仅在 MODE_IDLE 空闲状态下生效，运行中修改会被控制端安全互锁拒绝）

# 被动正弦关节选择（SG1~SG3 保持起始位不动）：
# 0 = 组合联动 GH1~GH3 + EB1 (joint4~joint7)
# 1 = 单关节 GH1（肩外展/内收，joint4）
# 2 = 单关节 GH2（肩内旋/外旋，joint5）
# 3 = 单关节 GH3（肩前屈/后伸，joint6）
# 4 = 单关节 EB1（肘屈曲/伸展，joint7）
int32 passive_joint_profile

# 被动正弦两档低速选择：
# 0 = 低速1档（慢速，0.5x 原频）
# 1 = 低速2档（标准低速，1.0x 原频）
int32 passive_speed_level

# 主动导纳阻尼三档预设：
# 0 = 低阻尼（0.70x）
# 1 = 标准阻尼（1.00x）
# 2 = 高阻尼（1.50x）
int32 admittance_damping_level
