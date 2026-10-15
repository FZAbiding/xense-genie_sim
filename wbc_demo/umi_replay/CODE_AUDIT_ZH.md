# 实时 IK 下一阶段代码审计

本轮基线为 `59005e2`。审计范围是 `umi_replay` 的输入、Jacobian、控制截止时间、预检声明与执行后审计；不是对整个仿真仓库的安全认证。
每项“已修复”都对应复现或回归测试；未验证的控制能力单独列出，不与 bug 混写。

## 1. 已复现并修复

| 级别 | 问题与影响 | 修复与证据 |
|---|---|---|
| P1 | 只要 `deadline_misses > 0`，审计就跳过位置、速度、加速度、耦合的全部越界判断，非法命令仍可能标为通过 | 位置/速度/机构耦合始终检查；缺少 hold 记录时仅把加速度审计标为不完整，不能悄悄当成完整通过 |
| P1 | CSV 的 NaN 经过 `max` 和比较后可绕过审计，形成 `status=passed` 与非有限指标共存 | 命令、跟踪记录与限值元数据先做有限性检查，并校验命令条数 |
| P1 | `min(native_velocity, NaN/Inf)` 可能掩盖非法用户速度配置；配置原来在首帧就位后才读取 | 原始配置在任何 ROS 初始化/运动前检查；校验完的配置快照传给执行器，不在就位后重新读取文件 |
| P1 | 夹爪 Jacobian 的扰动点清空了实测 mimic 偏差，而基准点保留偏差，有限差分会把静态误差除以 `1e-6` 放大 | 扰动时按 URDF mimic 比例增量更新，保留原有实测偏差。加入 0.002 rad 指垫关节偏差的测试：修复前出现约 61.26 m/rad 的错误项，修复后通过有限差分对照 |
| P2 | 在线模式不执行运动时只检查首帧，却输出容易理解为“完整预检通过”的状态 | 单独输出 `initial_pose_preflight_passed_no_execution`，记录 `preflight_scope=initial_pose_only` |
| P2 | 碰撞检查预算已经到期，仍可能提交服务请求，增加无效排队 | 在每次 `call_async` 之前检查截止时间，到期不发请求；取消已超时的 future |

这里的 P1 表示对本实验控制/验收可信度影响较高，不代表已发生硬件事故。特别是审计 fail-open 是**事后验证问题**，不能据此倒推 v5 的已记录正常轨迹必然非法。

### 复现与测试

- `tests/test_realtime_analysis.py`：超时不能关闭速度审计；NaN 命令不得通过。
- `tests/test_differential_ik.py`：非法配置、实际 mimic 偏差下的夹爪 Jacobian。
- `tests/test_execution_math.py`：坏配置必须在 `rclpy.init` 之前拒绝。
- `tests/test_realtime_execution.py`：无效/超时碰撞返回拒绝；截止时间到期不发新请求。
- `tests/test_pinocchio_backend.py`：Pinocchio 和原有模型在实际 mimic 偏差下的 FK/Jacobian、一致 QP 结果与质量矩阵检查。

失败前日志：`wbc_demo/logs/wbc-next-stage/bugs-before-fix.log`、`mimic-bug-before-fix.log`；修复后日志在同目录，保留原失败，不覆盖成成功。

## 2. 架构限制，不是“换库即可修复”的 bug

1. **不是实测状态驱动的笛卡尔闭环伺服**：当前用上一条命令做 IK 热启动，实测状态用于误差监控和停止。不能声称只靠 Pinocchio 就获得抗扰控制。
2. **一帧前瞻依赖已知文件**：不能拿已知 UMI 后一帧的结果冒充因果在线 VLA 输入。真实流接口需要时间戳、超时和无前瞻实验。
3. **固定 1/30 s 积分不是操作系统硬实时**：规划离散速度/加速度边界不等于实际电机加速度认证。超时/暂停后的速度状态连续性还需专项实验。
4. **碰撞只做离散拒绝，不主动绕障**：中点/端点有效不等于连续扫掠安全；同步服务调用也不适合直接搬到高频力矩环。
5. **没有显式奇异性管理与制动可行域保证**：阻尼能缓解数值问题，不等于奇异位形下仍可满足任务；临近限位时速度/加速度约束仍可能不可行并停止。
6. **只验证了 episode 000 的真实运行**：不能以一段视频宣称 11 个 episode、所有 workspace 都已通过。
7. **资源清理和报告仍需加强**：节点构造早期异常、回放中断后的耗时汇总等路径还没有完整故障注入覆盖。

这些问题进入 [WBC 路线与验收门槛](WBC_NEXT_STAGE_ZH.md)，本轮不通过扩大容差、关闭碰撞或悄悄切换生产后端来掩盖。

## 3. 借鉴开源工程的原则

- 从 [MoveIt Servo 官方文档](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) 借鉴明确的碰撞、奇异性、关节限位与停止状态设计，不只是计算一组关节角。
- 从 [Pink](https://github.com/pink-kinematics/pink) 借鉴任务、限值和配置积分的分层组织。它本身是局部、加权微分 IK，不能误当全局规划或严格任务层级。
- 从 [Mink](https://github.com/kevinzakka/mink) 借鉴任务/限制与几何碰撞约束的测试方式，但它基于 MuJoCo，不为借鉴 IK 而强行替换当前 PhysX。

上述是设计借鉴，不是把对应项目的安全保证直接继承到本仓库。
