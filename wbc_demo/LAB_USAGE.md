# G2 固定底盘全身 IK 实验台

## 1. 从哪里启动？

只在 `xense-genie_sim` 仓库根目录运行。关闭之前启动的本 demo 后端，避免重复节点。

```bash
mamba activate xense-wbc
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh lab
```

会打开 RViz 和中文控制面板。Isaac/PhysX 在后台真实计算，不打开 Isaac 原生窗口。
关闭控制面板后，启动脚本会清理自己创建的后端。底盘始终固定到世界。

若已经单独启动 `bash wbc_demo/run.sh sim` 与 `bash wbc_demo/run.sh moveit`，
只运行 `bash wbc_demo/run.sh interactive`，不要再运行 `lab` 重复启动后端。

默认 `ROS_DOMAIN_ID=42`，各终端如需覆盖，必须使用相同值。

## 2. 怎样交互？

1. 在控制面板选择“腰部解锁”或“腰部锁定”。
2. 输入左右手的绝对位置 XYZ（米）和 Roll/Pitch/Yaw（度），坐标系为 `odom`。
3. 也可以在 RViz 拖动蓝色目标；勾选 Displays → Lab Target Handles。
4. 或从下拉框载入 `deep_reach`、`orbit_left` 等预设。载入/拖动都**不会自动执行**。
5. 点“1. 求解并检查路径”，确认检查通过，再点“2. 执行已检查路径”。
6. 看两手误差曲线、5 个腰部关节角度曲线，以及 RViz 中的 TCP 实际轨迹。

修改目标或模式会使旧预览失效；机器人移动后，过期预览也不能直接执行。
执行时拖动目标只改变下一次请求，不会把当前运动突然改成新目标。

“停止 / 保持”是仿真软件停止，不是硬件安全急停。
执行循环收到停止请求后，重复发送测量位置作为保持目标；阻塞的 IK/规划服务调用需要有界返回后才能响应。
“回初始关节姿态”会包含腰部，即使面板当前选择锁腰；按钮文字已明确说明。

### 锁腰到底锁了什么？

- 解锁：`wbc_fixed_headless`，5 个 body + 14 个双臂关节参与 IK。
- 锁定：`wbc_fixed_arms`，只有 14 个双臂关节参与 IK。
- 锁定时腰部仍持续收到不变的位置保持目标，而不是从物理模型焊死这些关节。
- 因为位置驱动有跟踪误差，实际腰部允许微小漂移；执行后检查最大漂移不超过 0.015 rad。

## 3. 怎样对照“需要腰”与“不需要腰”？

在相同双手目标下点“同目标锁腰 / 解锁对照”。两种组使用**同一个机器人状态快照和目标**，
只求解，不执行；结果显示 `solved` / `rejected` 以及耗时和误差。

仅看到锁腰超时，不能数学证明目标不可达。这里还提供“寻找有几何证明的腰部优势案例”：

- 按逐步降低双手的目标扫描。
- 计算锁腰时 `arm_base_link` 到目标的距离。
- 用 URDF 从该根部到末端所有固定平移长度之和作为保守臂长上界。
- 如果目标距离仍大于这个上界，则不论手臂如何转动都够不到。
- 同时要求解锁腰部的 IK 成功，才将该目标标为有证明的对照案例。

找到后只载入目标，不自动运动。先回初始姿态再寻找，通常更容易得到直观案例。

自动寻找并执行该案例，也可用：

```bash
bash wbc_demo/run.sh all --compare --execute-found
```

如果目标未超过保守臂长上界，即使求解失败，也只记为预算内未找到可用解，不宣称几何不可达。

## 4. 大范围自动压力测试

```bash
bash wbc_demo/run.sh all --cycles 2
```

每轮 9 类双手联合目标，结束每个挑战后回到初始关节姿态：

| 目标 | 相对运行初始位姿的变化 |
|---|---|
| `long_reach` | 双手前伸 28 cm、下降 20 cm |
| `deep_reach` | 双手前伸 8 cm、下降 55 cm |
| `sweep_left/right` | 双手横向 ±25 cm，同时略前伸/下降 |
| `spread_hands` | 左右手向各自外侧展开 18 cm |
| `asymmetric` | 左手升高 18 cm，右手降低 30 cm，前伸量也不同 |
| `orbit_left/right` | 绕固定底盘的 XY 锚点旋转 ±30°，同时旋转手的目标朝向 |
| `wrist_rotation` | 左右工具局部 Roll 分别 +25° / -25° |

目标都以开始时的双手位姿为参考，不是反复累加位移。
配置文件是 `stress_targets.yaml`。失败会记录并停止，不悄悄缩小运动幅度冒充通过。
此外测试 +5 米的不可达目标，要求拒绝且不发送执行指令。

先只求解、不运动，可以连接已启动的后端运行：

```bash
bash wbc_demo/run.sh stress --probe-only
bash wbc_demo/run.sh compare
```

默认改用 blank 场景，以免把未进入 MoveIt 碰撞世界的 GDK 货架当成已实现避障。
经典四点小范围回归仍保留为 `bash wbc_demo/run.sh demo --cycles 3`，需要后端已启动。

## 5. 执行保护与仍未解决的问题

- IK 使用原生 URDF 限位：自定义启动传入 `pad_joint_limits:=false`，不利用原来的 0.01 rad 额外范围求解。
- 目标解检查原生关节范围及仓库提供的机械耦合不等式。
- 关节路径按最多约 0.025 rad 的采样间距检查有效性，每段至少 21 个样本。
- 直线关节插值若无效，会请求已有 OMPL RRT-Connect，再检查返回路径；规划失败则拒绝。
- 默认最大关节目标速度 0.3 rad/s、轨迹加速度参数 0.5 rad/s²；同时考虑原生速度限值，自动延长运动时间。
- 调度延迟不会通过突然追赶墙钟时间制造大步跳变；单次最大关节命令步长额外限制为 0.02 rad。
- 监测反馈是否停止、底盘漂移、执行后 TCP 误差；每个实验保存逐目标 JSON 与完整关节 CSV。
- 控制端使用独占租约，并拒绝多个 `/joint_command` 发布者，避免互相争抢控制。

这些是仿真保护，不是连续碰撞证明或真机安全认证：

- 起点和采样状态允许小的数值/跟踪漂移；严格真实机械可行性仍依赖平台模型与标定。
- 速度/加速度重定时约束的是目标轨迹，不保证实际电机的每个采样都达到相同边界。
- 没有场景避障物同步、动力学接触优化、力矩级 WBC 或真机急停链。
- 底盘焊接到世界，因此测试不能证明真实轮式平台持重时不会倾覆。
- 多解可能造成很大的躯干动作；通过目标不代表动作自然、姿态代价最优或适合真机。

## 6. 日志、录像与测试

每次实验有独立目录，避免覆盖上一次结果：

- `logs/stress-日期时间/report.json`、`joint_trace.csv`。
- `logs/compare-日期时间/report.json`、`joint_trace.csv`。
- `logs/lab-日期时间/report.json`、`joint_trace.csv`。
- 后端原始日志为 `logs/sim.log`、`logs/moveit.log`。

自定义 `--output-name` 可以给压力测试/对照实验目录命名；同名目录已存在时拒绝覆盖。

录制仿真渲染图像，不采集桌面：

```bash
python wbc_demo/record_camera.py --seconds 240 --output-name large-range
```

测试纯逻辑与 GUI/真实仿真链路：

```bash
source devel/setup.bash
python -m unittest discover -s wbc_demo -p test_lab.py -v
bash wbc_demo/run.sh lab --self-test
```

GUI 自测实际发送 interactive-marker feedback，检查预览、执行、软件停止、
重新执行、锁腰执行和同目标对照。它会移动仿真机器人，不是真机测试。
完成后自动关闭；测试中不要同时打开另一控制端。

当前已验证结果和发现的问题见 [EXPERIMENT_RESULTS.md](EXPERIMENT_RESULTS.md)。
