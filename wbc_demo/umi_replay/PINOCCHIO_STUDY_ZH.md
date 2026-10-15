# Pinocchio 能否用于当前 G2？实测与选型

## 结论

**可以作为当前机器人运动学/动力学计算底座；本轮已做模型级接入和离线对照，但未把在线生产后端切换成 Pinocchio。**
Pinocchio 不是“一句调用就自动完成双手全身 IK 与 WBC”的完整控制器。
它提供 FK、Jacobian、积分以及刚体动力学等基础算法；任务优化与控制契约仍需由上层实现。[Pinocchio 官方项目](https://github.com/stack-of-tasks/pinocchio)

## 1. 为什么值得接入

当前结构是：自写 Python URDF 几何 → 自写任务/限值 → OSQP → MoveIt 状态检查 → PhysX 位置驱动。
可先仅替换“几何/动力学计算”层，保留双手共享腰部、夹爪 EEF、机构耦合和碰撞保护。
`PinocchioKinematics` 已提供与现有模型相同的 `evaluate()` 接口，单步 QP 对照测试通过；它目前只被测试/离线基准调用。

## 2. 本轮如何测试，避免“装上就算完成”

- 没有改动正在运行的 `xense-wbc`：在忽略的 logs 目录创建了继承其基础包的隔离 venv，安装 **Pinocchio 4.1.0**。
- 用本仓库同一份 G2 Swiftpicker URDF；核对该 URDF 与真实回放报告中的 SHA256 一致。
- 从通过的 v5 实际 `joint_trace.csv` 取 100 个状态，包含就位、回放、回零与实测夹爪跟随偏差。
- 给两个实现施加相同的、非单位旋转和平移的测试基座变换，避免只在坐标轴恰巧对齐时通过。
- 对比两只夹爪真实指尖 EEF 的 FK、21 维命令 Jacobian；额外检查投影质量矩阵的正定性。
- 没有发机器人命令，没有把离线计算基准伪装成实测闭环控制。

| 指标 | 本轮结果 |
|---|---:|
| Pinocchio / Python | 4.1.0 / 3.12.15 |
| 原始模型 `nq / nv`，命令自由度 | 46 / 42，21 |
| EEF 最大位置差 | `1.913e-15 m` |
| EEF 最大旋转差 | `1.302e-15 rad` |
| Jacobian 最大元素差 | `2.410e-8`，阈值 `2e-6` |
| 原 Python FK/Jacobian 中位 / P95 | 1.149 / 2.614 ms |
| Pinocchio FK/Jacobian 中位 / P95 | 0.092 / 0.230 ms |
| 投影质量矩阵最小特征值 | `6.090e-4`，大于零 |

**这里只是 FK/Jacobian 计算段中位耗时约下降 12.4 倍，不是整个控制链提速 12.4 倍。**
同步碰撞服务、QP、Python/ROS 调度与渲染延迟还在；质量矩阵正定也不等于 URDF 惯量已与 PhysX/实机标定一致。
原始指标与图：[模型对照验收页](media/pinocchio-study-v1/index.html)。基准包含冷启动样本，不保证其他机器有相同耗时。

## 3. 必须保留的 G2 特殊处理

### 不把法兰当 EEF

当前 EEF 是两指垫末端面中心的中点，会随开度变化。新实现给两个实际指尖点分别建立 Pinocchio frame，
取两点位置/Jacobian 的平均；EEF 朝向按现有相机侧轴定义，从法兰姿态转换。
所以不是在法兰后面加一个永不变化的静态偏移。

### 不把所有关节都当控制自由度

模型 `nq != nv`；轮子的圆周配置、固定帧和实际受控变量不能按数组序号硬拼。
本实现按关节名映射，把全部模型速度通过矩阵映射到 19 个腰/臂变量加 2 个夹爪主关节。
底盘、轮子、头部不因此解锁；原有腰部/腕部线性耦合约束仍由上层 QP 负责。

### mimic 不能只“忽略”或“全部强制归理想值”

4.1.0 的加载接口支持 mimic 模型，本轮还核查过 `mimic=True` 的模型尺寸。
但 PhysX 反馈中的从动关节可能存在微小偏差，不能把这些偏差从实测 FK 中抹掉。
因此探测适配器明确使用 `mimic=False`：保留从动关节实测位置，再按 URDF 倍率把速度折叠到主关节。
模型自由度较大是有意设计，不是漏处理 mimic；理想命令与真实测量都能采用同一 EEF 定义。

### 明确 Jacobian 参考系

使用 EEF 点原点处、轴与模型世界系对齐的 `LOCAL_WORLD_ALIGNED` Jacobian，再转到约定的 `odom`。
不能把 `LOCAL`、`WORLD` 或以其他原点表达的空间速度直接混用；位置与角速度的排列也必须核对。
本轮非单位基座变换与数值差分测试就是为此设置。

## 4. 优秀项目如何分工

| 项目 | 本项目应借鉴/使用什么 | 不应误认为 |
|---|---|---|
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | 统一 FK/Jacobian/刚体动力学模型，替换手写热路径 | 完整 IK、避障器或控制器 |
| [Pink](https://github.com/pink-kinematics/pink) | 基于 Pinocchio 的局部加权微分 IK、任务与限值组织；下一步 IK 框架候选 | 全局规划、自动严格优先级、力矩 WBC |
| [PlaCo](https://github.com/Rhoban/placo) | Pinocchio 上的 C++/Python 运动学及逆动力学高层 API，可作为完整框架对照 | 不经适配即可支持 G2 特有约束 |
| [TSID](https://github.com/stack-of-tasks/TSID) | 基于 Pinocchio 的优化逆动力学，适合后续加速度/力矩/接触力任务 | 仅换库就能在当前位置接口实现真实力控 |
| [Mink](https://github.com/kevinzakka/mink) | MuJoCo 微分 IK 的几何碰撞、限制和测试组织 | 直接替换 PhysX 的无成本方案 |
| [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) | 碰撞/奇异性管理、停止状态与流式命令接口 | 两个单臂 Servo 自动组成共享腰部双手联合优化 |

**建议路径（本项目的工程判断）**：先 Pinocchio 统一模型，保留当前 QP 做公平 A/B；再小范围评估 Pink 的任务层适配。
需要逆动力学时评估 TSID / PlaCo，不同时堆进三套控制框架。

## 5. 复现探测，不影响现有服务

```bash
mamba activate xense-wbc
python -m venv --system-site-packages wbc_demo/logs/wbc-next-stage/pinocchio-venv
wbc_demo/logs/wbc-next-stage/pinocchio-venv/bin/python -m pip install \
  -r wbc_demo/umi_replay/pinocchio_requirements.txt
OPENBLAS_NUM_THREADS=1 wbc_demo/logs/wbc-next-stage/pinocchio-venv/bin/python wbc_demo/umi_demo.py benchmark-kinematics \
  --reference-run wbc_demo/logs/umi-realtime-web-reliable-01/run-001 \
  --samples 100 --output wbc_demo/logs/my-pinocchio-study
```

PyPI 正确包名是 `pin`，Python 导入名才是 `pinocchio`；另一条官方安装路径是 conda-forge 的 `pinocchio`。
具体生产环境依赖方案需要单独验证 C++/Boost/NumPy/ROS ABI，本次没有向基础环境强行安装这些二进制依赖。[官方安装说明](https://github.com/stack-of-tasks/pinocchio)

换用别的真实运行目录时，其报告必须为通过状态，并有原始 `joint_trace.csv`；输出目录必须不存在，以免覆盖证据。
