# G2 固定底盘：双臂 + 腰部全身 IK

**唯一总结入口：[根目录 PROJECT_GUIDE_ZH.md](../PROJECT_GUIDE_ZH.md)**。
坐标、连杆 / IK 教程、Flexiv / ARX5 对照、实测图视频、结果和待办已统一到根目录；下文保留原有操作和历史记录，不作为并列的当前总览。

早期复现入门见 [给 VLA 工程师的全身 IK 入门](WHOLE_BODY_IK_GUIDE_ZH.md)：
解释 FK/IK/WBC、运行命令、仓库已有能力与本次新增代码、worktree 与私人仓库同步，以及下一阶段实验。

**来源说明：本 demo 是基于仓库已有 G2 仿真与 MoveIt/BioIK 能力新增的测试流程，
不是原封不动的官方 demo，也没有重写 IK 核心算法。**

## 升级版：大范围测试与交互实验台

新增 **[XTac UMI 数据回放](umi_replay/README.md)**：完整数据下载、坐标定义、转换脚本、
固定底盘双臂/腰部预检和回放；先看 [坐标说明](umi_replay/COORDINATES_ZH.md)，
核查 [执行记录](umi_replay/WORK_LOG.md)。这是本地新增的数据适配，不是官方原封不动的回放 demo。

UMI 新增 **[Web 四视角在线回放](umi_replay/ONLINE_IK_AND_VIEWS_ZH.md)**：原始头显视频、同向虚拟第一视角、
两个可拖动外部视角；默认软直腰偏好，可选 ±0.5° 对照策略，无需 PyQt/PySide。

当前目录是 `xense-genie_sim`，迁移版本 `d975634` 已推送到私人仓库 `main`。
升级版操作说明见 **[LAB_USAGE.md](LAB_USAGE.md)**，测试结果见 [EXPERIMENT_RESULTS.md](EXPERIMENT_RESULTS.md)。

```bash
mamba activate xense-wbc
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh lab
```

此命令启动 PhysX、MoveIt、RViz 和中文交互实验台。拖动 RViz 蓝色目标或输入双手位姿，
先“求解并检查路径”，再“执行”；可以切换锁腰/解锁，做同目标对照并观察误差和腰部曲线。
关闭实验台会清理本次启动的后端。不要与另一套仿真/MoveIt 重复启动。

大范围自动测试：

```bash
bash wbc_demo/run.sh all --cycles 2
```

每轮 9 类目标，包括前伸 28 cm、下降 55 cm、左右横移 25 cm、双手展开、
不对称上下伸手、绕底盘 ±30° 扭转、双腕 ±25° 转动。每类目标执行后回到初始关节姿态。
两轮是 18 个挑战目标 + 18 次回家，不再是初版的 8/16 cm 四点序列。
默认使用没有货架的 blank 场景；原 GDK 场景可通过 `WBC_SCENE=scene_fixed.yaml` 选择，
但货架尚未接入 MoveIt 碰撞世界，不推荐拿它直接进行大范围避障测试。

自动寻找并执行锁腰不可达、解锁可达的几何证明案例：

```bash
bash wbc_demo/run.sh all --compare --execute-found
```

默认 ROS domain 为 42。`stress`、`compare`、`interactive` 子命令只启动控制端，
用于连接已经启动的 `sim` + `moveit`；不要与另一控制端并行发送命令。

## 初版验证记录

- 环境：`xense-wbc`，Python 3.12、RoboStack ROS 2 Jazzy、MoveIt 2.12.4。
- 已安装 Isaac Sim 6.0.0.1、OVRTX 0.3.0.312915、MuJoCo 3.8.0。
- 11 个 ROS 包编译通过，`pip check` 通过，RViz 模型显示正常。
- 已实际验证 `wbc_fixed_headless` 双 TCP IK/FK，双手位置残差约 1–3 微米，腰部关节参与求解。
- **已完成真实 PhysX 固定底盘运动，两次独立一键启动各连续通过 3 轮 / 12 个目标。**
- 最终三轮实测：双手误差 3.43–7.90 mm、姿态误差最大 0.51°、底盘位置漂移最大 4.03 微米。
- 先看 `RESULTS.md`，排障全过程见 `REPRODUCTION_LOG.md`，视频见 `logs/physx-demo.mp4`。

当前运行和开发目录是 **`xense-genie_sim`**。初次复现曾使用独立 worktree；
按操作者要求，现已备份旧状态并迁回原仓库，环境也重新绑定到本目录。
迁移、构建及远端同步记录见 [MIGRATION.md](MIGRATION.md)。

## 已配置环境下运行

在 `xense-genie_sim` 仓库根目录执行：

```bash
mamba activate xense-wbc
```

### 已验证的纯 IK 测试

终端 1：

```bash
bash wbc_demo/run.sh moveit use_rviz:=false
```

终端 2：

```bash
bash wbc_demo/run.sh demo --ik-only
```

该测试仅调用 `/compute_ik` 和 `/compute_fk`，**不启动物理引擎、不发送关节指令**。
结果写入 `wbc_demo/ik_probe_report.json`。

### 物理仿真与自动运动

只有在阅读并同意 [NVIDIA Omniverse EULA](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html)
后，才执行以下命令。先关闭上面单独启动的 MoveIt，避免重复节点。

```bash
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh all
```

本环境已经补齐同版本离线 Kit 扩展缓存；首次启动仍可能编译 GPU 着色器缓存。日志位于
`wbc_demo/logs/sim.log` 和 `wbc_demo/logs/moveit.log`。
测试成功后窗口保持打开，按 Ctrl+C 停止此脚本启动的进程。

也可分终端启动 `bash wbc_demo/run.sh sim`、`bash wbc_demo/run.sh moveit`，
最后执行 `bash wbc_demo/run.sh demo --cycles 3`。

## 初版经典回归做什么

以下四点序列仅指 `bash wbc_demo/run.sh demo --cycles 3`；它需要已经运行的仿真与 MoveIt。
现在 `all` 默认执行升级版压力测试，不再执行这段经典序列。

- 物理配置 `scene_fixed.yaml` 中 `pin_base_to_world: true` 将底盘焊接到世界。
- MoveIt 规划组 `wbc_fixed_headless` 包含 5 个 body/waist 关节与双臂 14 个关节，排除底盘与头部。
- 每次请求显式携带固定底盘的 `planar_joint` 多自由度状态，在 `odom` 系统一求双手 IK/FK。
- 双手从初始姿态同步向前/向下移动 8 cm、16 cm，然后返回，保持目标方向不变。
- 求解时检查碰撞，并在执行前对关节插值路径采样检查自碰撞。
- 运动指令只发送腰部和双臂关节，不发送底盘、头部或夹爪指令。
- 物理测试记录双手 FK/姿态残差、实测跟踪误差、腰部变化及底盘漂移到 `latest_report.json`。
- 连续关节状态保存到 `logs/joint_trace.csv`，自动运行的控制台日志保存到 `logs/demo.log`。
- 判定阈值：IK 位置/姿态误差 5 mm / 1°、实测 TCP 误差 2 cm / 3°。
- 底盘三维位置/姿态漂移在运行中持续检查，阈值为 1 mm / 0.001 rad。

这是**全身 IK + 仿真关节位置控制**，不是力矩级动态 WBC。
路径为关节空间平滑插值，不保证 TCP 严格沿直线移动。自碰撞检查是离散采样，
不是连续碰撞检测；GDK 场景物体未自动同步到 MoveIt 的碰撞世界。
脚本仅用于本机仿真，不用于真机。

RViz 默认选择 `wbc_fixed_headless`，只显示实际机器人，避免目标姿态的半透明模型遮挡。
手动体验时，在 Displays → MotionPlanning → Planning Request 勾选 Query Goal State，
拖动双手 marker 后点击 Plan 观察规划。
本 demo 的 MoveIt 以 `use_ros2_control:=false` 启动，自动运动由脚本发送
`/joint_command`，所以 **RViz 的 Execute 不连接执行控制器**。

### 可选录制

仿真运行期间，在另一终端执行下列任一命令，再启动 `demo`：

```bash
python wbc_demo/record_rviz.py --seconds 90
python wbc_demo/record_camera.py --seconds 90
```

前者录制 RViz 窗口并将其置前，其他窗口遮挡会影响画面；后者直接订阅 OVRTX 的
`/genie_sim/free_camera_rgb/image_raw`，不采集桌面，并按 `scene_fixed.yaml` 的
`viewer_camera` 设置视角，默认读取 `scene_stress.yaml`，也可用 `--scene scene_fixed.yaml`。
输出分别为 `logs/rviz-demo.mp4` 和 `logs/physx-demo.mp4`；`--output-name` 可指定新录像名。
录像只用于观察，性能/精度以 JSON 和 CSV 为准。

## 原生 mamba 构建说明

`environment.yaml` 记录 conda 侧依赖；Python 侧关键版本如上。官方机器人 mesh
已通过 Git LFS 下载，独立 assets 数据集只下载了 GDK/blank 场景及包元数据，
没有下载约 29 GB 的完整数据集，因此其他场景不保证可用。

本机原生构建需要以下兼容处理，均保存在此 worktree：

1. CLI 识别 mamba 中的有效 ROS prefix，不再强制 `/opt/ros`。
2. CLI 的源码目录保护只检查选定工作区，不误判整个 `~/src`。
3. 控制器头文件补齐 `<cassert>`，速度插值临时变量显式初始化。
4. 依赖 urdfdom 的弃用 `#warning` 不提升为编译错误，保留其他 `-Werror`。
5. `typeguard==4.3.0`、`huggingface-hub==0.36.0`、`pytest-cov==5.0.0`
   与 Isaac 的 `typing_extensions`、`click`、`coverage` 固定版本兼容。
6. conda OpenCV Python 包缺少 pip RECORD：仅在新建环境中覆盖安装
   `opencv-python-headless==4.13.0.90`，未卸载整套 ROS/OpenCV C++ 库。
7. 安装 `isaacsim[extscache]==6.0.0.1`，解决在线 registry 缺失
   `isaacsim.anim.robot.schema` 导致的启动失败。

最终完整环境与 Python 版本锁定清单位于 `logs/environment-full.yaml`、
`logs/pip-freeze.txt`。这是本机审计快照，其中 editable 路径需要在其他机器上调整，
不是可无修改跨机器使用的安装锁文件。

首次配置 CMake 缓存使用：

```bash
colcon --log-base devel_log build \
  --base-paths source/geniesim_ros/src/ros_ws/src \
  --build-base devel_build --install-base devel --merge-install \
  --parallel-workers 6 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS=-Wno-error=cpp
```

后续增量构建已验证使用仓库标准 CLI：

```bash
GENIESIM_SKIP_AUTOBOOT=1 \
GENIESIM_WORKSPACE="$PWD/source/geniesim_ros/src/ros_ws" \
geniesim ros build dev
```

不要在此精简配置中运行全量 bootstrap；它会安装与本 demo 无关的其他模块。
