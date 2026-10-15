# UMI 的“世界”和 EEF：先弄清你究竟在回放什么

**当前默认控制机器人模型夹爪 EEF；最新相机侧轴定义与来源核查见 [用户反馈核查](USER_FEEDBACK_REVIEW_ZH.md)。**
本文第 5 节中的虚拟工具方案仅保留为历史审计，已停用。

## 1. 不要把不同 UMI 系统的世界系混为一谈

UMI 是采集/迁移思路，不代表所有数据都共享一个世界原点。

**Stanford 原版 UMI**：官方 `06_generate_dataset_plan.py` 读取 SLAM 相机轨迹，
用 mapping 阶段的桌面标签变换把 SLAM 地图坐标变成标签坐标，再乘相机到工具末端的固定变换。
可以写成 `T_tag_tcp = inverse(T_slam_tag) × T_slam_cam × T_cam_tcp`。
这里的世界不是机器人底座，也不是每一帧相机坐标；工具平移偏置与具体硬件有关。
不能把原版 GoPro 的偏置照抄到 XTac。

**本次 TacVerse XTac-UMI-G1**：采集设备是双手手持夹爪 + PICO tracker + 头显，
不是 Unitree G1 机器人。HF 的 `meta/info.json` 实际写的是 `robot_type: xtac_umi_g1`。
其位姿来源是 PICO 跟踪链，不是 Stanford 的 GoPro/ORB-SLAM 导出链。

## 2. XTac 导出代码的实际坐标契约

核查本机 `TacCap-core` 的两个真源文件：

- `crates/taccap-mcap/src/lib.rs`：`pico_pose_to_world`、`compose_tracker_to_ee`。
- `crates/taccap-lerobot/src/bi_taccap.rs`：`ee_pose`、`append_pose`、`rotation_6d_xyzw`。

导出代码定义 world 为 **X 前、Y 左、Z 上，单位米**，轴映射为：

```text
p_world = G × p_pico = [-z_pico, -x_pico, y_pico]
G = [[ 0, 0,-1],
     [-1, 0, 0],
     [ 0, 1, 0]]
R_world_tracker = G × R_pico_tracker × transpose(G)
T_world_eef = T_world_tracker × T_tracker_eef
```

外参平移在 tracker 局部系中：`p_eef = p_tracker + R_tracker × p_tracker_eef`，
不能直接把外参平移加在 world 的 XYZ 上。
外参 JSON 的四元数是 **wxyz**，运行中与本回放文件的四元数是 **xyzw**。

邻近采集驱动 `lerobot-xense/.../pico4/tracker.py` 文档说明：原点在 Unity VR app 启动时建立，
不是点击连接或 Python 初始化时建立，重新启动 Unity 会改变原点。
**这是驱动代码的说明，不是本数据集提供的一次原点测量**；HF 没有发布这 11 段数据的
“world→机器人底座”标定，也没有证明它们所有 session 的启动原点完全相同。
驱动同时提醒原始 PICO 轴/手性上游文档不一致；我们遵循实际导出代码与已导出的数据，
不凭品牌文档猜测另一套镜像变换。

一个直观例子：原点在戴着头显的人的头部附近，桌上的 EEF 的 Z 是 -0.7 m 很正常，
表示低于原点 70 cm，不表示夹爪穿到地底。**把这个 Z 原封不动发给机器人的 odom 必然错。**

## 3. 数据里已经是 EEF，而不是 tracker

本次固定 HF revision `8ad651578fda4724514387e2d3d3551c3c9631f9`：

- 11 段 / 13,500 帧 / 30 Hz，LeRobot v3。
- `meta/taccap_extrinsics.json` 每段均为 `poses_are_ee: true`、`legacy_world_pose: false`。
- 左右 tracker→EEF 的平移分别约 `[-.16077,-.10586,.02490]`、`[-.16193,.10611,.02532]` m。
- 这些外参已应用。**本转换器不会再次乘它们，也不会再次做 PICO→world 轴映射。**

EEF 的机械含义来自采集代码中的 CAD 装配末端框架；源码说明末端位于两指中点。
这不等于当前 G2 的 `arm_l_end_link/arm_r_end_link`，也不意味着两种夹爪的实际接触点已完成标定。
`XTac-UMI-Description` 的 CAD EEF link 同样不能自动当成真实传感器标定结果。

## 4. r1…r6 不是欧拉角，也不是四元数

每只手 10 维：`x,y,z,r1,r2,r3,r4,r5,r6,gripper.pos`。
29 维状态还附加 9 维头显位姿，本次不回放头部。

```text
第一列 a = [r1,r2,r3]
第二列 b = [r4,r5,r6]
u = normalize(a)
v = normalize(b - dot(u,b) × u)
w = cross(u,v)
R = column_stack(u,v,w)
```

注意是**两列分组**，不是把六个数 reshape 成前两行。
转换器检查有限值、近似单位长度与正交性，再转为 xyzw 四元数。
例如 Z 轴转 90° 时，两列是 `[0,1,0]` 和 `[-1,0,0]`。

`action[t]` 在本数据实际等于下一时刻的 `observation.state[t+1]`（非末帧逐元素核对通过），
不是位置/角度增量。默认回放 `observation.state`，避免平白提前一帧；可显式选择 `--feature action`。

## 5. 从采集世界迁移到 G2：两个变换，不是一句“坐标对齐”

定义 `T_A_B` 表示 B 坐标系在 A 中的位姿，也将 B 中的点变换到 A。
MoveIt 接收的是 `odom` 中的臂末端 link 目标：

```text
T_odom_tip = T_odom_dataset × T_dataset_eef × inverse(T_tip_eef)
```

- 左乘：采集世界到仿真 odom 的**同一个**变换，左右手共享。
- 右乘逆：每只手的工具安装变换，左右可以不同。
- 不能左右分别平移“对上第一帧”后宣称完整保留双手的相互几何。

上一轮 `shared_world_virtual_tools` 是**历史仿真运动重定向，已停用**，不是夹爪模型 TCP：

1. 保持比例 1:1、Z 向上与默认零 yaw。
2. 一次共享平移，把第一帧双手中点放到配置中的 `[.20,0,1.05]` m。
3. 给两只臂末端定义沿其本地 +Z 平移 .212 m 的“虚拟工具”；第一帧臂末端 roll 分别为 +135°、-135°，呈向内且向下的姿态。
   固定工具旋转由这些显式初始朝向与源第一帧朝向求出，避免依赖微小的初始物理漂移。
4. 此后完整乘 SE(3)，**保留原数据朝向变化，不是一直锁定腕朝向**。

这像把两支笔连同它们画的图整体搬到另一张桌子，再改变夹持笔的安装角度：
整幅图不缩放、左右相对距离不变，但桌子上的绝对位置是人为选择的。
报告保存全部矩阵，你可以复算 `T_odom_tip × T_tip_eef` 来检查 EEF 世界轨迹。

.212 m 是**明确暴露的虚拟工具距离，不是测得的 TCP 标定**。
它与当前 Swiftpicker 初始开度下 URDF/STL 的远端面位置数量级一致：
`.007 + .10415 + .037365 + .0635 ≈ .212015 m`。
源码中的“臂末端 link”并不在手指末端；忽略这段几何会把整个夹爪塞到数据 EEF 点上。
最初零偏置版本首帧失败，关闭碰撞检查仅作诊断时，明确检出左右夹爪底座互撞；
实际预检/执行始终开启碰撞检查。中途还测试过水平、竖直工具姿态，失败结果保留在日志里。
最终选择的向内/向下姿态不改变源 EEF 轨迹比例或左右相对位姿，但改变 G2 手腕相对虚拟 EEF 的安装关系。

当前 `mode: calibrated` 只接受 `world_to_odom`（4×4），不再允许覆盖模型工具矩阵；
以前提供 `tip_to_eef` 的方式已被拒绝。不要填单位阵冒充实测世界标定。

## 6. 本次边界

机器人底盘焊接到世界，5 个腰/躯干关节 + 14 个双臂关节一起求解。
数据没有 G2 的关节角，必须 IK 重算；头部不动，夹爪开度只记录、不驱动。
没有重建原采集桌面/方块/盒子的接触场景，所以**轨迹回放成功不等于抓取任务复现成功**。
这仍是运动学回放 + 位置控制，不是动力学 WBC 或实机安全控制器。

## 来源与证据

- Stanford 官方 UMI 源码：浏览器核查 `real-stanford/universal_manipulation_interface` 的 `scripts_slam_pipeline/06_generate_dataset_plan.py`。
- HF 数据卡、info、逐段外参、设备快照：本地 `wbc_demo/data/xtac-umi-g1-block-to-box/`。
- 本机 TacCap-core commit：`40255689fd44c3b66310dfb3e63d686e30ddc737`。
- 本机 lerobot-xense commit：`1de0dc8bd9c94a0046b52fe6677f87d6e7b93809`。
- 源码快照、哈希与下载清单：`wbc_demo/logs/umi-research/source-evidence/`、数据目录 `download-manifest.json`。
- 本机源码是核对机制的证据；HF 未提供精确导出程序 commit，不能声称上述 commit 就是历史导出版本。
