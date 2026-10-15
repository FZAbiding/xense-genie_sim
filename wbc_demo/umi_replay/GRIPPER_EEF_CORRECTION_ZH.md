# 修正：UMI EEF 必须对齐机器人夹爪 EEF，而不是法兰

**v4 已支持 [开合相关的动态指尖 TCP](GRIPPER_AND_WEB_FIX_ZH.md)**；下文固定开启模型是历史实现，不再是当前限制。

> 历史说明：本文件记录凌晨第一轮几何 TCP 修正。随后用户指出相机在下方，发现其 EEF +Z 相机侧符号还需纠正。
> 最新轴定义、近零侧倾实测和同长度分屏见 [用户反馈核查](USER_FEEDBACK_REVIEW_ZH.md)。不要将本文件旧指标当作最新结果。

## 先承认上一轮的问题

上一轮虽然没有直接把所有 UMI XYZ 当作法兰 XYZ，但用了**按数据首帧拟合旋转的虚拟工具**。
这能得到一段可执行的运动，却不能证明机器人实际夹爪末端按 UMI 姿态运动。
用户指出的问题成立；之前的 15.46 mm 等数字是历史虚拟 EEF 结果，不能拿来证明本次夹爪 EEF 回放正确。
旧日志不覆盖，旧方案不再是默认，也不再接受 `shared_world_virtual_tools` 执行配置。

## 当前夹爪 EEF 的物理模型定义

当前机器人是 G2 + crsB + Swiftpicker。其 URDF 已移除旧 `gripper_*_center_link`，
源码明确要求下游自行应用 TCP 变换；不能把 `arm_*_end_link` 改个名字就称为夹爪 EEF。

本次采用可复算的**模型 TCP**：

- 原点：当前固定关节零位下，两只夹指最前端面的中心点的中点。
- 当前零位是开夹状态，不是“归一化开度为 0”的闭夹状态。
- +X 沿夹爪工具轴朝向指尖，+Y 朝左夹指，+Z 为叉乘方向。
- 使用真实 URDF 安装变换、夹指运动链与 STL 顶点计算，**不读取数据第一帧姿态来拟合工具旋转**。
- 它是仿真模型定义，不是厂家实测 TCP 标定，也不宣称有微米级硬件绝对精度。

左右夹爪同构，计算得到相对 `arm_l_end_link/arm_r_end_link`：

```text
p_flange_eef ≈ [-0.0000172504, 0, 0.2120057335] m
R_flange_eef ≈ Ry(-90°)
```

其中约 17 微米的横向量来自左右指尖网格的小几何不对称，代码保留原值，不强行抹零。
图示：`wbc_demo/logs/gripper-eef-correction/gripper-eef-frame.png`。
绿色点在两指尖之间，红色点是法兰，两者相距约 212 mm，绝不是同一个控制点。

模型计算实现：`gripper_eef.py`。每轮报告的 `gripper_eef_model` 保存 URDF/网格哈希、
每只夹指的远端面中心、夹爪 base→EEF 和法兰→EEF 矩阵。

### 为什么 EEF 的 +X 朝向指尖？

本次显式采用与 XTac 原始采集 CAD EEF 一致的工具轴习惯：原始
`XTac-UMI-Description/.../provenance/urdf/{left,right}.urdf` 中 `joint4` 的旋转为零，
与原本体坐标同向，而该原本体 +X 朝向指尖。
较新的 description 文档记录过把 EEF 改成 +Z 工具轴的变化，不能把新模型的轴约定不加区分地套在旧数据上。
本机核查版本为 `c46c088579f15b1f08e8aba7c313495e4813b0b0`，源文件哈希见
`wbc_demo/logs/gripper-eef-correction/source-axes-evidence.json`。
HF 没有发布精确导出程序 commit；这是公开定义/本机 CAD 源文件核查，不冒充一次新的硬件轴标定。

## 控制链现在以夹爪 EEF 为任务空间

```text
UMI 数据 EEF
    ↓ 左右共用的采集世界→odom 变换
机器人夹爪 EEF 目标
    ↓ 固定、由机器人模型确定的 TCP 安装关系
求解器内部的法兰目标 → 全身 IK → 腰部/双臂关节角
    ↓ 实际关节反馈 + FK + 同一个固定 TCP 关系
实际机器人夹爪 EEF → 与 UMI EEF 目标计算误差
```

严格公式：

```text
T_odom_robot_eef_target = T_odom_dataset × T_dataset_umi_eef
T_odom_flange_target   = T_odom_robot_eef_target × inverse(T_flange_robot_eef)
T_odom_robot_eef_actual = FK_flange(q_actual) × T_flange_robot_eef
```

**求解器内部仍使用法兰 link 名称，不等于任务目标是法兰。**
这是标准 TCP 换算：后端只支持法兰接口时，必须先乘工具变换的逆。
错误的是让法兰本身去到 UMI EEF 的同一个位姿，或把工具旋转随数据重新拟合。

`robot.py` 中的 `GripperEEFLab` 同时覆盖 IK 入口和 FK 返回值，
所以预检残差、插值中点检查、实时误差、RViz 目标箭头、轨迹曲线、回家检查全部使用夹爪 EEF。
报告内 `tracking_pose_frame: gripper_eef`；`solver_internal_links` 单独披露后端的法兰名。
`tracking.csv` 记录的就是 EEF，不需要后处理再给它冒加一次 TCP 偏置。

另广播 `arm_l_end_link→gripper_l_eef`、`arm_r_end_link→gripper_r_eef` 静态 TF，
可在 RViz 中直接核对夹爪点。没有向 PhysX 添加虚假的带质量 TCP 刚体，不改变动力学模型。

## 位置、朝向与开度边界

只用**一个共享世界变换**摆放整段数据，默认首帧双手中点 `[.30,0,1.05]` m、世界 yaw 为零。
不缩放、不独立平移左右手、不把首帧手腕强行旋转到某个“好看姿态”。
朝向就是源 EEF 朝向经共享世界旋转后的结果；硬件工具关系由模型固定。

夹爪仍保持初始零关节开度，数据的开合量只记录、不执行；模型点据此定义。
如果夹爪离开参考关节姿态超过 .02 rad，回放拒绝继续，避免把随手指几何变化的点当成原来的固定 TCP。
以后增加开合回放，需要重新明确 TCP 的参考开度/指尖几何，而不是去掉检查掩盖问题。
这不是块到盒子的接触任务复现，也不是实机安全控制器。

## 复跑与核查

命令保持不变，默认已经切换为模型夹爪 EEF：

```bash
mamba activate xense-wbc
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh umi-all \
  --input wbc_demo/data/converted/episode-000.json \
  --output wbc_demo/logs/umi-gripper-eef-my-run \
  --execute
```

去掉 `--execute` 只预检。
`mode: calibrated` 现在只接受世界变换 `world_to_odom`，不允许覆盖模型工具矩阵来拟合数据。
已新增测试验证：TCP 在网格指尖中点、轴向正确、EEF→法兰→EEF 闭环、旋转后的工具平移、
工具变换不随数据改变，以及拒绝旧虚拟工具配置。

本次预检证据：`wbc_demo/logs/umi-gripper-eef-preflight-01/report.json`，215/215 通过。
实际运动证据：`wbc_demo/logs/umi-gripper-eef-execute-01/`，结果完成后见本文后续记录。

## 重新执行的结果

2026-10-14，02:05:38–02:14:36（Asia/Shanghai）：

| 指标 | 模型夹爪 EEF 修正版实测 |
|---|---:|
| episode 0 原始帧 / 关键帧 | 1,250 / 215 |
| 预检、实际回放、回家 | 全部通过 |
| 跟踪参考 | `gripper_eef`，不是法兰，也不是旧虚拟工具 |
| 关键帧最大 IK 位置残差 | 7.934 微米 |
| 源相位覆盖 | 0–41.633335 s，全段 |
| 实际轨迹执行 wall time | 491.945 s |
| 对全部源 EEF 的左/右最大位置误差 | 11.980 / 12.605 mm |
| 左/右位置误差 P95 | 7.301 / 6.218 mm |
| EEF 最大姿态误差 | 1.591° |
| 底盘最大平移漂移 | 1.520 微米 |
| 底盘最大旋转漂移 | 0 rad（TF 精度内） |
| 单个腰部关节最大变化范围 | 55.250° |
| 实际左右跟踪采样 | 各 2,296 点 |

这些是重新运行的结果，没有复用或改名上一轮的数字。
完整报告、EEF CSV、误差图在 `wbc_demo/logs/umi-gripper-eef-execute-01/`。
预检/控制后端日志在 `wbc_demo/logs/umi-backend-20261014-020537/`。

模型图：`wbc_demo/logs/gripper-eef-correction/gripper-eef-frame.png`。
录像分为 `wbc_demo/logs/umi-gripper-eef-physical-01.mp4` 和 `umi-gripper-eef-physical-02.mp4`；
为增加自动随报告结束的录制功能，两个录制器之间存在短暂切换缺口，**不冒充完整连续录像**。
8 倍速拼接概览为 `wbc_demo/logs/umi-gripper-eef-summary-8x.mp4`，也保留这个切换缺口。
轨迹完整性以完整源相位、关节反馈 CSV、EEF CSV 和执行报告证明，不靠视频拼接宣称。
直接相机截图无桌面遮挡；误差图的长图例曾压住坐标标签，已移到图外并重新生成。

26 个 UMI/EEF 测试和原有 10 个实验台测试通过。
第一轮新增测试曾错误地假设所有横向几何量严格为零，随后按实际 17 微米网格偏置修正期望值；
没有修改模型数据来迎合测试。旧结果和失败日志保持原样。
