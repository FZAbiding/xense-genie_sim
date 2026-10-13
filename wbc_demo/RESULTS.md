# 复现结果：G2 固定底盘双臂 + 腰部全身 IK

## 结论

**成功。** 使用 mamba 环境 `xense-wbc`，真实运行 Isaac Sim 6.0 / PhysX，
不是仅显示模型或离线求解。底盘焊接到世界，5 个腰部/body 关节与双臂 14 个关节
联合求双 TCP IK，经平滑关节插值发送到仿真位置控制器。

两次独立一键启动各通过 **3 轮、12 个双手目标**。最终三轮运行于
2026-10-13 **00:29:31–00:31:11 CST**，完整原始结果单独归档，后续重跑不会覆盖。

| 最终三轮指标 | 实测结果 |
|---|---:|
| 双手 IK/FK 最大位置残差 | 2.83 微米 |
| 双手 IK/FK 最大姿态残差 | 0.000758° |
| 双手仿真关节反馈 FK 位置误差 | 3.43–7.90 mm |
| 双手仿真关节反馈 FK 最大姿态误差 | 0.510° |
| 单个腰部/body 关节最大相对变化 | 0.418 rad，约 24.0° |
| 全程底盘三维位置漂移最大值 | 4.03 微米 |
| 全程底盘旋转漂移最大值 | 0 rad，在 TF 数值精度内 |

测量说明：TCP 实测值来自仿真关节反馈的 URDF FK，不是外部视觉或 USD mesh 测量。
漂移相对于开始运动时的底盘 TF；不宣称启动前后的绝对位置误差。

## 明早先看这些

- **`logs/physx-demo.mp4`：40 秒 OVRTX 场景运动录像，直接从渲染图像话题录制，不受桌面遮挡影响。**
- `logs/rviz-preview.png`、`logs/physx-latest.png`：画面预览。
- `logs/rviz-demo.mp4`：90 秒 RViz 参考录像，后段被其他任务的 MuJoCo 窗口遮挡，以 PhysX 视频为主。
- `logs/final-run-report.json`、`logs/final-metrics.json`：最终三轮逐目标结果及汇总。
- `logs/final-joint-trace.csv`：最终三轮的连续关节轨迹。
- `logs/final-demo.log`、`logs/final-all-launch.log`：最终三轮执行日志。
- `logs/sim.log`、`logs/moveit.log`：物理引擎、渲染、MoveIt 原始启动/运行日志。
- `REPRODUCTION_LOG.md`：缺失扩展、TF/虚拟底盘启动竞态的失败记录和修复过程。
- `logs/final-build.log`、`logs/validation.log`：构建及依赖、语法、格式检查。
- `logs/environment-full.yaml`、`logs/pip-freeze.txt`、`logs/source-changes.patch`：环境与修改快照。
- `logs/warm-repeat-demo.log`、`logs/warm-repeat-report.json`：视频对应的额外一轮温启动测试，也已通过。

## 重跑

测试完成后已停止本 demo 的仿真/MoveIt，释放 GPU；其他任务未停止。
进入 `xense-genie_sim` 仓库根目录即可重跑，避免同时启动多个副本：

```bash
mamba activate xense-wbc
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh all --cycles 3
```

窗口在成功后保持打开。Ctrl+C 仅清理脚本自己启动的进程组。

## 边界

- 这是**全身 IK + 关节位置 PD 控制**，不是力矩级动态 WBC，也没有实现平衡/接触力分配。
- 自碰撞为离散路径采样检查，GDK 场景货架等没有自动同步进 MoveIt 的碰撞世界。
- 仅用于仿真。没有连接真机，没有配置 RViz Execute 控制器。
- 共享 GPU 同时存在其他任务，未停止它们；不将目标 100 Hz / 30 FPS 当作硬实时保证。
- 初次复现时因原索引存在 5721 项暂存删除而使用独立 worktree；现按操作者要求
  完成备份和迁回原仓库。原始测试数据保持不变，当前目录与同步情况见 `MIGRATION.md`。
- 仅下载本 demo 所需机器人模型和 GDK/blank 场景；其他场景不保证可用。
