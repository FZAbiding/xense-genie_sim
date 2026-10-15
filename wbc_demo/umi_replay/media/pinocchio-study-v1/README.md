# Pinocchio G2 模型研究 v1

打开 [index.html](index.html) 查看同目录实测图和 [metrics.json](metrics.json)。
这是离线模型兼容性与计算段基准，不是 Pinocchio 整段仿真闭环演示。

- 输入：v5 实际回放 `umi-realtime-web-reliable-01/run-001/joint_trace.csv`，均匀采样 100 个完整反馈状态。
- Pinocchio 4.1.0 运行在独立 logs venv；没有改动 xense-wbc 基础包或默认在线 IK 后端。
- 指尖 EEF、实测从动关节偏差和测试世界变换保持一致，最大 Jacobian 元素差 `2.410e-8`。
- 四项可选模型测试还覆盖同一 QP 的单步命令对照；不能据此宣称实机力控或闭环抗扰已通过。
- `metrics.json` 包含 URDF、实际回放报告和反馈 CSV 的输入 SHA256，原始文件仍在本机 logs，不公开整个数据集。
- 原理、限制和复现命令：[PINOCCHIO_STUDY_ZH.md](../../PINOCCHIO_STUDY_ZH.md)。
