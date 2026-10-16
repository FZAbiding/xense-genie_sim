# IK 学习配图与旧 v5 日志诊断

完整教程：[IK_LEARNING_GUIDE_ZH.md](../../IK_LEARNING_GUIDE_ZH.md)。浏览配图：[index.html](index.html)。

- `01`：二连杆 FK、Jacobian 与两支解析解；`02`：数值迭代；`03`：v5 职责链；`05`：误差向量分解。均为教学示意，不是机器人新实验。
- `04`：从旧 v5 `umi-realtime-web-reliable-01/run-001` 原始日志重算的曲线；无新机器人运动，不改历史验收指标。
- `metrics.json`：二连杆实际计算结果、v5 统计与输入 SHA256。速度贴近限值占比 0%，加速度贴近限值占比 23.52%；贴边不等于完成误差因果归因。
- 图由 `umi_replay.learning_materials` 生成，中文字体 Noto CJK；无 AI 生成“实验截图”。原 Pinocchio 基准图与 v5 真视频通过相对链接引用，不复制大视频。
- 没有完成 Pinocchio 在线 A/B、实测状态闭环升级或新的力矩 WBC 实验。

生成/复跑命令见教程第 15 节，输出目录必须不存在。
