# 第 0 段最新图文媒体（camera_side_up_v2）

真实运动来自 2026-10-14 19:10:52–19:18:09 的 `umi-upright-execute-02`，不是 AI 生成图或纯规划动画。
PNG/GIF 可随文档版本管理；清晰 MP4、完整 CSV 与离线验收页在 `wbc_demo/logs/umi-evidence-episode-000-v2/`。
旧 `../episode-000/` 是保留的历史材料，不能与新结果混用。

- `robot-preview.gif` / `robot-storyboard.png`：新原生录像 8 倍速概览的低帧率动图/六帧；不是实时执行速度。
- `head-left-frame.png`：UMI 原始 head_left 与新实际机器人录像的源相位近邻匹配分屏。
- `camera-axis-before-after.png` / `gripper-eef-frame.png`：同一实际 URDF/STL 在旧新 EEF 中的投影，后者是验收包的模型图别名。
- `waist-roll-before-after.png`：旧新 IK 计划与物理反馈，包含两个改变量，不是单变量权重消融。
- `tracking-summary.png`：新实际 EEF 对完整源轨迹及腰部反馈曲线。
- `source-storyboard.png`：HF 第 0 段首/中/末帧三联相机截图。
- `manifest.json`：输入来源、抽帧索引及完整精简包文件哈希。

HF 源采集素材由 **TacVerse** 发布于 `TacVerse/xtac-umi-g1-block-to-box`，
固定 revision `8ad651578fda4724514387e2d3d3551c3c9631f9`，数据卡许可 **CC-BY-SA-4.0**。
包含源画面的 `head-left-frame.png`、`source-storyboard.png` 等衍生素材继续按 CC-BY-SA-4.0 共享，
进行了片段抽取、缩放、拼接、标题标注；不意味着仓库其他代码改用该许可。
上游入口：[TacVerse 数据主页](https://huggingface.co/datasets/TacVerse/TacVerse)。

新录像没有切换录制器，但有渲染停顿和重复帧。分屏两栏严格同长度，
源相位匹配最大差 73.704 ms / P95 17.826 ms，不是逐帧完全同时或经过传感器延迟标定的硬件同步。
