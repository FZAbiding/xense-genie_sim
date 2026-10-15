# 第 0 段文档媒体与来源

这些文件用于 `../../UMI_REPLAY_GUIDE_ZH.md` 的图文验收，可跟代码一起版本管理。
清晰视频和原始数值证据在本机 `wbc_demo/logs/umi-evidence-episode-000/`；复制该整个目录可离线查看。
没有重新运行仿真，没有使用 AI 生成图像或重绘机器人画面。

| 文件 | 生成来源与用途 |
|---|---|
| `robot-preview.gif` | 有效夹爪 EEF 实测原生相机的 8 倍速概览，再降为 480 px / 4 fps；不是控制速度 |
| `robot-storyboard.png` | 同一概览均匀抽取 6 帧；标题为视频时间/帧号，不是源相位 |
| `source-storyboard.png` | HF episode 0 三联相机首/中/末帧；缩放、拼接、标题标注 |
| `gripper-eef-frame.png` | Swiftpicker 实际 URDF/STL 生成的模型 TCP 图，不是实机标定 |
| `tracking-summary.png` | 有效实测 CSV 对完整 30 Hz 源 EEF 的轨迹/误差/腰部曲线 |
| `manifest.json` | 输入来源路径、源 revision、抽帧索引、精简包文件 SHA256 |

源采集素材由 **TacVerse** 发布于 Hugging Face 数据集 **`TacVerse/xtac-umi-g1-block-to-box`**，
固定 revision `8ad651578fda4724514387e2d3d3551c3c9631f9`，数据卡许可 **CC-BY-SA-4.0**。
衍生的源相机截图 `source-storyboard.png` 仍按 CC-BY-SA-4.0 共享，修改如表所列；不意味着其他仓库文件改用该许可。
上游项目入口见原数据卡引用：[TacVerse 数据主页](https://huggingface.co/datasets/TacVerse/TacVerse)。

机器人录像有录制切换缺口和重复帧。不得把这些素材描述成连续全程录像，
也不得声称源视频与机器人视频逐帧精确同步；完整轨迹覆盖与误差核查依赖运行报告/CSV。
底盘锁定与腰部运动只代表空场景仿真，第 0 段之外没有实际运动验证。
