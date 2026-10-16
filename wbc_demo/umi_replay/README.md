# XTac UMI → 固定底盘 G2 回放

**从零学习**：[图解 FK、雅可比、解析/数值 IK 与 WBC，读懂 v5 误差](IK_LEARNING_GUIDE_ZH.md)。
附 [配图与实测诊断索引](media/ik-learning-v1/index.html)；特别区分模型一致性与实际跟踪精度。

**下一阶段研究**：[代码审计](CODE_AUDIT_ZH.md) · [Pinocchio 实测可行性](PINOCCHIO_STUDY_ZH.md) · [WBC 分阶段路线](WBC_NEXT_STAGE_ZH.md)。
Pinocchio 仅接入离线模型验证与可选测试，未更换在线默认后端；当前可视化研究结果见 [模型对照验收页](media/pinocchio-study-v1/index.html)。

**v5 原速在线 IK：[为什么慢、实时架构与核查方法](REALTIME_REPLAY_ZH.md)**。
Web 默认 1× / 30 Hz 在线微分 IK，UMI 使用浏览器原生视频；旧的预计算慢放模式仍可选择。
**直接看结果：[v5 自包含四分屏视频/图片验收页](media/episode-000-v5/index.html)**，本轮图像传输空档已修复，视频通过原有同步阈值。

**历史 v4：[夹爪开合与 Web 交互修复](GRIPPER_AND_WEB_FIX_ZH.md)**。
现在按 0 闭合 / 1 开启回放，使用随开度变化的指尖 TCP；
执行 `bash wbc_demo/run.sh umi-web` 后打开 `http://127.0.0.1:8765`，不是点击验收页的截图。

**[v4 同目录视频/图片验收页](media/episode-000-v4/index.html)**；本目录包含 MP4，不再依赖本机 logs 才能播放。

**新入口：[Web 四视角在线回放与 IK 原理](ONLINE_IK_AND_VIEWS_ZH.md)**。
原始 `head_left` + 同向虚拟第一视角 + 两个可拖动外部镜头，支持播放/暂停/恢复/停止；
Web 默认软直腰偏好，可切换 ±0.5° 对照策略，不新增 PyQt/PySide 依赖。
最新实际视频、误差和边界条件见 [四视角验收记录](MULTIVIEW_RESULTS_ZH.md)。

首次阅读推荐 **[给 VLA 工程师的科普、当前结果与工作记录](UMI_REPLAY_GUIDE_ZH.md)**：
从法兰/EEF、坐标变换、旋转编码讲到全身 IK、WBC、误差判断和复跑核查。
教程第 0 节现有真实运动 GIF、关键帧图与验收表；本机可打开 [最新版离线图文/视频验收页](../logs/umi-evidence-episode-000-v2/index.html)。
**当前应先看 [腰部/腕部相机/head_left 分屏/IK 来源核查](USER_FEEDBACK_REVIEW_ZH.md)**。
相机侧 EEF 修订为 `camera_side_up_v2`；旧 `umi-gripper-eef-*` 报告保留为历史，不再作为最新验收证据。

**当前默认已修正为机器人模型夹爪 EEF 控制，不再使用首帧拟合的虚拟工具。**
先读 [EEF 修正说明与结果](GRIPPER_EEF_CORRECTION_ZH.md)，再看 [数据坐标系说明](COORDINATES_ZH.md)。
[上一轮结果](RESULTS_ZH.md) 是历史虚拟 EEF 结果，不能作为本次夹爪 EEF 回放结果。
核查记录见 [执行记录](WORK_LOG.md)。
原始数据、转换产物与完整运行日志不进 Git；v4/v5 自包含验收包的 MP4、图片与报告随代码和文档保留。
原有交互实验台无需重写，回放复用其双臂/腰部 IK、安全限幅与状态检查。

## 目录职责

```text
wbc_demo/umi_demo.py                 单一 CLI 入口
wbc_demo/umi_replay/
  download.py                       固定 HF revision、下载预算、逐文件 SHA256
  dataset.py                        LeRobot v3、逐段外参与时间/置信度检查
  geometry.py                       ROS 无关的 6D rotation / SE(3)
  conversion.py                     原始 EEF 轨迹、误差受限关键帧选择
  preview.py                        原始三路视频预览、EEF 轨迹图
  analysis.py                       实际夹爪 EEF 对照完整源轨迹与误差图
  evidence.py / evidence_template.html 真实抽帧、GIF、离线图文/视频验收包
  synchronized_video.py             根据指令源相位与相机 ROS 时间做 head_left 分屏
  review.py                         模型相机侧、腰部 roll 新旧审计
  media/episode-000-v2/              最新可版本管理的文档 PNG/GIF 与来源清单
  gripper_eef.py                    URDF/STL 确定夹爪 EEF，禁止数据拟合
  robot.py                          夹爪 EEF FK/IK 接口与固定 TF
  execution.py                      ROS IK 预检、重定时、仿真执行与审计
  configs/g2_shared_world.json       仅共享世界摆放与安全限制
  tests/                            几何与真实数据测试
wbc_demo/data/                       被忽略：原始数据、转换文件
wbc_demo/logs/                       被忽略：预检/执行报告、CSV、视频
```

## 下载与转换（不依赖 ROS 后端）

在仓库根目录，使用已有 `xense-wbc` 环境：

```bash
mamba activate xense-wbc
python -m pip install -r wbc_demo/umi_replay/requirements.txt
python wbc_demo/umi_demo.py download
python wbc_demo/umi_demo.py convert --episode 0 \
  --output wbc_demo/data/converted/episode-000.json
```

默认完整下载 21 个文件 / 284,999,394 bytes，包含 8 路 RGB/触觉视频。
下载前检查 400 MB 上限；固定 revision，不随 HF main 的变动悄悄换数据。
`--metadata-only` 仅下载轨迹与元数据，但仍覆盖全部 11 段。
转换默认读实际观测 `observation.state`，不执行数据集提供的未来 action。
转换文件保留全部原始 EEF 帧、归一化开度、源时间和帧号，再附加关键帧索引。
同一路径不覆盖；重跑时使用新文件名。

生成原始头显/左右腕相机三联视频与轨迹图（不受任何桌面窗口遮挡）：

```bash
python wbc_demo/umi_demo.py preview \
  --input wbc_demo/data/converted/episode-000.json \
  --output wbc_demo/logs/umi-source-preview-new
```

## 先预检，再执行

先关闭旧实验台/旧仿真，不要重复启动后端。

```bash
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh umi-all \
  --input wbc_demo/data/converted/episode-000.json \
  --output wbc_demo/logs/umi-preflight-new
```

默认**只预检，不发送运动命令**。全部关键帧顺序求解；每段关节路径采样检查自碰撞、
关节原生范围和机械耦合约束，另外检查中点 FK 与目标插值偏差。
任何一个关键帧失败则整段失败，不缩小轨迹、不跳过失败帧，也不会只播放成功前缀。
默认控制点是两夹指远端面中点，工具关系由 URDF/STL 固定确定，距法兰约 .212006 m。
已移除左右初始 roll ±135° 的拟合逻辑；源姿态不再被这样的虚拟安装关系改变。
这是模型 TCP，不是厂家实测标定。详情见 EEF 修正说明。
最新 +X 朝指尖、+Z 朝相机侧，+Y 为 +Z × +X，不能用 URDF 的左右夹指命名猜跨设备轴向。
默认 UMI 回放对腰部 roll 加 ±0.5° IK 约束；其他腰部关节仍参与。

```bash
bash wbc_demo/run.sh umi-all \
  --input wbc_demo/data/converted/episode-000.json \
  --output wbc_demo/logs/umi-execute-new \
  --execute --speed 0.5
```

每次执行都会重新完整预检，先安全靠近第一帧，再连续回放并回家。
`umi-all` 启动独立后端并在结束/失败/中断后清理；日志在独立 `umi-backend-时间/` 中。
已有 `sim` + `moveit` 时用 `bash wbc_demo/run.sh umi replay ...`，但不能同时开实验台控制端。
`Ctrl+C` 请求停止并保持当前测量关节位置，不会继续播放后面的轨迹。

## 时间和误差：不冒充原速回放

- 源 30 Hz 轨迹不直接当成关节指令。
- 关键帧初始间隔最多 0.2 s，额外保留位置插值误差 > 5 mm 或旋转误差 > .03 rad 的帧。
- 每段关节用三次平滑插值，关键帧处停速；整段统一拉伸时间以满足速度/加速度限制。
- 这是保守、可能明显慢于人类的回放，不是 30 Hz 实时 IK，也不是高保真动力学模仿。
- `--speed` 是申请的速度上限，实际还受关节限速；读取报告的 `time_scale` 和实际 wall time。
- 循环限制调度延迟后的虚拟时间步，避免卡顿后突然追赶；视觉录像可能比名义重定时更长。
- 默认持续跟踪阈值 5 cm / 10°，末点 2 cm / 3°；超过即失败并保持。
- 底盘漂移 > 1 mm / .001 rad、反馈过期、多个命令发布者均停止执行。

注意：路径的离散碰撞采样不是连续碰撞的数学保证；未同步环境障碍，只验证空场景下模型自碰撞。
源关键帧近似误差、关节插值偏差、实际跟踪误差是三种不同误差，不要混成“IK 精度”。

## 核查文件

每轮 `--output`：`report.json`、`mapped_targets.json`、`joint_plan.json`、`joint_trace.csv`，
实际执行另有 `tracking.csv`，内含目标/实际位姿、源时间、ROS 反馈时间、左右误差与 5 个腰部角度。
报告保存输入哈希、源 revision、对齐/工具矩阵、逐帧 IK 残差和执行结果。
旧 `joint_trace.csv` 的 wall time 不等于设备采样时间；计算实际速度时应使用新的 ROS 时间记录。

运动完成后，计算**夹爪 EEF**对完整源轨迹的误差，而不只看关键帧 IK 残差：

```bash
python wbc_demo/umi_demo.py analyze \
  --input wbc_demo/data/converted/episode-000.json \
  --run-directory wbc_demo/logs/umi-execute-new
```

输出 `eef-metrics.json`、`eef_tracking.csv`、`tracking-summary.png`，
使用全部 30 Hz 源帧插值对照，包含采样近似、关节插值与实测跟踪的合并误差。
`source_time_s` 是三次插值当前命令对应的源相位，不是录像 wall time，也不是反馈采样时刻。

纯数据测试：

```bash
PYTHONPATH=wbc_demo python -m unittest discover -s wbc_demo/umi_replay/tests -v
```

## 生成可随文档一起验收的媒体资料

已有本机验收包：`wbc_demo/logs/umi-evidence-episode-000-v2/index.html`，包含同长度 head_left 分屏。
浏览器打开即可；没有 CDN、远程脚本、ROS 或仿真依赖。将整个目录复制到另一台机器可离线查看。
MP4 转为浏览器常见的 H.264 / yuv420p；原始慢速录像和运行结果不覆盖。

重新整理现有实测（**不重新启动机器人**），输出目录必须是新的：

```bash
mamba activate xense-wbc
python wbc_demo/umi_demo.py evidence \
  --run-directory wbc_demo/logs/umi-upright-execute-02 \
  --robot-video wbc_demo/logs/umi-upright-summary-8x.mp4 \
  --source-video wbc_demo/logs/umi-source-episode-000/source-episode.mp4 \
  --eef-diagram wbc_demo/logs/umi-upright-review/camera-axis-before-after.png \
  --synchronized-directory wbc_demo/logs/umi-upright-head-left \
  --review-directory wbc_demo/logs/umi-upright-review \
  --output wbc_demo/logs/umi-evidence-new
```

`--robot-video` 必须传已生成的 **8 倍速概览**，脚本不会再对它加速；`--source-video` 为 1 倍速三联源预览。
输入录像/模型图与本次运行的对应关系需由整理人确认，脚本不根据画面自动验证身份。
可另加 `--doc-media 新目录` 复制四张 PNG、GIF 和清单到文档媒体目录；同样拒绝覆盖已有目录。
打包前核对 `passed`、`gripper_eef`、回家状态，以及 EEF 指标是否与 CSV 一致；
历史虚拟 EEF 结果或指标不一致会拒绝打包。包内含报告、跟踪 CSV 与 SHA256，不含完整数据集/后端日志。
概览截图标的是视频 PTS，不是源相位；独立的源/机器人概览 **不做精确同步**。
单独分屏由新的 `sync-video` 命令生成；必须提供匹配的原速录像与逐帧时间 CSV，不能传加速剪辑。
本轮分屏最大源相位差 73.704 ms（渲染停顿），不是完美逐帧同时；方法/索引/复跑命令见反馈核查文档。
HF 源预览/截图来源 TacVerse，CC-BY-SA-4.0；衍生素材保留署名与许可，见媒体目录说明。

录制仿真原生相机（不受桌面窗口遮挡影响），在执行期间另一个终端运行：

```bash
bash wbc_demo/run.sh umi replay --help
python wbc_demo/record_camera.py --seconds 360 --output-name umi-replay-camera
```

第二条需先 `source devel/setup.bash`，默认 ROS domain 42。录像只记录仿真，不截取其他桌面程序。
回放只证明数据→坐标转换→双 TCP IK→PhysX 位置控制链条；夹爪/触觉/接触任务未复现。
