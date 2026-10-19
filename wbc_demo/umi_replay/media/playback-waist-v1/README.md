# 卡顿、首帧对齐与腰臂优先级核查

先读 [配套中文讲解](../../PLAYBACK_ALIGNMENT_AND_WAIST_ZH.md)。

这是一份**诊断证据**，不是新控制策略整段验收通过的声明。

- [首帧共同世界变换图](alignment.svg)：教学示意，不是标定结果。
- [实际 MJPEG 到达帧率 / 帧间隔](delivery.png)：修复前后各 15 s 静止窗口；平均接近 30 fps 不等于消除长尾。
- [腰部骨架与关节日志](waist-initial.png)：由 URDF FK 和旧 run-002 日志生成；中间是只读新候选，未实际执行。
- [真实 Web 截图](web-controls.png)：修复后真实 Chromium，等待操作状态；此时机器人保持此前失败后的位姿，源视频为首帧，二者不是同步回放。源视频 fps 低因为未播放。
- [实际鼠标操作结果](controls.json)：外部视角拖动、缩放、重置；不发布机器人运动目标。
- [汇总数值](metrics.json)：含输入哈希、模型固定腰候选、短窗统计；微小模型残差不是物理精度。
- [两次新回放失败](failures.json)：保留错误、周期与 IK 耗时、源 tick、来源哈希，不把失败隐去。
- [来源完整性](manifest.json)：本目录静态证据与实现源码哈希。

静止前后探针保持机器人位置和外部相机轨道；Web 重启将 ego 的头部轨迹相位归零，因此 ego 画面内容并非完全一致。
这些探针不是完整的控制性能 A/B。浏览器操作截图中的接收/编码 fps 也会低于静止窗口，不能用约 30 fps 的均值承诺全负载表现。

原始数据留在本机：

```text
wbc_demo/logs/playback-waist-review/
wbc_demo/logs/wbc-online-web-20261019-02/run-002/   已完成，含一次暂停
wbc_demo/logs/wbc-online-web-20261019-02/run-003/   新失败，夹爪跟踪保护
wbc_demo/logs/wbc-online-web-playback-review/      新失败，长周期后跟踪保护
```

重新生成图表的命令见配套文档。`tests/playback_probe.mjs` 可使用显式的 `PLAYWRIGHT_MODULE`
指向已有 Playwright ESM 模块；`inspect` 只操作镜头，`execute` 才请求机器人回放。
Playwright 不是生产 Web 服务的新依赖；不要把浏览器 QA 模块路径写死在业务代码中。
