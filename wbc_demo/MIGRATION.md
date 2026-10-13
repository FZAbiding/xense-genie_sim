# 迁回原仓库与远端同步

## 当前目录

开发、运行、构建都使用 **`xense-genie_sim`**，不再使用同级的 `xense-genie_sim-demo`。
当前分支为 `main`，`origin` 是操作者指定的私人仓库：
`https://github.com/FZAbiding/xense-genie_sim.git`。

## 备份与迁移内容

迁移前完整保存原目录和索引差异，备份目录为仓库同级的
`xense-genie_sim-migration-backup-20261013-204916`：

- `original/`：迁移前原目录的完整快照。
- `original-index.patch`：原索引的 5721 项暂存删除记录。
- `demo-source.patch`：原 demo worktree 的源码差异。
- `demo-worktree/`：归档后的旧 worktree，包括旧构建产物与复现资料。
- `generated-scene-before-migration/`：含旧路径的自动生成场景，留存但不再加载。

迁移时先补齐缺失源码、复制本次补丁及 demo，再仅将原索引恢复至 HEAD；
没有用 `reset --hard` 丢弃工作文件，没有把 5721 个暂存删除提交到远端。
迁移后的差异清单没有发现需要另行合并的原目录用户代码改动。

重新 editable 安装了 CLI、ROS shim、assets 包到 `xense-wbc`，
不复制旧路径的 ROS 构建树，而是在原仓库重新编译。
旧生成场景也不直接复用，首次启动会重新生成正确路径的 URDF/USD。
运行脚本默认使用 `ROS_DOMAIN_ID=42` 隔离本 demo 的 ROS 图；可通过同名环境变量覆盖。

## 提交与推送状态

迁移已完成：11 个 ROS 包在原目录重新编译通过，`pip check` 通过，
真实 PhysX 回归的 4 个双手目标全部通过，底盘最大位置漂移约 2.39 微米。
三个 editable 包均已确认指向原目录。
第一阶段迁移提交 `d97563413f5797a1afe90bcc6ea74c3d22bab727` 已推送到 `origin/main`，
通过 `git ls-remote` 核对远端哈希一致。升级实验的结果见 `EXPERIMENT_RESULTS.md`。

## 后续运行

从本仓库根目录：

```bash
mamba activate xense-wbc
export OMNI_KIT_ACCEPT_EULA=YES
bash wbc_demo/run.sh all --cycles 3
```

后续修改就在此目录进行。提交前检查差异并只暂存需要的源码/文档，
普通推送即可，不使用 force push。

`assets/`、构建产物、mamba 环境和 `wbc_demo/logs/` 不上传 Git。
源码推送与本机依赖环境/运行视频是不同的交付物。
迁移原始日志位于 `wbc_demo/logs/migration/`。
