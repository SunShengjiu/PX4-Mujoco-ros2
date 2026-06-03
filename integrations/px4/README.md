# PX4 集成说明

第一版工作流不会把 PX4 源码直接放进本仓库，也不要求用户手动修改 PX4 文件。

这个目录只保存受支持流程所需的最小 PX4 侧集成资源。

## 包含的资源

- `patches/release-1.15-mujoco-delta.patch`

这个 patch 适用于一个干净的外部 PX4 clone，分支为 `release/1.15`。

## Patch 修改内容

1. 添加自定义 airframe：`22002_mujoco_delta`
2. 在 `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 中注册该 airframe
3. 更新 `px4-rc.mavlink`，让 PX4 把 GCS MAVLink 链路发送到 `127.0.0.1:14550`

## 为什么第一版只使用最小 patch

受支持的工作流是：

- 本仓库负责 Python MuJoCo bridge
- 外部 PX4 clone 保持标准 `px4_sitl_default` 构建
- 本仓库负责编排 bridge、PX4 和 QGroundControl 的启动

这样集成关系更容易理解，也更容易在新的 Ubuntu 机器上验证。
