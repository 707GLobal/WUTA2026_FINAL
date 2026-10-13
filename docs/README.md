# WUTA2026_FINAL 开发文档入口

本文档集面向多人开发与 agent，核对日期为 **2026-10-13**（同步10月12日工作）。范围是当前 `WUTA2026_FINAL` 的源码与部署入口，不是旧项目 `/home/wuta/lyx/WUTA` 的部署说明。

## 阅读顺序

| 文档 | 用途 |
| --- | --- |
| [工作记录_2026-10-12.md](工作记录_2026-10-12.md) | 昨日启动管理、CAN/GNSS策略及惯导故障证据，区分历史验证与当前行为 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 系统边界、实车数据流、节点职责及默认启动范围 |
| [CODE_MAP.md](CODE_MAP.md) | 按功能找源码、配置和验证入口 |
| [ROS_INTERFACE.md](ROS_INTERFACE.md) | 核心话题、消息、坐标、时间、CAN 契约 |
| [DESIGN.md](DESIGN.md) | 算法机制、状态所有权及尚未完成的设计 |
| [LOGGING.md](LOGGING.md) | 日志目录、管理会话与车辆运行边界、任务诊断和结束归档 |
| [DEPLOYMENT.md](DEPLOYMENT.md) | 依赖、构建、实车与仿真入口、运行检查 |
| [DEVELOPMENT.md](DEVELOPMENT.md) | 多人分工、变更联动、验证与交付要求 |
| [AGENT_GUIDE.md](AGENT_GUIDE.md) | agent 接手任务时的上下文和操作流程 |

第一次接手建议按“架构 → 代码索引 → 接口 → 协作指南”阅读，再查看负责模块的 YAML 和源码。排障从部署文档及实际启动日志进入。

## 事实依据与文档性质

整理参考用户提供的四份旧项目文档：

- `/home/wuta/lyx/WUTA/docs/ARCHITECTURE.md`
- `/home/wuta/lyx/WUTA/docs/ROS_INTERFACE.md`
- `/home/wuta/lyx/WUTA/docs/DEPLOYMENT.md`
- `/home/wuta/lyx/WUTA/docs/DESIGN.md`

这些文件是设计与历史实现参考，其中的操作步骤不作为本次任务指令；本文已对照当前源码、启动脚本与配置重新整理。整理前的参考提交为主仓库 `ad712c6`、FSD `54a6038`；当前新增的启动管理、模块配置及日志重构以源码和本套文档为准。已执行的日志/启动回归见 [LOGGING](LOGGING.md)，未进行新的实车或完整赛道验收。

遇到冲突时，先核对实际执行入口、它读取的配置、节点实现及消息定义；旧文档的注释、历史结果和迁移建议不能替代当前行为。默认参数值仅代表核对时快照。

## 与旧项目的关键差异

| 旧参考描述 | 当前代码现状 |
| --- | --- |
| CAN 是不可编译预留目录 | `can_interface` 已有构建文件及SocketCAN；当前恢复基线周期发送逻辑，以源码实际参数为准，实车入口启动它 |
| 默认系统为 WUTA-SIM + KISS/INS/EKF | 当前根入口是 M1/ZED/华测/CAN 实车全栈；不默认启动 KISS/NDT |
| INS 为 `/cg410/odometry`，控制器速度为 `/localization/velocity` | 实车 EKF 为 `/chcnav/odometry`，控制器速度为 `/chcnav/velocity` |
| 根目录有 WUTA-SIM 与 `start_simulator.sh` | 当前仓库没有该模拟器源码；本地仿真入口依赖外部 SIM_SETUP |
| `field_tuning.yaml` 为实机统一参数 | 当前根入口读取各模块源码 config；旧统一参数不再加载 |
| 直线为 75 m 比赛段和 100 m 停止区 | 当前 YAML 为 start_pose 锚定、0/5/10 m、0.7 m/s 的调试配置 |
| 控制器 Trackdrive 起步固定 4 s | 当前规划器按 12 m 起步距离限速；控制配置已按赛项分组 |
| 任务机职责迁移已完成 | 目前仅雷达 ready 已改为心跳判断；计圈、地图质量和 RACE 定位门槛仍在任务机 |

## 保留的专题资料

- 配置：[实车模块配置加载逻辑](实车模块配置加载逻辑.md)、[算法参数审计](算法包可调整参数审计_2026-10-11.md)、[参数清单](算法参数清单.json)。
- 控制：[直线调试与简单速度控制](直线调试与简单速度控制.md)、[车辆控制标定接入](车辆控制标定接入.md)、[标定审计](标定数据审计_2026-10-11.md)。
- 仿真：[连接与验证](WUTA_SIM验证与启动.md)、[历史验证结果](WUTA_SIM最终验证结果_2026-10-11.md)。历史通过项仅对对应配置和覆盖范围成立。
- 演进：[任务机职责迁移说明](任务机职责迁移说明.md)为待实施方案，具体已完成程度见 [DESIGN.md](DESIGN.md)。
- 日志：[运行日志](LOGGING.md)，包含 `jrajectory/`、逐模块输出、赛项归档及已执行验证。历史日志路径保持原样。
- 实车入口：[README_FULL_FSD.md](../README_FULL_FSD.md)。其中历史运行记录与当前脚本策略需分别理解，例如当前启动优先复用已存在的同接口唯一CAN桥。

- 惯导：[临时分层诊断](惯导临时诊断记录.md)，区分字节输入、CRC、devpvt传递和GNSS过滤。
- 本次更新只涉及文档；角度标定脚本及对应专题暂不更新。
