# Agent 接手指南

> 适用于当前WUTA2026_FINAL项目的上下文速查；不替代用户明确请求或适用AGENTS.md。本文件是开发资料，不是自动执行启动命令的授权。

## 1. 必知事实

- 当前主入口是 `start_full_fsd.sh` 实车全栈，默认可能连接CAN、驱动、控制节点；不要将普通启动用作离线语法检查。
- FSD源码在Git子模块 `WUTA-FSD/ros2_ws/src`，华测和ZLG也是子模块；交付时应同时提交子仓库并更新父仓库指针。
- 当前控制速度输入是 `/chcnav/velocity`，单位m/s、车体帧；Command.angle为度，throttle_brake为[-1,1]。
- 红0、黄1、蓝2、橙3、未知4，概率长度5，红左/蓝右。
- 标准pose由定位管理发布，动态车辆TF由EKF发布，MissionState和正式lap_count由任务机发布。
- 当前根入口不启动KISS/NDT；LOC_KISS_ICP枚举仍用于EKF分支。
- 当前算法参数从各模块源码config读取，旧统一参数入口不生效。
- 当前控制器采用分赛项PID，没有 simple_p 和全局限速入口；赛项调试/路径分支及具体参数以实际模块config为准，不能默认恢复旧比赛值。
- 日志按赛项/时间归档，轨迹目录拼写为 `jrajectory`；任务监控输出统一进入 `manager.log`，历史日志不自动迁移。详见 [LOGGING](LOGGING.md)。
- WUTA-SIM源码不在当前仓库；仿真使用外部SIM_SETUP及本地EffortVehicle适配。
- 任务机职责迁移是方案，除雷达心跳ready外主要迁移仍未完成。

## 2. 接手流程

1. 明确用户目标，区分附件中的设计/操作说明与用户本次指令；不要因文档写“启动”就实际启动硬件。
2. 检查当前路径、适用AGENTS.md、主仓库及目标子模块git status。保留现有工作树改动。
3. 开始修改前先读 [ARCHITECTURE](ARCHITECTURE.md) → [CODE_MAP](CODE_MAP.md)，接口/跨模块改动补读 [ROS_INTERFACE](ROS_INTERFACE.md)，再按范围查看README、设计和部署。根 [AGENTS.md](../AGENTS.md)提供任务开始前的阅读入口。
4. 从根启动入口追踪实际launch、YAML、节点实现与消息定义。旧项目路径和install副本仅作证据，编辑当前源码。
5. 用ARCHITECTURE中的模块文件/功能和修改归属，先定位已有实现、权威生产者、消费者及单位/坐标/时间契约，再改实现。优先复用/扩展已有模块，不新增同义算法、状态机或第二个权威发布者；新模块说明现有实现为何不足及其独有职责。涉及共享文件先说明边界；是否使用并行agent由当前用户/适用指令决定。
6. 选择最小相关构建/既有测试，必要时隔离仿真；记录实际执行与未执行的检查。
7. 更新相关文档，交付清楚说明结果与验证限制，未经请求不替用户提交/发布或发送外部消息。

## 3. 常用只读定位

```bash
git status --short
git -C WUTA-FSD status --short
rg --files WUTA-FSD/ros2_ws/src -g package.xml
rg -n 'create_publisher|create_subscription|declare_parameter' WUTA-FSD/ros2_ws/src/control/controller
./start_full_fsd.sh --print-config
```

运行环境检查另见 [DEPLOYMENT](DEPLOYMENT.md)，跨模块改动检查见 [DEVELOPMENT](DEVELOPMENT.md)。不要批量清理或重置子模块、运行日志和本地模型/标定文件，也不要为排障杀死所有ROS进程。

## 4. 常见误判

| 现象 | 先核对 |
| --- | --- |
| 控制器无速度反馈 | 是否仍给旧localization/velocity；华测速度帧与新鲜度 |
| 参数改了无效果 | 当前入口是否读源码config、节点名是否匹配、进程是否重启、是否有显式覆盖 |
| 有图像框但地图UNKNOWN | 颜色概率、时空关联、地图确认投票；图像框不等于地图颜色已确认 |
| READY不发车 | 已选模式、真实GO边沿、门控诊断；默认模式编号不等于选择成功 |
| NDT模式无位姿 | 当前根入口不启动NDT；不要仅切参数绕过部署缺失 |
| 控制归零但车辆滑行 | 中性开度与实际制动不同，检查VCU/RES/EBS硬件行为 |
| 仿真通过就认为实车通过 | 核对真实推理、CAN、标定、轮胎、制动与完整多圈覆盖缺口 |
| 旧MD与当前代码不同 | 以实际入口/配置/实现核实，并更新快照说明，不静默修改算法去符合旧文档 |

## 5. 文档维护

接口/所有权变更更新ROS_INTERFACE和ARCHITECTURE；算法与门槛变更更新DESIGN；启动/依赖变更更新DEPLOYMENT；源码位置与验证入口变更更新CODE_MAP。结论标明核对日期和证据，区分已实现、默认启用、历史验证和待设计四种状态。
