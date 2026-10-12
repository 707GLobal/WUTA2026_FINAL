# 多人开发约定

> 本页是协作建议与交付流程，2026-10-12。实际用户任务及仓库适用的AGENTS指令优先。

## 1. 分工与改动边界

按感知、定位、建图、规划、控制、任务/CAN、部署/工具、可视化划分负责人。首次接手阅读 [架构](ARCHITECTURE.md)、[代码索引](CODE_MAP.md) 和 [接口](ROS_INTERFACE.md)。功能由一个模块生产，其他模块消费结论；跨模块行为由相关负责人共同确认接口。

共享高冲突文件包括消息定义、`start_full_fsd.sh`、实车感知launch、模块config、任务状态机和显示topics。提前说明要改哪些文件，避免不同开发者同时写同一配置。第三方定位源码优先通过参数和适配层接入，必要的源码修改说明原因。

工作树可能存在其他人的本地修改。先查看主仓库和目标子模块git status；保留无关修改，按文件提交自己的变化。WUTA-FSD/华测/ZLG是独立Git仓库：先在各子模块提交，再更新主仓库指针；仅提交父仓库不能保存子模块未提交内容。新克隆或更新子模块前先确认本地资产与修改。

## 2. 接口变更联动

| 改动 | 同步检查 |
| --- | --- |
| 消息字段、颜色或枚举 | 所有发布/订阅节点、Python/C++绑定、显示桥、仿真overlay、保存地图与rosbag兼容 |
| 话题名或QoS | 两端节点、remap、模块config、自检、监控、显示、仿真记录 |
| 坐标帧或外参 | TF唯一发布者、时间戳、华测桥、建图、融合投影及起步锚点 |
| 速度/转向约束 | 规划→控制→CAN，单位、符号、两处最大转角、标定插值与看门狗 |
| MissionState.description | 任务监控解析与兼容提示；不能把诊断格式当随意日志改写 |
| 圈次/地图质量职责迁移 | 发布者所有权、消费者、启动入口和断环方案 |
| 启动配置来源 | `check_fsd_module_configs.py`及其测试、配置清单、部署文档 |

修改消息后重建所有受影响消费者及外部模拟器overlay，停止旧实例后用新环境启动；避免旧install、新消息和旧rosbag混跑。修改参数尽量在负责模块源码config，不在launch临时叠加隐藏调参。

## 3. 验证阶梯

文档改动检查本地链接、路径和参数与源码一致；不需要启动硬件。配置改动先用 `./start_full_fsd.sh --print-config`，再运行相关既有回归。代码按目标包构建并测试：

```bash
source /opt/ros/humble/setup.bash
cd WUTA-FSD/ros2_ws
colcon build --base-paths src --symlink-install --packages-up-to controller --parallel-workers 2 --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon test --packages-select controller
colcon test-result --verbose
```

将controller替换为目标包；消息改动扩大至所有依赖包。部署端最终仍使用统一build-only完成全栈构建。包内 test、工具测试的环境需求不同，先读测试导入和脚本，不能把“有文件”当成“已运行”。

配置来源回归（需要Humble的launch和已构建FSD Python包环境）：

```bash
PYTHONPATH="$PWD/tools${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -m pytest tools/test_fsd_module_configs.py
```

部分测试依赖本地engine或标定文件，缺失时记录环境阻塞。控制逻辑有分赛项PID C++测试；历史 simple_p、CAN看门狗及标定表已从当前源码移除，不能把旧测试报告作为当前能力。感知有Python核心/标定/深度测试；颜色有ROS回归。涉及行为链再做隔离仿真；实车验证应由明确硬件任务安排，保留配置与日志。

日志和启动管理回归先加载 ROS/FSD 环境并关闭用户 Python 包路径污染：

```bash
source /opt/ros/humble/setup.bash
source WUTA-FSD/ros2_ws/install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest -q \
  fsd_startup/test_startup.py fsd_startup/test_logging.py fsd_startup/test_terminal_ui.py \
  fsd_startup/test_check_alignment.py tools/test_fsd_module_configs.py \
  tools/test_can_reconnect.py tools/test_privileged_group.py tools/test_chcnav_serial_guard.py
ROS_DOMAIN_ID=222 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest -q \
  tools/test_logging_ros.py WUTA-FSD/ros2_ws/test/test_cone_colors_ros.py \
  -k 'planner_switch or restarted_planner or mission_lidar or mission_quality'
```

隔离域命令只启动测试算法节点，不连接硬件；执行前确认域未被其他测试使用。
日志布局、验证边界和历史数据处理见 [LOGGING](LOGGING.md)。

## 4. 交付记录

提交或交接说明应包含：问题与最终行为、修改模块/文件、消息/参数兼容变化、执行的构建和测试及结果、未验证边界、后续必须补齐的环境/硬件条件。仿真保存summary和配置快照，不能只给“通过”两个字。

历史审计/验收文件保留日期；新的功能事实更新本套总览并链接专题证据，不覆盖旧失败记录。本套文档是工作树快照，源码变化后负责人同步更新相关页。
