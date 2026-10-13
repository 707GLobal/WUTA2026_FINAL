# 多人开发约定

> 本页是协作建议与交付流程，2026-10-12。实际用户任务及仓库适用的AGENTS指令优先。

## 1. 分工与改动边界

按感知、定位、建图、规划、控制、任务/CAN、部署/工具、可视化划分负责人。开始每个任务前先读 [架构](ARCHITECTURE.md) 和 [代码索引](CODE_MAP.md)，接口/跨模块改动补读 [接口](ROS_INTERFACE.md)。依据架构中的“模块—文件—功能”和修改归属定位已有实现，先明确生产者、消费者与业务层，再扩展对应模块。功能由一个模块生产，其他模块消费结论；跨模块行为由相关负责人共同确认接口。

避免新增同义节点、算法、状态机、配置入口或第二个权威发布者。新模块先说明现有实现为何不足、独有职责与接口；共享需求优先复用/提取现有核心。当前正式计圈、地图质量和RACE定位门槛仍在任务机，不能依据待迁移方案在下游重复实现。根 [AGENTS.md](../AGENTS.md)和 [AGENT_GUIDE](AGENT_GUIDE.md)采用同一阅读与权责约定。

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

<a id="startup-development"></a>

## 5. 启动管理包开发指南

核对日期：2026-10-13。启动包目录实际名称为 `fsd_startup`。本页面向修改启动管理工具的开发者；日常操作见 [README](../fsd_startup/README.md)，系统职责见 [ARCHITECTURE](ARCHITECTURE.md)，项目接手流程见 [AGENT_GUIDE](AGENT_GUIDE.md)。

### 5.1 职责与入口

本包属于部署工具层，复用现有实车启动入口，不实现驾驶算法或任务状态机。

```text
start_fsd_manager.sh
  └─ manager.py：参数、单实例锁、操作会话与运行监管
       ├─ terminal_ui.py：Textual/Rich 纯键盘前端
       ├─ checks.py：独立预检查
       ├─ start_full_fsd.sh：实际硬件、TF和算法节点编排
       ├─ runtime_checks.py：消费根脚本进度，形成检查表
       ├─ window.sh → dashboard.py：独立系统Python只读ROS采集
       └─ processes.py / privilege.py / log_context.py：清理、提权、日志生命周期
```

`manager.py` 默认使用单终端 TUI；`--plain` 使用保留的逐行菜单和 Tk 窗口。`autostart.sh` 经 GNOME 终端调用同一个管理入口。真实节点启动、依赖顺序和首帧判定归根 `start_full_fsd.sh`，不要在前端再复制启动链。

### 5.2 文件定位

| 文件 | 修改归属 |
| --- | --- |
| [terminal_ui.py](../fsd_startup/terminal_ui.py) | 组件布局、焦点、快捷键、Mocha主题、检查表和日志呈现 |
| [manager.py](../fsd_startup/manager.py) | 参数解析、操作队列的执行目标、运行进程、状态快照和输出收集 |
| [checks.py](../fsd_startup/checks.py) | 预检查目录、部署文件、设备权限/网络/占用与产物；配置来源复用根工具 |
| [runtime_checks.py](../fsd_startup/runtime_checks.py) | 根启动脚本文本事件到展示状态的映射；不自行判任务ready |
| [dashboard.py](../fsd_startup/dashboard.py) | ROS只读订阅、消息摘要、接收频率、末帧间隔、GNSS与任务诊断呈现 |
| [processes.py](../fsd_startup/processes.py) | Linux PID启动身份、后代追踪和本包登记进程的信号清理 |
| [privilege.py](../fsd_startup/privilege.py) | sudo命令和本地askpass环境适配；本地密码文件不提交、不写日志 |
| [log_context.py](../fsd_startup/log_context.py) | RunLog唯一目录、配置快照、run.json及停止后的赛项归档 |
| [window.sh](../fsd_startup/window.sh) | 加载ROS overlay，再用系统Python运行采集或保留的Tk显示 |
| [install_ui.sh](../fsd_startup/install_ui.sh)、[requirements-ui.txt](../fsd_startup/requirements-ui.txt) | 独立UI依赖安装及固定版本 |
| [autostart.sh](../fsd_startup/autostart.sh) | 桌面会话延迟启动、打开终端、避免重复窗口 |

### 5.3 键盘与布局

当前 TUI 没有按钮、鼠标操作或底部命令输入框，也没有数字页面快捷键。

| 操作 | 按键 |
| --- | --- |
| 焦点向前/向后移动 | Tab / Shift-Tab |
| 页面菜单选项、检查行选择 | ↑ / ↓ |
| 打开页面、展开检查详情 | Enter |
| 并行预检查、启动、权限处理 | C / S / P |
| 停止运行，保留界面 | X / Ctrl-C |
| 清理所属进程并退出 | Q / Ctrl-Q |
| 暂停/恢复实时日志刷新 | Space |

左侧 `OptionList` 负责选择页面，右侧使用 `DataTable`、`Select` 和 `RichLog` 展示内容。底部 `Static` 仅显示快捷键提示，不可输入。页面目录在 `VIEWS` 中维护，操作绑定在 `FsdApp.BINDINGS` 中维护。

窄终端缩小左侧菜单宽度，菜单始终保留；没有命令输入框时不能隐藏唯一页面选择入口。焦点由 Textual 的原有 Tab 遍历管理，隐藏的检查表或组件选择器不参与当前页面操作。

### 5.4 线程与数据流

`FsdApp.manager_loop()` 在一个后台线程内串行执行 `Manager.command()` 和 `Manager.pump()`；它是管理器操作的执行者。启动、停止、权限处理和日志收尾不可在另一个线程同时修改同一运行。

UI动作只向 `commands` 队列提交操作。预检查内部通过线程池并行执行独立项目；耗时操作期间UI仍能刷新。停止和退出排队等待当前操作，不强行打断正在进行的资源变更。

`Manager.publish()` 原子写入 `state.json`，并向 `ui_events` 提交状态快照。`emit()` 写真实管理日志并提交显示事件。前端每0.2秒读取队列，更新检查表和日志；不要从管理线程直接调用Textual控件。

后台ROS采集在单独的系统Python进程中运行，使用 `window.sh --telemetry-json` 模式。它每0.5秒替换 `.runtime/fsd_manager/telemetry.json`；前端只读这个缓存。缓存不是历史日志，不能当成可回放记录。

### 5.5 检查与业务边界

预检查入口是 `checks.CHECKS`；运行检查入口是 `RuntimeChecks.rows/consume()`。两者阶段不同：文件存在、进程启动、收到首帧、持续在线、任务READY和真实发车不能合并成一个“成功”。

新增检查时先定位根入口已有规则，核对模式与配置覆盖：`sensor-only`、`reuse-ins`、`no-rviz`、`build-only`及后出现的build/RViz开关。独立项目可并行，但定位、规划控制等依赖关系仍由根脚本推进。

如果修改根脚本的进度文案，同步检查 `RuntimeChecks.consume()` 的匹配规则和 [test_check_alignment.py](../fsd_startup/test_check_alignment.py)。避免解析不到事件后仍展示“通过”。原脚本未执行的验证应保持待验证或跳过，并显示原因。

车辆页消费真实 `MissionState`、设备自检和CAN输入事件；GNSS含义与任务诊断复用根工具。界面不能发布模式、GO、急停解除或Command，也不能用订阅频率自行决定任务是否放行。权威生产者和单位契约见 [ROS_INTERFACE](ROS_INTERFACE.md)。

外部惯导和符合条件的外部CAN桥按已有规则复用。停止本次运行只清理登记且启动身份匹配的所属进程，不能全局 `pkill` ROS或删除外部桥。

### 5.6 主题修改

当前主题为 Catppuccin Mocha，主题名 `fsd-mocha`。颜色有三个入口：

- `terminal_ui.py` 的 `STYLES`：日志和状态单元格。
- `FsdApp.CSS` 与注册的 `Theme`：背景层次、正文、焦点、边框和滚动条。
- `manager.py` 的 `COLORS`：保留的逐行CLI ANSI色值。

Base `#1e1e2e`、Mantle `#181825`、Crust `#11111b`；正文 `#cdd6f4`，辅助文字 `#a6adc8`。强调色 Mauve `#cba6f7`、Blue `#89b4fa`、Teal `#94e2d5`；成功 `#a6e3a1`、警告 `#f9e2af`、错误 `#f38ba8`。

纯配色修改只触及上述入口，不改组件尺寸、事件或门控。继续保留 `--no-color` 的用户偏好。

### 5.7 依赖与离线验证

UI依赖固定在 `requirements-ui.txt`，安装目标为 `.runtime/fsd_manager/ui_deps`。管理器将该目录加入自己的 `sys.path`，不将它加入ROS子进程的 `PYTHONPATH`，避免UI升级影响驱动和生成消息绑定。

从仓库根目录执行：

```bash
bash fsd_startup/install_ui.sh
/usr/bin/python3 -m pytest -q fsd_startup/test_terminal_ui.py
/usr/bin/python3 -m pytest -q fsd_startup/test_startup.py fsd_startup/test_check_alignment.py fsd_startup/test_logging.py
bash -n start_fsd_manager.sh fsd_startup/autostart.sh fsd_startup/window.sh fsd_startup/install_ui.sh
```

| 测试 | 验证重点 |
| --- | --- |
| `test_terminal_ui.py` | 无输入框/按钮，Tab焦点、方向键页面选择、完整错误、窄终端、键盘退出 |
| `test_startup.py` | 并行错误收集、权限适配、失败保留操作、PID身份与清理 |
| `test_check_alignment.py` | 检查项目与根入口一致、参数模式、首帧和进程状态分离 |
| `test_logging.py` | 日志目录、配置快照、输出收集、归档和故障恢复 |

界面测试使用 Textual `run_test()`、临时目录、模拟检查和关闭遥测，不启动实车。不要用普通启动命令验证UI：默认启动会连接硬件；即使 `--menu`，实际管理器也可能恢复并清理先前登记的残留。

主题或布局可在测试会话中调用 `save_screenshot()` 导出真实控件SVG，再检查焦点和小窗口布局。不为了截图启动CAN或驾驶节点。

### 5.8 日志与交付维护

运行记录归 `RunLog` 管理；`manager.log`、`modules/`、`configs/`、`artifacts/`及 `jrajectory/` 沿用 [LOGGING](LOGGING.md) 的结构。任务诊断输出由管理器收集，UI不另建运行目录或第二套归档机制。

修改前查看主仓库和相关子模块状态，保留已有无关修改。改当前源码，不改build/install副本。改UI操作同步README和相关测试；改文件定位或职责同步ARCHITECTURE/CODE_MAP；改变真正的ROS接口才修改ROS_INTERFACE。
