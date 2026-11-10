# 按功能定位代码

2026-11-05测试路径地图：显示子模块`src/fsd_visualization/fsd_visualization/mission_review.py`接收最终Lane/标准pose并发布独立点线图层；同目录`mission_review_history.py`维护路径版本/定位及新旧JSONL回放，`mission_review_map.py`负责断线、独占采集目录和原子SVG导出。参数在该包`config/mission_review.yaml`，图层在`config/{fsd,mission_review}.rviz`，原坐标系/视角不变。根`start_full_fsd.sh`正常模式自动启动采集，输出集中到`test_maps/`，原CSV不变。验证入口为显示子模块`tests/test_mission_review{,_ros}.py`；[使用说明](../test_maps/README.md)。

> 当前临时状态（2026-11-01）：按用户要求，工控机直接发送给制动电机的全部路径已用`WUTA_DIRECT_BRAKE_PAUSED`标记注释停用，包括NMT 0x000、0x603初始化/参数/启动/正常停止/异常和析构发送，以及初始化请求、应答解析和推进。原实现完整保留供恢复；target=2按中性处理。当前台架为真实GO→仅通过0x210请求5%微驱动5秒（可配置0～5%，0恢复中性等待）→50%制动10秒→中性/FINISH，不再等待直接初始化或3秒间隔；0x301心跳和台架超时/按路由限幅/转向归中保护保留。此限制只约束FSD自身，VCU或其他车端节点仍可能自行向电机发送报文。下文直控、双路流程为停用方案记录，以本条为准。未启动硬件。

> 表内 FSD 路径均相对 `WUTA-FSD/ros2_ws/src/`，2026-10-13复核，同步10月12日工作树。

| 功能 | 核心实现 | 配置 / 验证入口 |
| --- | --- | --- |
| ROS 消息 | `common/wuta_msgs/msg`、`common/autoware_msgs/msg` | 各包 CMakeLists/package.xml；`WUTA-FSD/ros2_ws/test/test_cone_colors_ros.py` |
| LiDAR 锥桶检测 | `perception/lidar_detection/src/{lidar_detection_node,traditional_detector,dl_detector}.cpp` | `perception/lidar_detection/config/lidar_detection.yaml`；DL 为占位，使用传统后端；点云输入 Reliable/Volatile/KeepLast(1) |
| 图像检测 | `perception/camera_detection/camera_detection/{yolo,yolo_node}.py`、`src/lwdetr_tensorrt_node.cpp` | `config/camera_detection.yaml`；`test/test_yolo.py` |
| 双目深度适配 | `perception/camera_detection/src/stereo_detection_adapter_cpp.cpp`、`camera_detection/{adapter,stereo}.py` | `config/stereo_adapter.yaml`；`test/test_stereo.py` |
| 相机雷达后融合 | `perception/detection_fusion/src/detection_fusion_node_cpp.cpp`、`detection_fusion/{node,core,calibration}.py` | `config/fusion.yaml`；`test/test_core.py`、`test/test_calibration.py` |
| 实车感知启动 | `perception/detection_fusion/launch/fusion_modules_hardware.launch.py` | 读取各模块 config；旧 `fusion_hardware.launch.py` 是另一入口，不能混用参数来源 |
| 锥桶建图 | `mapping/cone_map_builder/src/cone_map_builder.cpp`、`include/cone_map_builder/cone_map_builder.hpp` | `config/cone_map_builder.yaml` |
| 边界 / 中心线 | `planning/boundary_detector/src/boundary_detector_node.cpp`（含processAccelerationMap）、`include/boundary_detector/acceleration_lines.hpp`（同包）、`thirdparty/pathplanning` | `config/boundary_detector.yaml` |
| 任务路径 / 速度 / 轨迹记录 | `planning/path_generator/src/path_generator_node.cpp` | `config/path_generator.yaml`；记录到 `wuta_logs/<赛项>/<YYYYmmdd_HHMMSS>/trajectory/`，分段为 `trajectory/segments/<赛项>/<YYYYmmdd_HHMMSS>_<序号>/` |
| 横纵向控制 | `control/controller/src/{controller_node,pure_pursuit,twist_filter}.cpp`、`include/controller/speed_pid.hpp` | `config/controller.yaml`；`test/test_speed_pid.cpp` |
| 定位输出管理 | `localization/localization_manager/src/localization_manager.cpp` | `config/ekf.yaml`；不自主决定赛项定位模式 |
| EKF / 第三方定位 | `localization/robot_localization`、`localization/kiss-icp` | robot_localization 原生测试/参数；KISS 是否启动由入口决定 |
| KISS 增量过滤 | `localization/kiss_icp_wrapper/src/kiss_odom_sanitizer_node.cpp` | `config/kiss_icp_hesai128.yaml`；当前实车不启动 |
| NDT / 点云地图保存 | `localization/ndt_localization/src/{ndt_localization,map_saver}.cpp` | `config/ndt_localization.yaml`；map_saver 坐标变换仍有 TODO |
| 任务 / 自检 / 计圈 | `system/mission_manager/src/mission_manager.cpp` | `config/mission_manager.yaml`；门控诊断编码在 MissionState.description；车检不设GNSS门槛，设备自检保留 |
| CAN 任务输入 / 执行器输出 | `system/can_interface/src/{can_interface,can_socket}.cpp` | `config/can_interface.yaml`；协议在 `docs_for_can`；GO转发另受0x211整车状态7→8及5 s保险约束 |

## FSD 之外

CLI与无线独立进程联启：`fsd_startup/manager.py:ensure_radio` → `fsd-visualization-code/radio_link/start_linux.sh`；终端适配 `fsd_startup/radio_window.sh`（GNOME窗口/tee日志，已有发送端用tail查看）；无线单实例锁在 `radio_link/fsd_radio/cli.py`，输出 `.runtime/fsd_radio/linux.log`。无线不登记到CLI车辆进程表，CLI退出后继续运行。

惯导串口默认值与覆盖入口：根`start_full_fsd.sh`（INS_PORT/--ins-port）、CLI`fsd_startup/{checks,manager}.py`、华测`src/chcnav/config/chcnav_fsd_bridge.yaml`及`src/chcnav/{scripts/start_chcnav_fsd.sh,launch/chcnav_fsd_bridge.launch.py}`。默认绑定Prolific的by-id路径，避免ttyUSB重编号选错E22；具体设备见[DEPLOYMENT](DEPLOYMENT.md)。

独立显示环境加载与启动依赖检查：`fsd-visualization-code/scripts/environment.sh`、`fsd-visualization-code/start.sh`；支持显式`FSD_SETUP`及本仓库相邻FSD安装环境；无线`radio_link/start_linux.sh`复用同一环境加载器，`setup_linux.sh`创建系统Python虚拟环境。

| 功能 | 入口 |
| --- | --- |
| 日志上下文 / 归档 | `fsd_startup/log_context.py`（`run.json`、`manager.events.jsonl`、`manager.radio.events.jsonl`）；`fsd_startup/mode_cache.py`；`tools/run_logged.py`；结构见 [LOGGING](LOGGING.md) |
| 任务诊断展示 | `tools/mission_diagnostics.py`；权威等待条件在 FSD mission_manager |
| 实车编排 / 进程清理 | [start_full_fsd.sh](../start_full_fsd.sh) |
| 纯键盘终端与自启 | 根 `start_fsd_manager.sh`；独立 [CLI子模块](https://github.com/starry1N/WUTA-FSD-CLI) 的 `fsd_startup/{manager,terminal_ui,dashboard,runtime_checks}.py`、`autostart.sh`；[开发指南](DEVELOPMENT.md#startup-development) |
| 模块配置 / 组件日志选择器 | `fsd_startup/terminal_ui.py`；配置消费审计 `manifest.json`，日志读取当前模块目录；键盘回归 `fsd_startup/test_terminal_ui.py` |
| GNSS状态含义 | `tools/gnss_status.py`；任务门控在mission_manager，CLI车辆/传感器页及运行检查消费权威诊断；车检显示不要求 |
| 惯导分层诊断 | `temporary_ins_diagnostics.hpp`与华测parser/protocol/bridge内计数；[诊断记录](惯导临时诊断记录.md) |
| CAN模式会话缓存 | `zlgcan_bridge/mode_cache.{c,h}`；`system/can_interface/src/mode_cache.cpp`及`mode_cache`配置 |
| 华测解析和 FSD 桥 | `humble-chcnav-cgi_ros2pkg/src/chcnav/src/chcnav_fsd_bridge/ChcnavFsdBridge.cpp`；`scripts/start_chcnav_fsd.sh`；`config/chcnav_fsd_bridge.yaml` |
| ZLG 桥 | `zlgcan_bridge/{main,can_side,zlg_side,can_log}.c`；`start_zlg_bridge.sh`、`bridge.yaml`、Makefile |
| 显示桥 / 布局 | `fsd-visualization-code/src/fsd_visualization/fsd_visualization/{bridge,configuration,diagnostics}.py`；`fsd-visualization-code/src/fsd_visualization/config/{topics.yaml,fsd.rviz}` |
| RViz固定诊断面板 / 融合坐标表 | `fsd-visualization-code/src/fsd_rviz_plugins/src/diagnostics_panel.cpp` | `fsd-visualization-code/tests/test_diagnostics.py`；`tests/gui_smoke.py` |
| 安全图像插件 | `fsd-visualization-code/src/fsd_rviz_plugins/src/safe_image.cpp` |
| E22双向无线显示 / Foxglove | `fsd-visualization-code/radio_link/fsd_radio/{ros_sender,geometry,protocol,link,transport,worker,pc_receiver,foxglove_view,cli}.py`；配置 `radio_link/radio.yaml`，部署与测试见 `radio_link/README.md` |
| 无线必要字段压缩 / 本地样式 | `fsd-visualization-code/radio_link/fsd_radio/essentials.py`；样式 `radio.yaml:foxglove.styles`，编码成本复现 `radio_link/tools/compare_bandwidth.py`，回归 `radio_link/tests/test_essentials.py` |
| 配置来源和话题一致性 | `tools/check_fsd_module_configs.py`、`tools/test_fsd_module_configs.py` |
| 参数枚举审计 | `tools/audit_algorithm_parameters.py`；生成清单见 docs 现有 JSON/CSV |
| 实车任务只读监控 | `tools/monitor_mission.py` |
| 提权进程监管 / CAN 重连 / 串口保护测试 | `tools/run_privileged_group.py`、`test_privileged_group.py`、`test_can_reconnect.py`、`test_chcnav_serial_guard.py` |
| 仿真组织与车辆适配 | `tools/run_wuta_sim_validation.py`、`tools/wuta_sim_effort_vehicle.py` |
| 仿真评估 / 汇总 / 几何复核 | `tools/sim_metrics.py`、`tools/summarize_wuta_sim.py`、`tools/review_sim_geometry.py` |
| 固定路径起步参考/Trackdrive旋转等价和惯导速度复核 | `tools/check_acceleration_start_pose.py`、`tools/check_chcnav_velocity_replay.py` |

修改前先查看目标包的 CMakeLists、package.xml、配置与 README，确认真实可执行名和是否安装。build/install 中的副本不是编辑入口。

## 独立显示仓库维护（2026-10-17）

显示源码位于[starry1N/fsd-visualization-code](https://github.com/starry1N/fsd-visualization-code)私有子模块，挂载路径仍为`fsd-visualization-code/`。显示环境加载与启动依赖检查在`scripts/environment.sh`、`start.sh`；无线安装/启动在`radio_link/{setup_linux,start_linux}.sh`，配置在`radio_link/radio.yaml`；路径均相对子模块。实际设备映射、USB拓扑、资源观察及共存验证边界见[DEVICE_COEXISTENCE](../fsd-visualization-code/docs/DEVICE_COEXISTENCE.md)。新克隆或更新执行`git submodule update --init --recursive`，需要私有仓库访问权限。

无线源时间对齐（0.3.0）：编码在`fsd-visualization-code/radio_link/fsd_radio/essentials.py`，采样来源在`ros_sender.py`和`geometry.py`，缓冲/SDK时间在`foxglove_view.py`；回归`radio_link/tests/test_source_time.py`，可导入布局`radio_link/foxglove_layout.json`。

## CLI无线日志与传感器（0.4.0）

CLI完整日志源：`fsd_startup/log_context.py:append_manager_event`、`fsd_startup/manager.py:emit`、`tools/run_logged.py`；传感器源：`fsd_startup/dashboard.py:Telemetry.snapshot`。无线转发/磁盘确认：`fsd-visualization-code/radio_link/fsd_radio/cli_telemetry.py`；回归`radio_link/tests/test_cli_telemetry.py`。

历史0.5.0必要日志规则位于`fsd_startup/log_context.py:necessary_radio_event`，原写入者生成`manager.radio.events.jsonl`；Manager状态字段`radio_log_events`定位当前源。相关回归源码保留；当前完整日志策略以0.6.2及之后章节为准，本次未运行回归。

新日志适配（0.5.1）：`radio_link/fsd_radio/cli_telemetry.py:CliForwarder.scan/acknowledge`负责新旧根目录与归档重定位；`fsd_startup/dashboard.py:component_log_files`为两个CLI界面提供统一日志列表，`runtime_checks.py`覆盖perception_drivers。回归为`radio_link/tests/test_cli_telemetry.py`、`fsd_startup/test_log_views.py`和`tools/test_sensor_log_routing.py`（Windows使用Git Bash，不使用WSL）。

## 无线0.6.0代码入口（2026-10-20）

- `fsd_startup/log_context.py:publish_radio_run/reset_radio_run/RunLog.radio_selection`：现有日志生命周期发布当前运行指针；必要日志筛选省略已由传感器快照覆盖的普通GNSS状态行，警告错误保留。
- `fsd_startup/manager.py`、`terminal_ui.py`、`dashboard.py`：发布radio_run，终端在scope切换时清理历史，图形实时日志读取当前运行日志。
- `fsd-visualization-code/radio_link/fsd_radio/cli_telemetry.py`：选定journal、跨归档路径续传、cli_run控制帧、PC当前stream接收与有界回放；不再发现或补传历史目录。
- 同目录 `link.py`、`configuration.py`、`ros_sender.py` 与 `radio.yaml`：3000 B/s总预算、空速/串口相关参数检查及50 ms发送端采集调度；`foxglove_view.py`处理当前scope显示会话切换。未执行本轮测试。

0.6.1：`radio_link/fsd_radio/foxglove_view.py:_create_channels/_reset_session`统一通道创建及会话切换后的通道重公告，避免使用关闭的缓存通道。

0.6.2：`fsd_startup/log_context.py:RunLog.radio_selection`与`manager.py:publish`指向manager.events.jsonl全文；`radio_link/fsd_radio/cli_telemetry.py:CliForwarder.scan`接收该完整源，并兼容旧必要journal名。必要事件规则仅用于旧子集产物，不再筛选新版无线流。

最终规划路径开关位于显示子模块src/fsd_visualization/config/fsd.rviz的Final planned path；源话题沿用topics.yaml，radio_link/radio.yaml的layers.path为1秒/权重3/4个Marker/每Marker48点，PC通道/radio/scene/path。

传感器原生面板：显示子模块radio_link/fsd_radio/foxglove_view.py:sensor_dashboard为PC格式化入口；radio_link/foxglove_layout.json使用Indicator/Gauge/Table并保留原始诊断页。数据仍来自cli_telemetry.expand_status，不新增采集者。

当前交付和目录索引见[DELIVERY.md](DELIVERY.md)：无线唯一布局源为radio_link/foxglove_layout.json，新鲜度与GNSS质量为background，接收年龄为bar；旧方案及工具临时文件不作为开发入口。

配置来源与原生Parameters接入见[FOXGLOVE_PARAMETERS.md](FOXGLOVE_PARAMETERS.md)。CLI原模块配置页读取预检查manifest，与根入口默认的11个模块YAML同源；新增E键编辑器和Parameters读取所选实际文件，支持额外驱动/外参配置。文件值不代表节点当前采用值。

## CLI统一参数与无线管理入口（0.7.0）

| 功能 | 入口与所有权 |
| --- | --- |
| 配置来源目录、原有交叉校验 | `tools/check_fsd_module_configs.py:configuration_files/module_files/manifest`；供审计与编辑共用 |
| CLI统一操作与持久结果 | `fsd_startup/control_api.py`；Manager.pump原线程调用Manager.operate，心跳不执行业务 |
| YAML版本/注释/备份/事务 | `fsd_startup/config_store.py`；CLI唯一写入者，目录定位依赖宿主tools |
| CLI本地配置编辑 | `fsd_startup/config_editor.py`，`terminal_ui.py` E键；同邮箱接口 |
| 无线RPC分块 | `fsd-visualization-code/radio_link/fsd_radio/rpc.py`；复用既有512字节扩展 |
| CLI邮箱与PC参数桥 | `radio_link/fsd_radio/cli_control.py`（相对显示子模块）；不另起进程管理器 |
| Parameters与操作按钮 | `foxglove_view.py` SDK回调、`pc_receiver.py` PC调度、`foxglove_layout.json`原生面板 |

完整使用、源时间、文件来源、运行生效边界见 [FOXGLOVE_PARAMETERS](FOXGLOVE_PARAMETERS.md)。

0.7.1调度：显示 `radio_link/fsd_radio/link.py:Outbox.commands/next_piece` 为独立指令FIFO；`rpc.py:CONTROL_OPERATIONS/send/accept/tick`传播优先级并保留容量；`cli_control.py`按操作分类请求/回复；`worker.py`与Endpoint清理断链/换会话的两个队列。CLI `control_api.py:pump`优先选择待处理指令。回归源码 `radio_link/tests/test_command_priority.py`，本轮未运行。

CLI状态指示：显示radio_link/fsd_radio/cli_control.py:PcControl.tick派生display_state，radio_link/foxglove_layout.json的Indicator!cliState消费，位于CLI控制页顶部；原phase/online仍保留在详情，不新增无线采集。

当前页面定位：radio_link/foxglove_layout.json的“CLI 日志与控制”引用Indicator!cliState及Publish!cli_check/start/stop/status；Table!sensorParams、RawMessages!cliSensors仅在“传感器概览”。独立“参数控制”只引用Parameters!cliConfig及Publish!cli_save/reload。此次仅布局修改。

PcControl.tick的busy/display_state是CLI操作中指示入口，Indicator!cliState显示橙色；使用原活动操作与本机管理请求，排除config.list/read。CLI业务执行与优先级不变。

RSSI入口：显示radio_link/fsd_radio/protocol.py:Framer的可选包尾字节状态，transport.py:rssi_status本机采集/新鲜度，worker.py按角色选择serial.append_rssi_pc/linux并输出/radio/link.rssi，configuration.py约束240字节分包；foxglove_layout.json的Indicator!radioRssi位于Table!taskState下方。原始单位，不假定dBm；test_rssi.py用例未运行。

Windows自动串口：显示radio_link/fsd_radio/ports.py:resolve_pc_port只枚举/匹配；worker.py在原重连循环选择实际端口；radio.yaml:serial.pc_port=auto与pc_usb配置；start_windows.ps1默认跟随配置，-Port仍手动覆盖。test_ports.py用例未运行。

参数操作状态：radio_link/fsd_radio/cli_control.py的parameter_note/parameter_state及config回复/超时分支；foxglove_layout.json的Indicator!parameterState与RawMessages!parameterStatus位于参数控制页，普通CLI结果不覆盖参数详情。

启动任务退出码指示：fsd_startup/manager.py:publish和control_api.py:status发布runtime_exit_code；显示cli_control.py:runtime_exit/EXIT_MEANINGS解释，LinuxControl转发心跳；Indicator!cliState包含退出码规则。exit_detail保留其他码，不推断具体硬件失败。

Windows Python自动发现：radio_link/windows_python.ps1:Find-RadioPython共用于setup_windows.ps1和start_windows.ps1，查找项目/已激活虚拟环境、Python Launcher、PATH及注册表，跳过商店占位程序，支持-Python覆盖。前者管理项目环境安装，后者只选择依赖可用环境并从源码目录启动。

Windows exe：radio_link/windows_receiver.py为PC入口/内置与外部配置选择/导出，复用fsd_radio.configuration.load_config和pc_receiver.run_pc；windows_receiver.spec收集Python依赖与配置资源，build_windows.ps1固定PyInstaller构建。configuration.py仅扩展冻结时渲染器资源根目录。成品本机output/FSD_E22_Receiver.exe，源码仓库不提交二进制或build/dist缓存。

## 静态制动台架（2026-10-21）

| 功能 | 现有实现内的扩展入口 |
| --- | --- |
| CLI M键循环选择/停止后恢复 | `fsd_startup/terminal_ui.py`、`manager.py:cycle_run_profile`；运行期间不可切换 |
| 台架依赖检查与配置编辑 | `fsd_startup/{checks,runtime_checks,config_store}.py`、`tools/check_fsd_module_configs.py:brake_bench_manifest` |
| 根启动与停机顺序 | `start_full_fsd.sh --brake-bench`，复用原日志和进程生命周期 |
| 5秒微驱动/10秒50%制动/中性释放 | `control/controller/src/controller_node.cpp:runBrakeBench`、`config/controller.yaml:acceleration.bench_brake`（FSD源码相对路径） |
| 选档/GO与传感器豁免 | `system/mission_manager/src/mission_manager.cpp:advanceWhenReady/selfCheckTick`；`common/wuta_msgs/msg/MissionState.msg:brake_bench_mode` |
| 台架命令超时与中性帧 | `system/can_interface/src/can_interface.cpp:sendControlFrame`及析构；真实CAN协议出口保持 |
| 离线验证 | `fsd_startup/test_brake_bench.py`、`tools/test_brake_bench_ros.py`、FSD `system/can_interface/test/test_brake_bench_can.cpp` |

## 人工驾驶记录（2026-10-22）

| 功能 | 原有实现内的扩展入口 |
| --- | --- |
| CLI M键三模式循环、互斥/停止锁、模式显示 | `fsd_startup/manager.py:cycle_run_profile/options_from`、`terminal_ui.py` |
| 正常算法链与只收CAN启动 | `start_full_fsd.sh --manual-record`；预检查 `fsd_startup/checks.py:manual_record_occupancy/can_occupancy`，运行检查 `runtime_checks.py` |
| 人工记录目录与原日志生命周期 | `fsd_startup/log_context.py:RunLog.create`、`manager.py:start`、`tools/run_logged.py`、`tools/monitor_mission.py` |
| 只收桥与设备发送保护 | `zlgcan_bridge/start_zlg_bridge.sh --receive-only`、`main.c`、`zlg_side.c`；VCU日志仍由 `can_log.c` 写入 |
| CAN接收及所有发送禁用 | FSD `system/can_interface/src/can_interface.cpp:receive_only`（FSD路径相对ros2_ws/src） |
| 自动任务不激活、诊断保留 | FSD `system/mission_manager/src/mission_manager.cpp:manual_record_mode`、`tools/mission_diagnostics.py` |
| 原实际轨迹记录（未另建实现） | FSD `planning/path_generator/src/path_generator_node.cpp:onPose/appendDrivenRecord` |
| 无硬件回归 | `fsd_startup/test_manual_record.py`、`zlgcan_bridge/tests/test_receive_only.py`、FSD `system/can_interface/test/test_receive_only_can.cpp`、`tools/test_manual_record_ros.py` |

2026-10-22台架前置0x603：FSD `system/can_interface/src/can_interface.cpp:prepareBrakeBench/parseResFrame`负责A/B交替三组及GO转发顺序；`test/test_brake_bench_prepare.cpp`为内存设备离线验证，覆盖6帧顺序/字节、GO后置、重复GO、未就绪/自检缺失、写失败急停和正常模式无额外发送。

2026-10-23早期临时状态（已由下述延时后初始化方案替代）：`can_interface.cpp:parseResFrame`中`prepareBrakeBench()`调用已临时注释；函数和历史测试源码保留，`brake_bench_prepare` CTest标记DISABLED。当前台架只走原GO→任务机→controller定时→0x210控制链，原0x301心跳保持。

Foxglove M键适配（0.7.10）：fsd_startup/manager.py:operate(cycle_mode)/cycle_run_profile/publish；control_api.py:execute/status及优先队列。显示radio_link/fsd_radio/cli_control.py转发模式字段、发送expected_mode并派生display_state_with_mode，rpc.py:CONTROL_OPERATIONS加入cli.cycle_mode。foxglove_layout.json的Publish!cli_cycle_mode位于原检查/启动按钮行，原Indicator!cliState消费含模式字段。

历史方案（2026-10-23，当前停用）——延时后初始化：`control/controller/src/controller_node.cpp:runBrakeBench`等待后发布初始化请求并在成功结果后开始保持计时；`system/can_interface/src/can_interface.cpp:startBrakeBenchInitialization/tickBrakeBenchInitialization/brakeBenchInitFrame/parseBrakeBenchSdo`负责NMT、顺序SDO和失败急停，`sendControlFrame`增加初始化成功门控。该历史版本曾使用`config/can_interface.yaml:brake_bench.initialization`；当前配置不提供此项；时序和开度仍由controller.yaml管理。旧prepareBrakeBench实现已替换，历史六帧测试仍DISABLED。

历史方案（当前停用）——直控：controller.yaml `acceleration.bench_brake.post_init_delay_sec=3.0`为新增初始化后等待参数；`runBrakeBench`恢复握手并计算额外等待，`can_interface.cpp:updateBrakeBenchMotor`将Command负开度编码为6071千分比，复用`brakeBenchInitFrame/tickBrakeBenchInitialization/parseBrakeBenchSdo`处理制动/释放SDO；`stopBrakeBenchMotorBestEffort`负责失败和析构清零/暂停。`sendControlFrame`在台架始终发送中性0x210。此句描述历史版本；当前已恢复WUTA_DIRECT_BRAKE_PAUSED编译排除。

保留源码映射（当前编译排除）：can_interface.cpp的brakeBenchInitFrame固定steps[0..6]为七条配置SDO、steps[7]为唯一EF启动；startBrakeBenchInitialization先发01 00/DLC2，parseBrakeBenchSdo在第七条SDO成功后回报配置完成，controller等待3秒后由updateBrakeBenchMotor只触发steps[7]。steps[8..9]仅供结束释放。tickBrakeBenchInitialization按模式帧后2ms、其余20ms的最小间隔和成功应答推进；取消参数覆盖及模式/状态回读。

历史方案（当前停用）——双路：controller.cpp的runBrakeBench先发target=1/负Command保持10秒，再target=0请求配置并等inter_stage_delay_sec，最后target=2/负Command保持10秒；publishBenchCommand同时刷新目标选择。can_interface.cpp的target订阅在切换时清缓存，sendControlFrame仅target=1转发VCU负开度、仅target=2调用电机启动。配置超时及急停出口沿用，初始化字节不变。

可恢复禁用惯导：根`start_full_fsd.sh`的`DISABLE_INS`与`--no-ins/--with-ins`；CLI`manager.py:options_from/permissions`、`checks.py:serial/dependencies`、`runtime_checks.py`同步跳过惯导/定位首帧检查；根入口`MISSION_INS_ARGS`仅在禁用时覆盖任务机`check_imu=false`，恢复读取配置值；恢复命令见[DEPLOYMENT](DEPLOYMENT.md#临时屏蔽惯导与恢复2026-10-25)。

## 感知 Reliable 实现与验证（2026-10-25）

| 入口 | 实现位置 |
| --- | --- |
| 雷达检测输入 | FSD `perception/lidar_detection/src/lidar_detection_node.cpp` |
| 相机输入与标注图 | FSD `perception/camera_detection/camera_detection/{yolo_node,adapter}.py`、`src/{stereo_detection_adapter_cpp,lwdetr_tensorrt_node}.cpp` |
| 融合输入 | FSD `perception/detection_fusion/src/detection_fusion_node_cpp.cpp`、`detection_fusion/node.py` |
| 任务机雷达/相机心跳 | FSD `system/mission_manager/src/mission_manager.cpp` |
| CLI首帧检查与遥测 | 根 `start_full_fsd.sh:wait_topic`、`fsd_startup/dashboard.py:telemetry_qos` |
| RViz及显示桥 | `fsd-visualization-code/src/fsd_visualization/config/fsd.rviz`、`fsd_visualization/{configuration,bridge}.py`；根 `config/full_fsd.rviz` |
| 离线验证 | 根 `tools/test_sensor_log_routing.py`、`tools/test_camera_reliable_ros.py`；CLI `test_startup.py`；显示 `tests/test_interfaces.py::test_configuration_profiles` |

隔离ROS相机回归覆盖Python/C++深度适配、曝光时间保留、三维估计及YOLO输入/标注图Reliable连接。QoS深度与兼容边界见[ROS_INTERFACE](ROS_INTERFACE.md#感知链路-qos2026-10-25)。

macOS产物：显示.github/workflows/macos-receiver.yml（workflow_dispatch、arm64/x86_64矩阵、Artifacts与可选Release）；radio_link/build_macos.sh、start_macos.command、MACOS_README.md；共享windows_receiver.spec按平台收集串口后端。打包不执行应用或硬件测试，构建信息随包附带。

## 独立任务复盘（2026-10-31）

- 显示子模块`review.sh`：正常模式由根入口自动启动的只读采集入口，也可独立启动；live传入已有运行目录，replay传入保存的JSONL，不依赖自检RViz或启动车辆节点。
- `src/fsd_visualization/fsd_visualization/mission_review.py`（相对显示子模块）：现有ROS输入、采样TF、最终Marker快照及专用RViz进程。
- 同目录`mission_review_history.py`：目标点/标准pose/原始惯导位置/规划版本的观察分段与增量journal；急停滑行、断流未确认、JSONL恢复。
- `src/fsd_visualization/config/mission_review.{yaml,rviz}`：专用输入/收尾设置及定位、惯导、当前/历史规划、目标与状态图层。
- `tests/test_mission_review{,_ros}.py`：纯记录及隔离ROS消息/TF/晚订阅/回放验证；不操作硬件。

2026-11-01临时停用直发：controller.cpp初始化接口注册及runBrakeBench的直接电机阶段、can_interface.cpp初始化接口/协议实现/推进/析构停止均按WUTA_DIRECT_BRAKE_PAUSED编译排除，配套头文件声明和成员保留在同名注释块中。target订阅当前接受1和3，0/停用的2/未知值中性；恢复须同步恢复目标2及原门控，勿仅取消帧表注释。现行内存CAN测试断言整个生命周期不发送0x000/0x603。

2026-11-01制动前微驱动：FSD controller的`runBrakeBench/publishBenchCommand`使用带符号开度，源码controller.yaml的`acceleration.bench_brake.pre_drive_opening`控制原delay_sec阶段；can_interface.cpp的target订阅及sendControlFrame仅为目标3放行0～5%正开度。配置检查在根tools/check_fsd_module_configs.py；回归为tools/test_fsd_module_configs.py、tools/test_brake_bench_ros.py与CAN内存设备brake_bench_can。直接电机停用标记保持。

2026-11-09发车保险：can_interface.cpp新增`parseVcuStateFrame/vcuGoGateOpen`接收0x211 Byte1整车状态；仅在7→8跳变并满`go_gate.wait_sec`（默认5 s）后转发0x1E4 GO，此前GO直接丢弃；启动时已在8则从首次收到起保守计时，`go_gate.stale_sec`控制断流关门，`go_gate.enabled`供无0x211台架关闭。不新增ROS话题，mission_manager代码不变；内存CAN回归为`test_go_gate_can`。

复盘已知限制与2026-11-05离线验证结果统一见[test_maps说明](../test_maps/README.md#已知限制2026-11-05核对)；当前提交保留这两个已复现问题，尚未修复。
