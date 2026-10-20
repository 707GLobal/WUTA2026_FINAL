# 按功能定位代码


> 表内 FSD 路径均相对 `WUTA-FSD/ros2_ws/src/`，2026-10-13复核，同步10月12日工作树。

| 功能 | 核心实现 | 配置 / 验证入口 |
| --- | --- | --- |
| ROS 消息 | `common/wuta_msgs/msg`、`common/autoware_msgs/msg` | 各包 CMakeLists/package.xml；`WUTA-FSD/ros2_ws/test/test_cone_colors_ros.py` |
| LiDAR 锥桶检测 | `perception/lidar_detection/src/{lidar_detection_node,traditional_detector,dl_detector}.cpp` | `perception/lidar_detection/config/lidar_detection.yaml`；DL 为占位，使用传统后端 |
| 图像检测 | `perception/camera_detection/camera_detection/{yolo,yolo_node}.py`、`src/lwdetr_tensorrt_node.cpp` | `config/camera_detection.yaml`；`test/test_yolo.py` |
| 双目深度适配 | `perception/camera_detection/src/stereo_detection_adapter_cpp.cpp`、`camera_detection/{adapter,stereo}.py` | `config/stereo_adapter.yaml`；`test/test_stereo.py` |
| 相机雷达后融合 | `perception/detection_fusion/src/detection_fusion_node_cpp.cpp`、`detection_fusion/{node,core,calibration}.py` | `config/fusion.yaml`；`test/test_core.py`、`test/test_calibration.py` |
| 实车感知启动 | `perception/detection_fusion/launch/fusion_modules_hardware.launch.py` | 读取各模块 config；旧 `fusion_hardware.launch.py` 是另一入口，不能混用参数来源 |
| 锥桶建图 | `mapping/cone_map_builder/src/cone_map_builder.cpp`、`include/cone_map_builder/cone_map_builder.hpp` | `config/cone_map_builder.yaml` |
| 边界 / 中心线 | `planning/boundary_detector/src/boundary_detector_node.cpp`、`thirdparty/pathplanning` | `config/boundary_detector.yaml` |
| 任务路径 / 速度 / 轨迹记录 | `planning/path_generator/src/path_generator_node.cpp` | `config/path_generator.yaml`；记录到 `wuta_logs/<赛项>/<YYYYmmdd_HHMMSS>/trajectory/`，分段为 `trajectory/segments/<赛项>/<YYYYmmdd_HHMMSS>_<序号>/` |
| 横纵向控制 | `control/controller/src/{controller_node,pure_pursuit,twist_filter}.cpp`、`include/controller/speed_pid.hpp` | `config/controller.yaml`；`test/test_speed_pid.cpp` |
| 定位输出管理 | `localization/localization_manager/src/localization_manager.cpp` | `config/ekf.yaml`；不自主决定赛项定位模式 |
| EKF / 第三方定位 | `localization/robot_localization`、`localization/kiss-icp` | robot_localization 原生测试/参数；KISS 是否启动由入口决定 |
| KISS 增量过滤 | `localization/kiss_icp_wrapper/src/kiss_odom_sanitizer_node.cpp` | `config/kiss_icp_hesai128.yaml`；当前实车不启动 |
| NDT / 点云地图保存 | `localization/ndt_localization/src/{ndt_localization,map_saver}.cpp` | `config/ndt_localization.yaml`；map_saver 坐标变换仍有 TODO |
| 任务 / 自检 / 计圈 | `system/mission_manager/src/mission_manager.cpp` | `config/mission_manager.yaml`；门控诊断编码在 MissionState.description；车检不设GNSS门槛，设备自检保留 |
| CAN 任务输入 / 执行器输出 | `system/can_interface/src/{can_interface,can_socket}.cpp` | `config/can_interface.yaml`；协议在 `docs_for_can` |

## FSD 之外

惯导串口默认值与覆盖入口：根`start_full_fsd.sh`（INS_PORT/--ins-port）、CLI`fsd_startup/{checks,manager}.py`、华测`src/chcnav/config/chcnav_fsd_bridge.yaml`及`src/chcnav/{scripts/start_chcnav_fsd.sh,launch/chcnav_fsd_bridge.launch.py}`。默认绑定Prolific的by-id路径，避免ttyUSB重编号选错E22；具体设备见[DEPLOYMENT](DEPLOYMENT.md)。

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
| 显示桥 / 布局 | `fsd-visualization-code/src/fsd_visualization/fsd_visualization/{bridge,configuration}.py`；`fsd-visualization-code/src/fsd_visualization/config/{topics.yaml,fsd.rviz}` |
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

0.5.0必要日志规则位于`fsd_startup/log_context.py:necessary_radio_event`，原写入者生成`manager.radio.events.jsonl`；Manager状态字段`radio_log_events`定位当前源。完整性与筛选边界均在上述无线回归中验证。

新日志适配（0.5.1）：`radio_link/fsd_radio/cli_telemetry.py:CliForwarder.scan/acknowledge`负责新旧根目录与归档重定位；`fsd_startup/dashboard.py:component_log_files`为两个CLI界面提供统一日志列表，`runtime_checks.py`覆盖perception_drivers。回归为`radio_link/tests/test_cli_telemetry.py`、`fsd_startup/test_log_views.py`和`tools/test_sensor_log_routing.py`（Windows使用Git Bash，不使用WSL）。

## 无线0.6.0代码入口（2026-10-20）

- `fsd_startup/log_context.py:publish_radio_run/reset_radio_run/RunLog.radio_selection`：现有日志生命周期发布当前运行指针；必要日志筛选省略已由传感器快照覆盖的普通GNSS状态行，警告错误保留。
- `fsd_startup/manager.py`、`terminal_ui.py`、`dashboard.py`：发布radio_run，终端在scope切换时清理历史，图形实时日志读取当前运行日志。
- `fsd-visualization-code/radio_link/fsd_radio/cli_telemetry.py`：选定journal、跨归档路径续传、cli_run控制帧、PC当前stream接收与有界回放；不再发现或补传历史目录。
- 同目录 `link.py`、`configuration.py`、`ros_sender.py` 与 `radio.yaml`：3000 B/s总预算、空速/串口相关参数检查及50 ms发送端采集调度；`foxglove_view.py`处理当前scope显示会话切换。未执行本轮测试。

0.6.1：`radio_link/fsd_radio/foxglove_view.py:_create_channels/_reset_session`统一通道创建及会话切换后的通道重公告，避免使用关闭的缓存通道。

0.6.2：`fsd_startup/log_context.py:RunLog.radio_selection`与`manager.py:publish`指向manager.events.jsonl全文；`radio_link/fsd_radio/cli_telemetry.py:CliForwarder.scan`接收该完整源，并兼容旧必要journal名。必要事件规则仅用于旧子集产物，不再筛选新版无线流。

最终规划路径开关位于显示子模块src/fsd_visualization/config/fsd.rviz的Final planned path；源话题沿用topics.yaml，radio_link/radio.yaml的layers.path为1秒/权重3/4个Marker/每Marker48点，PC通道/radio/scene/path。
