# 按功能定位代码

> 表内 FSD 路径均相对 `WUTA-FSD/ros2_ws/src/`，2026-10-12 工作树快照。

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
| 任务路径 / 速度 / 轨迹记录 | `planning/path_generator/src/path_generator_node.cpp` | `config/path_generator.yaml`；记录到 `logs/<赛项>/<运行时间>/jrajectory/` |
| 横纵向控制 | `control/controller/src/{controller_node,pure_pursuit,twist_filter}.cpp`、`include/controller/speed_pid.hpp` | `config/controller.yaml`；`test/test_speed_pid.cpp` |
| 定位输出管理 | `localization/localization_manager/src/localization_manager.cpp` | `config/ekf.yaml`；不自主决定赛项定位模式 |
| EKF / 第三方定位 | `localization/robot_localization`、`localization/kiss-icp` | robot_localization 原生测试/参数；KISS 是否启动由入口决定 |
| KISS 增量过滤 | `localization/kiss_icp_wrapper/src/kiss_odom_sanitizer_node.cpp` | `config/kiss_icp_hesai128.yaml`；当前实车不启动 |
| NDT / 点云地图保存 | `localization/ndt_localization/src/{ndt_localization,map_saver}.cpp` | `config/ndt_localization.yaml`；map_saver 坐标变换仍有 TODO |
| 任务 / 自检 / 计圈 | `system/mission_manager/src/mission_manager.cpp` | `config/mission_manager.yaml`；门控诊断编码在 MissionState.description |
| CAN 任务输入 / 执行器输出 | `system/can_interface/src/{can_interface,can_socket}.cpp` | `config/can_interface.yaml`；协议在 `docs_for_can` |

## FSD 之外

| 功能 | 入口 |
| --- | --- |
| 日志上下文 / 归档 | `fsd_startup/log_context.py`；`tools/run_logged.py`；说明见 [LOGGING](LOGGING.md) |
| 任务诊断展示 | `tools/mission_diagnostics.py`；权威等待条件在 FSD mission_manager |
| 实车编排 / 进程清理 | [start_full_fsd.sh](../start_full_fsd.sh) |
| 华测解析和 FSD 桥 | `humble-chcnav-cgi_ros2pkg/src/chcnav/src/chcnav_fsd_bridge/ChcnavFsdBridge.cpp`；`scripts/start_chcnav_fsd.sh`；`config/chcnav_fsd_bridge.yaml` |
| ZLG 桥 | `zlgcan_bridge/{main,can_side,zlg_side,can_log}.c`；`start_zlg_bridge.sh`、`bridge.yaml`、Makefile |
| 显示桥 / 布局 | `fsd-visualization-code/src/fsd_visualization/fsd_visualization/{bridge,configuration}.py`；`fsd-visualization-code/src/fsd_visualization/config/{topics.yaml,fsd.rviz}` |
| 安全图像插件 | `fsd-visualization-code/src/fsd_rviz_plugins/src/safe_image.cpp` |
| 配置来源和话题一致性 | `tools/check_fsd_module_configs.py`、`tools/test_fsd_module_configs.py` |
| 参数枚举审计 | `tools/audit_algorithm_parameters.py`；生成清单见 docs 现有 JSON/CSV |
| 实车任务只读监控 | `tools/monitor_mission.py` |
| 提权进程监管 / CAN 重连 / 串口保护测试 | `tools/run_privileged_group.py`、`test_privileged_group.py`、`test_can_reconnect.py`、`test_chcnav_serial_guard.py` |
| 仿真组织与车辆适配 | `tools/run_wuta_sim_validation.py`、`tools/wuta_sim_effort_vehicle.py` |
| 仿真评估 / 汇总 / 几何复核 | `tools/sim_metrics.py`、`tools/summarize_wuta_sim.py`、`tools/review_sim_geometry.py` |
| 起步参考和惯导速度复核 | `tools/check_acceleration_start_pose.py`、`tools/check_chcnav_velocity_replay.py` |

修改前先查看目标包的 CMakeLists、package.xml、配置与 README，确认真实可执行名和是否安装。build/install 中的副本不是编辑入口。
