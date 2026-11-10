# 当前核心 ROS 与 CAN 接口

## 测试路径地图增量接口（2026-11-05）

现有fsd_mission_review新增订阅`/localization/pose`（geometry_msgs/PoseStamped，Reliable/Volatile/depth1000），其他驾驶话题/生产者保持不变。观察历史schema 2增加pose和plan_invalidated事件，schema 1仍可回放。标准pose和原始惯导独立保存，不以任何一路替代另一条权威定位出口。

新增`/mission_review/{localization,current,planned_history}`，类型均为visualization_msgs/MarkerArray，Reliable/Transient Local/depth1，分别是标准定位轨迹、当前位置和历史路径版本。原`/mission_review/planned`显示当前有效最终路径的点线，空Lane用DELETEALL清除当前图层、历史保留。全部Marker仍在map帧，按采样TF转换后存储；历史显示时间为零，原RViz Fixed Frame和视角不变，不生产TF。显示默认50ms，接收样本不按此频率抽样；断流/跳变/时间回退只断开显示连线。输出文件及查看方法见[test_maps](../test_maps/README.md)。

> 当前临时状态（2026-11-01）：按用户要求，工控机直接发送给制动电机的全部路径已用`WUTA_DIRECT_BRAKE_PAUSED`标记注释停用，包括NMT 0x000、0x603初始化/参数/启动/正常停止/异常和析构发送，以及初始化请求、应答解析和推进。原实现完整保留供恢复；target=2按中性处理。当前台架为真实GO→仅通过0x210请求5%微驱动5秒（可配置0～5%，0恢复中性等待）→50%制动10秒→中性/FINISH，不再等待直接初始化或3秒间隔；0x301心跳和台架超时/按路由限幅/转向归中保护保留。此限制只约束FSD自身，VCU或其他车端节点仍可能自行向电机发送报文。下文直控、双路流程为停用方案记录，以本条为准。未启动硬件。

> 核对：2026-10-13（同步10月12日工作）。以下是 `start_full_fsd.sh` 全车模式的核心契约，非全部调试话题枚举。话题可配置或 remap 时，以实际启动配置为准。

## 1. 感知、定位、建图和控制

类型简写均指 `包/msg/类型`。

| Topic | 类型 | 发布者 → 消费者 |
| --- | --- | --- |
| `/rslidar_points` | sensor_msgs/PointCloud2 | M1 → LiDAR 检测、融合、自检、显示 |
| `/zed/zed_node/rgb/image_rect_color` | sensor_msgs/Image | ZED → 相机检测、任务自检 |
| `/zed/zed_node/depth/depth_registered` | sensor_msgs/Image | ZED → 双目适配 |
| `/zed/zed_node/rgb/camera_info` | sensor_msgs/CameraInfo | ZED → 双目适配 |
| `/camera/yolo/cones` | wuta_msgs/CameraConeDetectionArray | YOLO/LW-DETR → 双目适配 |
| `/camera/yolo/image_annotated` | sensor_msgs/Image | 相机检测 → 显示 |
| `/perception/camera/cones` | wuta_msgs/CameraConeDetectionArray | 双目适配 → 后融合 |
| `/perception/camera/camera_info` | sensor_msgs/CameraInfo | 双目适配 → 后融合 |
| `/perception/lidar/cones_raw` | wuta_msgs/ConeArray | LiDAR 检测 → 后融合 |
| `/perception/fused/cones` | wuta_msgs/ConeArray | 后融合 → 建图（原订阅 `/perception/lidar/cones` 被 remap）及显示 |
| `/perception/fusion/status` | std_msgs/String | 后融合 → 调试工具，内容为 JSON |
| `/chcnav/devpvt` | msg_interfaces/Hcinspvatzcb | 华测协议节点 → FSD桥、任务机设备心跳及GNSS门控、终端状态 |
| `/chcnav/odometry` | nav_msgs/Odometry | 华测 FSD 桥 → EKF、CLI首帧检查 |
| `/chcnav/velocity` | geometry_msgs/TwistStamped | 华测 FSD 桥 → 控制器、规划记录 |
| `/odometry/filtered` | nav_msgs/Odometry | EKF → 定位管理 |
| `/localization/pose` | geometry_msgs/PoseStamped | 定位管理 → 建图、边界、规划、控制、任务机、显示 |
| `/system/localization_ready` | std_msgs/Bool | 定位管理 → 任务机、规划 |
| `/system/localization_confidence` | std_msgs/Float32 | 定位管理 → 任务机、规划 |
| `/mapping/cone_map` | wuta_msgs/ConeMap | 建图 → 边界、任务机、显示；5 Hz |
| `/planning/centerline` | autoware_msgs/Lane | 边界 → 规划、显示 |
| `/planning/global_centerline_ready` | std_msgs/Bool | 边界 → 任务机、规划 |
| `/planning/path_confidence` | std_msgs/Float32 | 边界 → 规划 |
| `/planning/final_waypoints` | autoware_msgs/Lane | 规划 → 控制、显示 |
| `/control/command` | autoware_msgs/Command | 控制 → CAN 接口、规划记录；默认控制回路50 Hz |

`/localization/velocity` 出现在旧仿真和显示配置中，但不是当前控制器的速度订阅。2026-10-14显示桥默认 velocity 已改为 `/chcnav/velocity`；旧配置或历史bag接入需核对话题。

## 2. 任务与故障

| Topic | 类型 | 发布者 → 消费者 / 语义 |
| --- | --- | --- |
| `/system/mission_state` | wuta_msgs/MissionState | 唯一发布者任务机 → 各算法节点、CAN、监控；10 Hz |
| `/system/lap_count` | std_msgs/UInt32 | 当前唯一发布者任务机 → 建图、规划；正式定位过线计圈 |
| `/system/mission_mode_cmd` | std_msgs/String | CAN → 任务机；trackdrive/skidpad/acceleration/inspection/ebs_test |
| `/system/start_command` | std_msgs/Bool | CAN → 任务机；RES GO 变化沿，且需 0x211 Byte1 7→8 满 5 s 后才会转发，非周期自动发车 |
| `/system/emergency` | std_msgs/Bool | CAN 或任务机自检 → 任务机/控制器；急停锁存 |
| `/system/devices_inspection` | wuta_msgs/DevicesInspection | 任务机 → CAN；ok + failures |
| `/system/mission_complete` | std_msgs/Bool | 控制器 → 任务机；Trackdrive 由正式圈次完成 |
| `/ndt/map_ready` | std_msgs/Bool | 可选 map_saver → 任务机；当前默认不依赖 |

任务机不订阅旧 `/system/lidar_ready`，而以M1点云接收心跳判断雷达在线；惯导自检以`ins_heartbeat_topic`（默认`/chcnav/devpvt`）接收心跳判断，GNSS=0不等于设备离线。默认心跳超时2秒。

`MissionState.description`包含`gnss_status`、`gnss_ready`及`wait`等诊断，显示端消费实际字段。GNSS=4为RTK固定解定位定向，8为固定解定位不定向，不等同于4。`gnss_ready`反映当前赛项的启动门控：车检不设GNSS门槛（设备心跳自检仍保留），其他任务及RACE要求新鲜4；完整0–9含义见`tools/gnss_status.py`。

桥默认`min_gnss_status=0`仍保留有限值/标准差检查；显式原点直接采用，自动原点在状态4时固定。GNSS低质量时收到odometry不代表可发车。

## 3. 消息契约

定义目录：`WUTA-FSD/ros2_ws/src/common/{wuta_msgs,autoware_msgs}/msg`。

| 消息 | 字段与解释 |
| --- | --- |
| Cone | position、color（默认4）、confidence；位置含义由数组 header.frame_id 决定 |
| ConeArray | Header + Cone[]；保留采样时间与坐标帧 |
| ConeMap | Header、red/blue/yellow/orange/unknown_cones、is_closed；目前没有地图质量结论字段 |
| CameraConeDetection | detection_id、bbox_xyxy[4]、color_probabilities[5]、confidence、position_valid、position、position_covariance[9] |
| CameraConeDetectionArray | Header + detections；曝光时间、矫正后左目光学帧 |
| MissionState | Header、state、mission_mode、localization_mode、description；description 含门控诊断，监控依赖其格式 |
| DevicesInspection | bool ok、string[] failures |
| Lane / Waypoint | Header + waypoints；每个航点含 PoseStamped 与 TwistStamped，速度剖面在 twist 中 |
| Command | Header、speed（m/s）、angle（前轮角度，deg）、throttle_brake（[-1,1]，正驱动/负制动）；CAN Signal1 取开度，非 speed |

颜色固定：**RED=0、YELLOW=1、BLUE=2、ORANGE=3、UNKNOWN=4**。相机概率顺序同号、长度5；建图边界红左/蓝右，黄/橙不进入边界配对。旧整数颜色地图、旧 rosbag 和旧消息 install 需显式迁移或重录重建。

状态编号：IDLE=0、READY=1、INSPECTION=2、EXPLORE=3、MAPPING_DONE=4、RACE=5、FINISH=6、EMERGENCY=7。任务模式：TRACKDRIVE=0、SKIDPAD=1、ACCELERATION=2、INSPECTION=3、EBS_TEST=4。定位模式：LOC_KISS_ICP=0（当前为 EKF 分支）、LOC_NDT=1。

## 4. 坐标、时间与 QoS

实车 TF 主链：`map -> odom -> base_link -> zed_camera_link -> 相机光学帧 -> rslidar`。根脚本发布同原点 map→odom 和车辆→ZED 静态外参；EKF 唯一发布 odom→base_link；ZED 车辆动态 TF 在本入口关闭。相机雷达外参定义 `p_camera = R p_lidar + t`，静态发布父相机光学帧、子雷达帧。

华测桥发布 odom/base_link 里程计和 base_link 下速度，不发布 TF。定位管理将 EKF pose 标记为 map，成立条件是上述 map/odom 同原点静态关系；若改用非零 map→odom，应同时调整转换实现。

检测和融合保留传感器采样时间；建图按采样时间查 TF，缺失时排队重试。不能用最新车辆位姿直接转换历史检测。相机为曝光时间，框坐标属于矫正后左图，注册深度以米解释。

普通 `create_publisher/subscription(...,10)` 为 Reliable/Volatile/depth10；传感器点云、图像等输入常用 SensorDataQoS。正式 lap_count、global_centerline_ready、path_confidence 使用 Reliable/Transient Local/depth1；CAN/任务机急停使用同类锁存 QoS，GO 不锁存。接入时用 `ros2 topic info -v` 检查双方，实际频率取决于设备与运行负载。

## 5. CAN 当前实现

| 帧 | 配置默认 | 当前作用 |
| --- | --- | --- |
| 控制 | 0x210（528），周期0.1 s，同时受回调发送影响 | Signal1 纵向、Signal2 转向为两字节大端；Signal3 在线、Signal4 完成 |
| 心跳 | 0x301（769），周期0.05 s，DLC1 | Data[0] 随设备自检 ok 为1/0 |
| VCU 输入 | 0x501（1281） | 任务模式；当前模式周期重发，供晚启动任务机同步 |
| VCU 状态输入 | 0x211（529） | Byte1=整车状态；无人自检完成判据为 7→8，接收端起算 5 s |
| RES 输入 | 0x1E4（484） | 首字节0x13 GO、0x10急停、0x11上线；GO只在状态变化沿发布 |

发车保险：`0x1E4` 的 GO 边沿只在 `0x211 Byte1` 出现 7→8 且满 `go_gate.wait_sec`（默认5 s）后转发；此前 GO 直接丢弃，不挂起不重放。启动时若已在 8，从首次收到 8 起保守重计等待；`go_gate.stale_sec`（默认1 s，<=0关闭）用于 0x211 断流时关门。台架 VCU 不发 0x211 时可用 `go_gate.enabled=false` 关闭本层。急停 0x10 不受该保险影响。

CAN接口已回退到 `54a6038`：首份自检结论到达前不发送控制帧，正常模式之后按最近一次控制消息发送，不包含通用命令超时保护或标定表映射；台架模式额外执行下述500ms路由/Command/任务状态新鲜度检查。最大转角读取 `steer.max_steer_angle`，总启动检查与控制器的 `vehicle.max_steer_angle` 核对。当前 CAN 车速发布代码未启用，闭环速度来自华测。

标定与报文格式以 `system/can_interface/config/can_interface.yaml`、`src/can_interface.cpp` 及 `docs_for_can` 协议为准；ZLG 桥提供硬件传输，不负责算法任务状态。ROS 归零不等同于已经验证硬件制动。

## 6. 可视化与可选定位

算法 MarkerArray：`/mapping/cone_map_viz`、`/planning/centerline_viz`、`/planning/final_waypoints_viz`、`/planning/driven_trajectory_viz`、`/control/target_viz`。当前硬件不保证发布显示配置中的原生 fused/guided-clusters marker，默认 bridge 从真实算法消息生成显示。

可选 KISS/NDT：`/kiss/odometry`、`/kiss/odometry_sanitized`、`/ndt/pose`、`/ndt/path`、`/ndt/aligned_cloud`、`/initialpose`。源码中部分输入仍为 `/hesai/pandar`，当前 M1 部署若启用必须先适配。它们不属于默认实车运行链。

## 无线显示扩展接口 2026年10月17日

`fsd-visualization-code/radio_link`消费既有标准MarkerArray、PoseStamped、TwistStamped和TF，排除图像与原始点云；不增加ConeMap、车辆TF、MissionState或驾驶指令的发布者。native模式消费配置中的原生`*_viz`，bridge模式复用原显示生成器的`/fsd_visualization/*`映射，要求已有显示桥生产这些数据。

| Topic | 类型 | 发布者与消费者 |
| --- | --- | --- |
| `/fsd_radio/extension/tx` | std_msgs/String，JSON对象 | Linux扩展生产者 → 无线发送队列 → PC `/radio/extension/rx` |
| `/fsd_radio/extension/rx` | std_msgs/String，JSON对象 | 无线接收端 → 明确接入的扩展消费者；不自动转发到驾驶/任务话题 |
| `/fsd_radio/status` | std_msgs/String，JSON状态 | Linux无线节点 → 只读监控，含串口连接、字节数、重试、未知输入 |

扩展ROS接口Reliable/Volatile/depth10，显示输入Best Effort/Volatile/depth1；波特率115200 bps 8N1、模块空速62.5 kbps，当前默认双向合计UART预算3000字节/秒。PC只经Foxglove SDK发布本地显示通道，不向ROS发布TF。压缩、量化、抽样和数据年龄语义见[无线模块说明](../fsd-visualization-code/radio_link/README.md)。本机默认COM3；Linux无线串口从radio.yaml读取，可通过启动参数覆盖，不能复用惯导设备。

2026-10-19（0.2.0历史；当前schema 3见末节）：ROS话题与所有权不变，几何无线消息改为schema 2必要字段二进制。固定frame/量化步长用双端一致配置及16位指纹检查，点坐标与必要尺寸5厘米量化，源时间保留毫秒；外观样式、标签和临时显示ID由PC生成。Linux抽样计数仅保留在`/fsd_radio/status.sampling`；PC `/radio/layers`显示schema、解码对象数与新鲜度。扩展JSON和外层分片/ACK仍保留，刷新间隔不提高，先升级接收端再升级发送端。

## 任务诊断兼容约定

`/system/mission_state` 的消息结构不变。`description` 保留 lap、mode_selected、
go_count、go_pending、lidar_ready、localization_ready 及地图/定位/RACE字段，
新增 `diag_version=2` 和 `wait=<原因列表或none>`。等待原因由任务机提供，展示端
不维护第二套门控规则；旧版本或字段缺失时显示未知。
原因值包括 lidar、localization、mode、go、map_closed、map_quality、centerline、
pose、confidence、ndt、lap、emergency、finished。
`mission_monitor` 只订阅状态和自检，不发布赛项、GO或控制指令。状态事件统一进入
manager.log；仅在启动前模式缓存不可用时，把首次观察到的赛项补写进 `run.json.mission`。
目录与归档见 [LOGGING](LOGGING.md)。


## 模式会话缓存

ZLG桥将当前会话的有效VCU模式写到`/run/zlgcan_bridge`。CAN接口默认启用`mode_cache`，等待2秒恢复模式，实时模式优先；仅模式可恢复，RES GO/急停不缓存。默认`max_age_sec=0`不按时间过期，仍验证桥PID、启动时间、接口及会话身份；缓存不代表VCU持续在线。日志启动入口只读同一缓存决定赛项目录，不发布任务状态。


### 固定路径的启动参考（2026-10-13）

`path_generator`的acceleration/skidpad/ebs均支持`anchor_frame=start_pose|map`。start_pose是规划内部冻结的二维参考，不新增TF边或pose发布者；YAML的X前、Y左、yaw=0沿启动车头；直线加速启用use_cone_direction时，X方向改为锥桶拟合的前向切线，起点投影到中心线。最终Lane和planned日志保持map，路径速度仍为沿路径目标速度。公共华测ENU/车体速度及map→odom同原点关系不变。Trackdrive使用真实map锥桶中心线，不再次应用启动旋转。

### RViz只读诊断与坐标表（2026-10-14）

根启动脚本的bridge模式默认加载`fsd_rviz_plugins/DiagnosticsPanel`。显示桥消费上表算法结果及MissionState、DevicesInspection、Command、融合JSON诊断、定位ready/confidence、全局中心线ready/path_confidence和急停事件；只发布`/fsd_visualization/*`显示结果。地图输入为`wuta_msgs/ConeMap`，不是ConeArray。默认速度来自`/chcnav/velocity`，表示车体运动速度而非驱动轮速。

`/fsd_visualization/status`（std_msgs/String，Best Effort/Volatile/depth1）保留速度/轨迹/输入墙钟帧龄字段，新增`schema_version=1`、`stale_seconds`、`streams`和`cone_coordinates`。streams每项含label/topic/state/age_s及实际消息摘要；有header的输入还提供stamp_ns/frame/sample_age_s。状态包括未启用、未收到、空结果、正常、超时缓存、时间戳超前及无效数据；显示超时阈值不是控制器或任务机的驾驶门控。锁存输入按生产者QoS订阅；急停单次事件显示末次锁存值，不按普通周期消息判定过期。

坐标表按融合消息header保留原始XYZ；可切换base_link，使用同一采样时间TF且不回退最新TF。TF不可用时显示原因、车体坐标为空，原始坐标仍可查看；表格序号仅为本帧数组索引，非跨帧ID，confidence不是位置误差。坐标表采样纳秒以字符串传输，避免Qt JSON浮点精度丢失。诊断面板独立监测显示桥墙钟断流（2秒），防止桥退出后旧值仍显示正常。

MissionState.description提供真实门控字段，缺失显示未知；面板不生成第二套发车结论。Command开度正/负/零仅标记驱动请求/制动请求/中性，不推断实际电机状态。sensor-only关闭未启动的任务、建图、规划、控制和定位诊断输入，保留感知与真实CAN急停事件；不伪造TF或车辆数据。

## 独立显示部署补充（2026-10-17）

显示及无线实现改由`fsd-visualization-code`子模块维护，现有ROS消息、话题与发布所有权不变。Linux E22串口从`radio_link/radio.yaml`的`serial.linux_port`读取，`--port`可覆盖；本机配置为CH340的by-id路径，避免占用惯导。`/fsd_radio/status`的`connected`仅证明串口打开，`tx_bytes`证明写入UART；实际双向状态还需`rx_bytes`、`peer_session`、`peer_age_s`及带ID扩展往返共同核对。设备共存证据见[DEVICE_COEXISTENCE](../fsd-visualization-code/docs/DEVICE_COEXISTENCE.md)。

## 无线源时间契约（2026-10-19，0.3.0）

ROS消息、话题和生产所有权不变。无线schema 3完整保留PoseStamped、TwistStamped及每个保留Marker的header.stamp（纳秒），速度与定位分别标时；frame_locked Marker也固定在其源时刻查询TF。PC的`/radio/state`以定位源时间、`/radio/speed`以速度源时间写SDK log_time，对象SceneEntity.timestamp取原Marker时间；默认缓冲3秒，配置与边界见无线README。源时间为0或无Header时不能从接收端补采集时间，计入未知时间诊断。扩展JSON增加source_stamp_ns/clock_domain/time_origin字段，缺省入队时间明确标为enqueue；应用生产者提供的源时间优先。来自不同时间基准的数据不能直接对齐。

## CLI无线日志与传感器（0.4.0）

无线0.4.0不新增ROS订阅/驾驶出口：读取CLI的manager.events.jsonl与telemetry.json。PC增加`/radio/cli/log`（foxglove.Log）、`/radio/cli/sensors`、`/radio/cli/vehicle`、`/radio/cli/transfer`、`/radio/cli/health`；它们是PC SDK通道，不是ROS话题。日志保留CLI源端Unix纳秒时间，传感器各项保留Header时间或标未知；块封装时间和源端接收时间单列。L/b日志字节块、A持久化确认、cli_status JSON契约见无线README。

0.5.0无线日志源改为`manager.radio.events.jsonl`必要事件子集，字段与L/b/A协议不变，使用独立stream ID避免与旧全文偏移混用。`/radio/cli/log`只含选中事件完整原文，CLI全部历史保留在Linux；传感器通道不受文本筛选影响。

0.5.1将日志发现扩展至`wuta_logs/`及旧`logs/`，继续消费状态文件的`radio_log_events`、`log_dir`，归档按stream ID重定位。未改变L/b/A、cli_status、源时间、ROS订阅或160 B/s预算。CLI保留原`module_log_dir`字段，两个界面额外从本次`log_dir`列出外部驱动、华测和CAN日志。

## 无线0.6.0当前运行契约（2026-10-20）

ROS所有权、消息字段及源时间语义沿用原接口。CLI新增 `.runtime/fsd_manager/radio_run.json`，字段scope（32位小写hex UUID）、journal（可空）、t（Unix纳秒字符串）、label、active；manager状态包含同一radio_run对象。开始新运行更换scope；同一运行归档只更新路径和结束时间。

无线新增JSON `cli_run`：k、scope、t（Unix纳秒整数）、stream（必要journal的UUID，可空）、label、active。发送端优先传输并每2秒重发选择信息；cli_status增加current_run。PC仅接受选定stream的L/b块，拒收旧scope及旧stream；A仍只确认已落盘字节。PC在当前scope变化时重置Foxglove会话（包括图表历史），保留并重放当前几何缓存；同一运行重连可从本机持久化journal恢复最近2000条/4 MiB显示。两端及CLI须同步升级，不能以旧接收端实现当前运行隔离。

默认UART 115200，link.air_rate_bps=62500仅描述硬件空速，不写模块；总预算3000 B/s包含双向帧、确认及重试。目标车辆4 Hz、传感器快照每2秒；实际频率随帧长、日志和重试变化。几何源时间继续按原规则对齐，本机乱序缓冲默认0.5秒。本轮未执行验证。

0.6.1：会话变化后重新公告同名Foxglove通道，客户端通道ID可变化；缓存重新绑定新ID并保留原源时间。无线帧和ROS接口不变。

0.6.2当前策略：radio_run.journal与radio_log_events指向manager.events.jsonl，run.json的manager_radio_events亦为该文件；L/b消息、源时间、stream ID与A落盘确认格式不变。选择完整stream后，info、菜单/检查、普通构建、GNSS状态及错误原文全部按原顺序传输。更新并重启CLI及Linux发送端建立新scope，避免在已选旧scope内改换stream。

2026-10-20：启用既有/planning/final_waypoints_viz（native）或/fsd_visualization/planned_path（bridge）无线消费，Foxglove沿用/radio/scene/path。目标1秒、权重3，最多4个Marker且每Marker48点；源时间、抽样几何与现有帧格式不变，不等同于全量Lane消息上传。

PC新增/radio/cli/dashboard SDK通道，仅为现有CLI快照的本地显示投影：source_stamp_ns为原快照源时间，states为gnss/ins/lidar/camera/fusion状态码（0未收到、1在线、2源端超时、3缓存过期），gnss_quality为原详情状态码或-1未知，rows为中文表格字段。receive_age_s/stale为本机显示元数据；每秒刷新不改变原样本源时间，不是新传感器样本或ROS话题。无线接口不变。

## 无线0.7.0：CLI统一管理接口

ROS几何/定位/速度/任务接口不变。`/fsd_radio/extension/tx`既有JSON发布入口新增本机截获的 `{"type":"cli","action":"check|start|stop|save|reload|status"}`，PC转为RPC，Linux拦截cli_rpc/cli_rpc_ack后投递CLI邮箱，不再转发这些内部消息到通用ROS扩展接收话题。

新增通用无线消息k=cli_control：t为CLI心跳Unix纳秒字符串，含boot、online、phase、running、run、active、last、config_error，目标2秒更新。RPC块body含type、id、dir、i，数据另含n与base64 data；原扩展512字节限制及轮询主从规则不变。压缩JSON最多256KiB，业务回复含最终ok/result/error与Linux source_stamp_ns，块ACK不代表执行成功。

PC SDK新增Parameters能力：selection/module按需读取，yaml/<模块>/<完整路径>修改PC草稿，meta/*只读；Publish保存按钮成组提交revision+changes，由CLI原工作线程执行。新增本机SDK通道 `/radio/cli/control`，快照用Linux心跳源时间，内嵌结果用自身执行结束源时间。boot/run绑定、持久去重、超时未知状态与配置生效边界见 [FOXGLOVE_PARAMETERS](FOXGLOVE_PARAMETERS.md)。不新增驾驶/任务控制话题，管理启动/停止由CLI既有业务执行。

无线0.7.1：RPC数据块及ACK新增可选p=1（指令）/0（普通数据），缺省0；成功/错误业务回复按原操作分类进入同优先级。cli.check/start/stop/status/result、config.set及其ACK/回复走独立指令FIFO，config.list/read走普通扩展FIFO。指令每次新分片选择时优先于日志/几何，无新ROS话题、无新串口通道，总预算保持。PC /radio/link新增command_queue计数；extension_queue只计普通扩展。旧端兼容解析，但须配套更新才能在两方向实现优先级。

0.7.2仅PC显示增加/radio/cli/control.display_state，来自原phase与在线/新鲜度判定；源时间保持原CLI心跳值。用于原生状态Indicator，原ROS及无线协议不变。

0.7.3 PC /radio/cli/control增加busy布尔显示字段，display_state在线有管理请求活动/等待结果时为操作中；离线优先，自动参数读取排除。phase与源时间保留，ROS和无线接口不变。

0.7.4仅PC /radio/link增加嵌套rssi：enabled/valid/raw/display_raw/age_s/captured_at_unix_ns/clock_domain/time_origin/samples/packet_session/packet_seq。raw为模块附加无符号字节，display_raw有效时0..255，否则-1等待/-2关闭/-3过期/-4断开。captured_at_unix_ns为本机UART捕获时间，time_origin=local_uart_capture，不是模块硬件时间或Linux ROS源时间。应用无线/ROS消息不变，模块包尾RSSI在原Framer提取，不进入应用CRC或JSON；依赖240字节模块分包、一应用帧一无线包及对应RSSI开关匹配。

0.7.5仅PC连接诊断/radio/link增加port_selection=auto/explicit，port为当前实际选择的COM号（尚未匹配时可为空）。默认按USB芯片匹配，不改变无线、ROS、CLI或RSSI格式。

0.7.6 PC /radio/cli/control新增parameter_state与parameter_detail，供参数操作Indicator/详情显示，保存结果来自原config.set回复；不更改ROS或无线格式，原源时间保持。

0.7.7 CLI心跳cli_control和cli.status新增可选runtime_exit_code（整数或null），来源为Manager的start_full_fsd子进程。PC /radio/cli/control增加exit_detail及exit_code_origin=start_full_fsd_child，display_state可为退出码:<值>或其他退出码；原phase/源时间保留。旧缺失字段可兼容，完整码显示需要CLI与发送端更新。ROS话题及执行请求格式不变。

## 静态制动台架接口（2026-10-21）

MissionState新增`bool brake_bench_mode`，由mission_manager发布；任务机、控制器及CAN的启动参数同名且默认false，根入口统一启用。控制器双向核对运行模式，不一致锁定中性。需重建消息及全部直接消费者并停止旧节点，旧install/overlay不可混用。消息其他字段及赛项/状态枚举保持。

台架仍消费真实`/system/mission_mode_cmd`的acceleration与之后的`/system/start_command` GO，状态为IDLE→READY→EXPLORE→FINISH。description增加`brake_bench`与`sensor_checks_exempt`诊断，台架wait使用`acceleration_mode,go`；原始gnss_status/lidar_ready/localization_ready继续如实报告。DevicesInspection.ok在台架表示豁免传感器后的台架检查结论，不是传感器在线证明；急停态为false。

原`/control/command`在台架的speed=0、angle=0；当前默认先以target=3请求+0.05持续5秒，再以target=1请求-0.5持续10秒，结束target=0并持续中性。CAN 0x210 Signal1依次为34405(0x8665)、16383(0x3FFF)、32767(0x7FFF)，Signal2始终32767；控制帧周期100ms，另有输入回调触发发送。路由、Command或台架活动MissionState接收年龄超过500ms、自检失败、RES急停或状态不再活动时中性。正常模式协议不变。控制器急停订阅为Reliable/Transient Local/depth1，与原急停发布端一致，重启可收到锁存急停。

## 人工驾驶记录接口（2026-10-22）

`--manual-record` 不改变消息定义或话题所有权。controller不启动，无本模式的 `/control/command` 发布者。CAN接口参数 `receive_only=true` 禁用全部发送（0x210/0x301、回调/定时器/退出），保留0x501/RES接收与既有ROS解析出口；ZLG桥 `--receive-only` 同时阻断SocketCAN→硬件。原VCU模式快照继续生成，但本次RunLog不按缓存分类。

`mission_manager.manual_record_mode=true` 时忽略选档、GO、完成事件，不激活驾驶任务；保留真实设备自检、急停和标准定位消费，正式圈数保持未激活状态。MissionState.description增加 `manual_record=0|1`；人工记录正常时 `wait=manual_record`，故障时仍为急停诊断。此状态表示采集用途，不能作为自动驾驶就绪或发车许可。原 path_generator 收到有效定位即写实际轨迹；无控制指令时原cmd_*及cmd时间列为nan，无活动任务则规划文件可仅有表头。无新增ROS发布者或原始传感器录制话题。

### 历史：台架制动电机前置CAN接口（2026-10-22，已停用）

> 本节仅记录历史方案，不是当前启用接口。当前初始化请求/结果接口和直接电机协议均被编译排除；生效行为见“台架制动前微驱动”。

> 2026-10-23早期临时状态（已由下述延时后初始化方案替代）：按用户要求，parseResFrame中调用prepareBrakeBench的入口已暂时注释，以下0x603前置流程仅作历史记录，当前不会执行。GO直接进入原任务机，台架仍按原控制器参数等待5秒、以50%制动10秒、释放中性；0x210和0x301保持，FSD不再额外发送0x603。


只在brake_bench_mode=true时，can_interface收到0x1E4首字节0x13的新变化沿，且权威任务状态为新鲜的acceleration/READY、台架自检ok，才向标准ID0x603写入6帧，DLC8，依次为 `2B 71 60 00 18 FC FF FF` 与 `2B 40 60 00 1F 00 00 00` 交替三组。六次SocketCAN写入成功后才发布原/system/start_command；无新ROS话题或消息字段。原任务机决定EXPLORE，原controller随后执行5秒等待和10秒制动。任一写入失败停止序列、不发布GO、发布原/system/emergency=true；无自动重试、无SDO应答确认。正常模式和receive_only不发送这些前置帧。

无线0.7.10：新增控制请求cli.cycle_mode，含expected_mode（normal/brake_bench/manual_record）、原boot/过期时间/源时间与请求ID；Linux原Manager串行执行M键循环，运行/清理及sensor-only/build-only会话拒绝。选中模式变化则拒绝迟到请求，不自动重试新ID。CLI心跳与cli.status增加selected_mode、active_mode，后者沿用run_log模式hardware/brake_bench/manual_record/sensor_only或null；PC /radio/cli/control新增mode_display、display_state_with_mode，保留原display_state/phase及源时间。请求及回复/ACK沿用高优先级，不新增ROS话题、驾驶指令或模块配置写入。

### 历史：台架延时后初始化接口（2026-10-23，已停用）

> 本节仅记录历史方案，不是当前启用接口。当前初始化请求/结果接口和直接电机协议均被编译排除；生效行为见“台架制动前微驱动”。

| 话题 | 类型 | 唯一生产者 → 消费者 | 语义 |
| --- | --- | --- | --- |
| `/control/brake_bench/initialize` | std_msgs/Bool | controller → can_interface | 真实GO进入EXPLORE且等待5秒后，true请求本次初始化 |
| `/can/brake_bench/initialized` | std_msgs/Bool | can_interface → controller | true表示SDO写入及模式/状态回读通过；false表示失败 |

两接口仅在brake_bench_mode=true时创建，Reliable/Volatile/depth10；每次进程运行只尝试一次，无自动重试。GO仍直接交原任务机。初始化期间0x210纵向及转向中性；成功后才允许原Command的负开度，10秒保持从成功结果开始计算。controller等待结果超过8秒锁定中性。

NMT使用标准ID0x000、DLC2、`01 03`，仅启动节点3；随后标准ID0x603、DLC8依次执行下列请求，监听0x583：

| 步骤 | 请求数据（默认参数） | 作用 |
| --- | --- | --- |
| 1 | `2B 40 60 00 01 00 00 00` | 初始化设备 |
| 2 | `2B 71 60 00 00 00 00 00` | 清零目标转矩，防止继承旧目标 |
| 3 | `2F 60 60 00 04 00 00 00` | 转矩模式4 |
| 4 | `23 81 60 00 0A 00 00 00` | 目标速度原始值10 |
| 5 | `2B 87 60 00 E8 03 00 00` | 转矩斜率1000‰/s |
| 6 | `2B 40 60 00 03 00 00 00` | 打开刹车 |
| 7 | `2B 40 60 00 0F 00 00 00` | 使能电机 |
| 8 | `2B 40 60 00 1F 00 00 00` | 启动，目标转矩仍为0 |
| 9 | `40 61 60 00 00 00 00 00` | 回读模式，要求4F及值4 |
| 10 | `40 41 60 00 00 00 00 00` | 回读状态，要求4B、使能bit2=1、故障bit3=0、暂停bit8=0 |

各写请求要求匹配index/subindex的0x60应答；每步应答超时500ms，应答后至少间隔20ms发送下一步。发送失败、SDO abort/格式异常、模式/状态不符或初始化期间输入失效，停止序列、回报false并发布原急停事件，0x210中性。手册第325页示例参数用于配置，目标转矩固定0，未直接发送示例500‰或历史负转矩。0x210仍由VCU解释开度，最终制动力及释放须由硬件验收。其他控制器若同时发送同节点SDO，可能覆盖设置，且同一对象的应答无法唯一归属发送者，本实现不能排除这种总线竞争。

### 历史：台架电机直控（2026-10-23，已停用）

> 本节仅记录历史方案，不是当前启用接口。当前初始化请求/结果接口和直接电机协议均被编译排除；生效行为见“台架制动前微驱动”。

该历史版本恢复过初始化请求/结果话题，并保持初始化NMT及10步SDO；结果成功后controller继续发布中性3秒，再发布原Command负开度10秒。can_interface在台架模式将负开度转换为6071额定转矩千分比，0x210纵向/转向始终0x7FFF，不再向VCU请求制动。50%是额定转矩请求，不是实测制动力/行程比例。

| 阶段 | ID / DLC | 数据 |
| --- | --- | --- |
| 初始化确认后再等3秒，发送指定原始转矩报文 | 0x603 / 8 | `2B 71 60 00 17 FC FF FF` |
| 前帧成功应答后启动 | 0x603 / 8 | `2B 40 60 00 1F 00 00 00` |
| 10秒制动请求结束，清零转矩 | 0x603 / 8 | `2B 71 60 00 00 00 00 00` |
| 前帧成功应答后暂停 | 0x603 / 8 | `2B 40 60 00 1F 01 00 00` |

以上沿原SDO事务出口逐帧确认0x583、500ms超时、应答后20ms间隔。保持10秒从controller发出负Command开始，包含SDO应答和转矩斜坡时间，不等价于实测50%转矩平台持续10秒。一次进程只执行一次电机制动；输入失效/任务结束触发清零和暂停，释放事务不受FINISH/失联门控阻止。SDO失败沿原急停事件退出，并尽力直发清零+暂停；正常析构也尽力直发，不能保证断电/强杀/总线故障下送达或机械复位。原同节点多发送者覆盖限制仍适用。

### 保留源码：最小配置/启动序列（2026-10-23，当前编译排除）

> 本节仅记录历史方案，不是当前启用接口。当前初始化请求/结果接口和直接电机协议均被编译排除；生效行为见“台架制动前微驱动”。

| 顺序 | CAN ID | DLC | 原始数据 | 下一帧最小间隔 |
| --- | --- | --- | --- | --- |
| 1 | 0x000 | 2 | `01 00` | 20ms |
| 2 | 0x603 | 8 | `2B 40 60 00 01 00 00 00` | 20ms |
| 3 | 0x603 | 8 | `2B 40 60 00 03 00 00 00` | 20ms |
| 4 | 0x603 | 8 | `2B 40 60 00 0F 00 00 00` | 20ms |
| 5 | 0x603 | 8 | `2F 60 60 00 04 00 00 00` | 2ms |
| 6 | 0x603 | 8 | `23 81 60 00 02 00 00 00` | 20ms |
| 7 | 0x603 | 8 | `2B 71 60 00 E8 03 00 00` | 20ms |
| 8 | 0x603 | 8 | `2B 87 60 00 E8 03 00 00` | 成功应答后3秒，用户确认的延时例外 |
| 9 | 0x603 | 8 | `2B 40 60 00 EF 00 00 00` | 启动后保持原10秒请求 |

GO后5秒开始。/can/brake_bench/initialized=true现在表示第8帧配置写入应答完成；controller收到后等待post_init_delay_sec=3，原负Command仅触发第9帧，不能重复第7帧。无初始化清零、无回读、无比例换算或报文参数覆盖。速度02按原始值保留，不能沿最小代码注释直接宣称1rps；转矩+1000也不宣称50%额定转矩。NMT广播影响范围同原最小代码，未改为节点3单播。

SDO写入仍逐帧等待0x583、匹配index/subindex的60成功应答，500ms超时/abort失败沿急停退出。间隔从请求发送时刻计算，需同时满足应答到达及最小间隔；台架接收轮询不大于2ms，普通Linux调度不保证精确2ms。已删除初始化profile_velocity/torque_slope配置覆盖项。结束/异常的清零与暂停在配置/启动序列之外，报文沿现有停止逻辑保留，正常序列不插入。

### 历史：分时双路台架（2026-10-23，已停用）

> 本节仅记录历史方案，不是当前启用接口。当前初始化请求/结果接口和直接电机协议均被编译排除；生效行为见“台架制动前微驱动”。

新增`/control/brake_bench/target`：std_msgs/UInt8，controller唯一发布、can_interface消费，Reliable/Volatile/depth10，仅台架创建。值0=双方中性/配置阶段，1=仅VCU，2=仅直接电机。随每次台架Command发布更新；目标切换时CAN立即清除缓存Command及其seen标志，新Command到达前中性。目标/Command/任务状态任一超过500ms则两路均无制动请求，直接电机已启动时清零+暂停。正常Command生产者及任务机所有权不变。

VCU阶段不要求FSD电机初始化成功，只要求原任务状态/自检及新鲜目标/Command，负0.5编码0x210 Signal1=0x3FFF，横向0x7FFF。此时FSD不发送直接电机配置/启动帧。VCU释放后目标切0，0x210恢复0x7FFF，再按原最小代码执行电机配置；此前表格最终EF的触发条件更新为：配置确认成功且从VCU释放起inter_stage_delay_sec=3到期。电机目标2阶段0x210保持0x7FFF，负Command仅触发EF，不重复转矩帧。各路请求保持10秒，结束同时归中性。

新参数`acceleration.bench_brake.inter_stage_delay_sec=3.0`替代post_init_delay_sec，其计时从VCU释放/配置请求发出开始；配置过慢不会跳过确认或缩短后续10秒保持。其他源若同时向电机发送SDO，仍可能覆盖FSD设置，分时仅约束本进程的发送出口。


### 直线加速双侧方向（2026-10-25）

不新增ROS话题或消息。boundary_detector在MISSION_ACCELERATION下输出`/planning/centerline`（map系、两个有序点；从第一点指向第二点为前进方向，速度字段为0，由规划配置速度）。无效或稳定次数不足时发布空Lane，供规划清除候选方向。`/planning/path_confidence`为拟合内点比例/残差评分；`/planning/global_centerline_ready`保持false，不代表赛道闭环。`/planning/centerline_viz`同时显示红/蓝边界、中心线和前向箭头。

path_generator在任务活动后使用新鲜且稳定的中心线一次生成并冻结`/planning/final_waypoints`和planned记录；没有可信方向则等待，发布空final_waypoints清除控制器缓存，不发布有效加速路径。执行中传感器缺失不改写已冻结路径；传感器失效/急停仍由既有任务机、控制器负责。减速/停止的纵向距离仍是path_generator的acceleration.anchor配置，定位与测距职责不变，不新增锥桶终点判据。

### 感知链路 QoS（2026-10-25）

下表均为Reliable / Volatile / KeepLast；当前实车RoboSense点云及ZED的RGB、注册深度、CameraInfo发布者提供Reliable。

| 消费或发布端 | 深度 |
| --- | --- |
| lidar_detection原始点云输入 | 1 |
| YOLO/LW-DETR图像输入、标注图发布 | 5 |
| Python/C++深度适配器的检测框、深度、CameraInfo输入 | 5 |
| C++/Python融合的相机检测/内参、雷达检测/原始点云输入 | 5 |
| mission_manager雷达与相机心跳输入 | 5 |
| CLI感知首帧检查、PointCloud2/Image/CameraInfo/ConeArray遥测 | 1 |
| RViz原始点云与标注图、显示桥融合ConeArray输入 | 1 |

检测结果和深度适配输出保留原Reliable/depth10发布。惯导兼容订阅、其他显示Marker及急停Transient Local锁存契约不变。RViz配置生成器覆盖旧模板的原始点云与标注图QoS。

Best Effort发布者不能满足这些Reliable订阅，仿真输入也须配套提供Reliable。更新后须重启相关算法、CLI与显示，旧Best Effort标注图发布进程不能满足新显示订阅。原最新图像覆盖、检测/深度20帧缓存、10 ms匹配容差与50 ms等待不变；Reliable减少传输丢样，但不保证算法过载时每帧均被处理。

## 独立复盘只读接口（2026-10-31）

节点fsd_mission_review订阅/localization/pose（geometry_msgs/PoseStamped）、/chcnav/odometry（nav_msgs/Odometry）、/chcnav/velocity（geometry_msgs/TwistStamped）、/planning/final_waypoints（autoware_msgs/Lane）、/control/target_viz（visualization_msgs/MarkerArray，仅pp_target ADD）、/system/mission_state（wuta_msgs/MissionState）；均Reliable/Volatile/depth1000，保留现有发送端契约。/system/emergency订阅Reliable/Transient Local/depth1。发送端和控制消息不修改；目标Marker由现有控制器在有订阅者时生成，因此须在GO之前启动采集。对位姿仅使用采样时间TF→map，不生成TF或第二份标准pose。

输出/mission_review/{actual,planned,planned_history,localization,current,targets,label}均为visualization_msgs/MarkerArray，/mission_review/status为std_msgs/String，统一Reliable/Transient Local/depth1。每次几何快照自带DELETEALL和完整ADD集合，帧为map，时间零供离线历史图层使用。status包含segment、phase、reason、样本数量、规划版本数、warnings、max_observed_gap_s、pending_transforms。结束条件仅标记复盘状态；frozen不代表任务机FINISH，replay无实车数据订阅。当前JSONL schema=2，兼容schema 1回放；数据保存于工作空间test_maps独占采集目录，来源运行目录仅保留采集日志和ready指针，不变更已有轨迹CSV。旧Lane与待TF目标点的已知时序问题见[test_maps限制说明](../test_maps/README.md#已知限制2026-11-05核对)。

## 台架制动前微驱动（2026-11-01，覆盖旧无正驱动约束）

现有`/control/brake_bench/target`（std_msgs/UInt8）扩展值3=仅VCU制动前微驱动；值1=仅VCU负制动，0/停用的2/未知值保持中性。`/control/command`的消息定义、QoS与正驱动/负制动约定不变，台架speed=0/angle=0，throttle_brake前delay_sec阶段为pre_drive_opening，随后hold_sec为-opening，结束为0。控制器与CAN共用小正开度范围0～0.05，控制器拒绝非法参数，CAN仅目标3限幅允许正值；目标1遇正值归中性。路由切换先清缓存，新Command到达前可出现保护性中性帧。

默认GO→5%微驱动5秒→50%制动10秒→中性/mission_complete→FINISH；0.0参数恢复原中性等待。Signal1大端5%=86 65、中性7F FF、50%制动3F FF。所有原任务新鲜度/自检/RES/有限值门控不放宽；无正转向、无0x000/0x603直接电机发送。原正常驾驶协议与MissionState所有权不变。
