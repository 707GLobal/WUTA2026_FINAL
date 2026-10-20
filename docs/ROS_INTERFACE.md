# 当前核心 ROS 与 CAN 接口


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
| `/system/start_command` | std_msgs/Bool | CAN → 任务机；RES GO 变化沿，非周期自动发车 |
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
| RES 输入 | 0x1E4（484） | 首字节0x13 GO、0x10急停、0x11上线；GO只在状态变化沿发布 |

CAN接口已回退到 `54a6038`：首份自检结论到达前不发送控制帧，之后按最近一次控制消息发送，不包含命令超时保护或标定表映射。最大转角读取 `steer.max_steer_angle`，总启动检查与控制器的 `vehicle.max_steer_angle` 核对。当前 CAN 车速发布代码未启用，闭环速度来自华测。

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

`path_generator`的acceleration/skidpad/ebs均支持`anchor_frame=start_pose|map`。start_pose是规划内部冻结的二维参考，不新增TF边或pose发布者；YAML的X前、Y左、yaw=0沿启动车头。最终Lane和planned日志保持map，路径速度仍为沿路径目标速度。公共华测ENU/车体速度及map→odom同原点关系不变。Trackdrive使用真实map锥桶中心线，不再次应用启动旋转。

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
