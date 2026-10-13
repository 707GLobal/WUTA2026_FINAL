# 实车全栈启动

新增彩色交互入口 `./start_fsd_manager.sh`：并行检查、详细错误保留菜单、
华测串口 ACL 与 CAN sudo 权限处理、残留清理和 Rich + Textual 单终端界面。
界面仅用键盘操作，无可点击按钮；检查、车辆状态、传感器参数和日志都在同一个终端。
建议先运行 `./start_fsd_manager.sh --menu`；详细操作见 [启动管理包](fsd_startup/README.md)。
原入口的行为说明如下，管理包发生错误时会保留自己的菜单。

入口：`./start_full_fsd.sh`。默认启动华测组合惯导、ZLG CAN 桥、ZED 2i、
RoboSense M1、YOLOv8s TensorRT、雷达检测、融合、EKF、定位管理、锥桶地图、
边界检测、路径规划、控制、CAN 接口和任务管理。

算法参数直接读取 `WUTA-FSD/ros2_ws/src` 各模块config：控制器使用
`control/controller/config/controller.yaml`，CAN标定与通信使用
`system/can_interface/config/can_interface.yaml`。主目录旧统一YAML不再加载。
用 `./start_full_fsd.sh --print-config` 查看实际来源和参数，无需连接硬件。
详细优先级见 [实车模块配置加载逻辑](docs/实车模块配置加载逻辑.md)。
修改模块YAML后重启对应节点生效。
目标速度在规划模块按赛项设置，控制器在对应赛项的 `pid.*` 中调节速度闭环和开度限幅。
调参布局以 `54a6038` 的分赛项设计为基准，已移除统一限速与纯比例速度调试入口。
用法和跑偏排查见 [分赛项速度调试](docs/直线调试与简单速度控制.md)。

首次部署先确认以下文件和设备：

- `models/yolov8sp2-int8.engine`：当前工控机上构建的 1280×768 TensorRT engine。
- `config/camera_lidar.yaml`：M1 到 ZED 左目光学帧的实测外参。
- `config/base_to_zed.yaml`：从安装尺寸和 M1↔ZED 标定推算的车辆 TF。`base_link`
  位于后轴中心，`rslidar` 在前方 2.33 m，侧向/高度同后轴；已确认雷达坐标轴
  与车辆坐标轴对齐。根据 ZED 2i URDF，推算 `base_link → zed_camera_link`
  为平移 `[0.6729, -0.0616, 1.0315]` m。它与粗测的相机前向 0.70 m、
  高度 1.00 m 相差约 3 cm；左目与相机安装点有约 6 cm 的侧向偏移。
  若安装姿态或坐标原点定义不同，应重新计算，不要直接改动单个数字。
- M1 网口 `enp8s0` 地址 `192.168.1.102/24`，雷达发送到该地址的 UDP 6699/7788。
- 组合惯导串口默认 `/dev/ttyUSB0`，ZLG 设备 USB ID `0471:126a`。
  串口权限按 `humble-chcnav-cgi_ros2pkg/README.md` 配置：临时执行
  `sudo chmod 666 /dev/ttyUSB0`，或将当前用户加入 `dialout` 组后重新登录。
- ZED、RoboSense 已安装的 ROS overlay 仍位于 `/home/wuta/WUTA/`；
  相机推理的 Python 依赖默认取 `/home/wuta/lyx/WUTA/.hardware_deps`。
  可用 `ZED_SETUP`、`RSLIDAR_SETUP`、`HARDWARE_DEPS` 环境变量覆盖。

```bash
cd /home/wuta/WUTA2026_FINAL
./start_full_fsd.sh --check           # 只检查硬件、部署文件与外参
./start_full_fsd.sh --build-only      # 首次构建华测桥、FSD 全部包和 ZLG CAN 桥
./start_full_fsd.sh --skip-build      # 使用 fsd-visualization-code 启动 RViz
./start_full_fsd.sh --status
./start_full_fsd.sh --stop
```

默认通过 `fsd-visualization-code/start.sh` 打开 RViz，使用该目录自带的显示布局，
显示相机检测画面、融合锥桶、赛道地图、中心线、规划路径、行驶轨迹、
控制预瞄目标和定位速度。M1 原始点云图层默认关闭，可在 Displays 中启用。
`--rviz` 与旧参数 `--image-view` 都启用该窗口，不再额外启动 rqt 图像窗口。
无图形桌面时使用 `--no-rviz`。关闭 RViz 会按现有进程监控逻辑停止本次全栈。

配置为 `fsd-visualization-code/src/fsd_visualization/config/fsd.rviz` 和同目录的
`topics.yaml`，可修改图层透明度、视角和话题。显示程序使用该代码包的安全图像插件和只读桥接，
首次默认启动自动构建可视化；更新插件后执行 `./start_full_fsd.sh --build-only`。
当前硬件融合节点未发布参考包中的原生融合来源/引导簇点集图层，因此使用 bridge
显示真实融合结果；原生融合来源/引导簇图层需算法发布对应数据后才能显示。
相机标注来自 `/camera/yolo/image_annotated`，点云来自
`/rslidar_points`；控制目标仍来自控制器真实 `/control/target_viz`。
`--sensor-only` 使用雷达坐标系，仅显示已有的传感器/感知结果，不启动规划控制。

只测试显示（隔离 ROS domain，合成数据）：

```bash
FSD_SETUP="$PWD/WUTA-FSD/ros2_ws/install/setup.bash" bash fsd-visualization-code/check-gui.sh --mode bridge --dashboard
```

当前启动优先复用所选 `CAN_IFACE`（默认 `can0`）上的唯一外部 ZLG 桥：验证进程身份、vcan类型与UP状态，退出FSD时保留外部桥。
没有桥时才创建本次运行的桥，新桥须在30秒内输出设备初始化完成且接口已启用；重复实例或接口不匹配时报错，避免重复打开USB设备。
CAN接口可恢复同一存活桥会话缓存的VCU模式，但不恢复RES GO或急停，实时模式优先。

只检查传感器与感知链，不启动建图、规划和控制时，使用 `--sensor-only`。
若华测链路已在另一终端运行，使用 `./start_full_fsd.sh --skip-build --reuse-ins`。
该模式仍检查 `/chcnav/odometry`，退出时保留外部华测进程。默认启动会检查串口占用，
独立华测启动脚本也检查占用并持有设备锁，防止两个读取进程争读同一串口。
串口变化可加 `--ins-port=/dev/ttyUSB1`；模型和外参可用
`--model=FILE`、`--calibration=FILE`、`--base-to-zed=FILE` 指定。

脚本启动华测桥、CAN 桥、相机与 M1，等待图像、点云和融合结果；
全车模式还等待 `/chcnav/odometry`，随后启动 EKF 与定位管理，确认 `/localization/pose` 后再启动地图、
规划、控制、CAN 接口和任务管理。ZED 不发布车辆动态 TF；EKF 发布
`odom -> base_link`，脚本发布 `map -> odom` 和实测 `base_link -> zed_camera_link`。
首版按 EXPLORE 配置运行，使用正常融合和累积地图；状态机即使转入 RACE
也继续使用 EKF，不启用 NDT。KISS-ICP、禾赛 NDT 节点完成构建但不会在
当前 M1 + 华测配置下自动运行。

CAN 桥脚本需要 sudo 创建虚拟 SocketCAN 接口；本脚本在前台认证，再提权启动后台桥。
后台桥由 `tools/run_privileged_group.py` 持续监管并转发退出信号，避免 `setsid` 分叉后
外层启动进程返回 0，被误报为桥已退出。
本机通过权限700且排除Git的本地askpass文件自动认证；不将口令写入日志。未配置本地辅助文件时，以入口实际认证提示为准。
设备初始化通过不代表已收到 VCU 报文，应检查桥日志的 `VCU->FSD` 计数是否增长。
CAN 接口会按现有配置发送上线心跳，控制帧由自检结果门控；脚本不会模拟
VCU 模式或 RES GO。所有日志写入 `logs/<赛项>/<运行时间>/`，Ctrl-C 只关闭此入口
启动的进程。

实测记录（2026-10-10）：华测 `/chcnav/odometry`、ZED 图像、M1 点云、YOLO 检测和融合话题均有真实消息；ZLG 桥已编译，完整 CAN 链路需在能输入 sudo 密码的终端启动。第三方华测解析器与 M1 驱动在限时测试收到中断信号时曾报告退出异常，进程均已结束。

实测记录（2026-10-11）：直接执行 `./start_full_fsd.sh`，自动复用已运行的 `can0` ZLG 桥。
M1 点云、ZED 图像、融合结果、华测惯导、定位位姿、任务状态六项首帧检查均通过，
13 个进程组启动完成，任务状态进入 READY，设备自检通过，等待任务选择和 GO。
EKF 启动时显式指定节点名 `ekf_node`，与参数文件匹配；融合配置使用 15 元素布尔数组。
话题首帧检查显式指定消息类型，允许 DDS 在启动期间发现发布者。
`--status` 与 `--stop` 实测通过，停止后本次 ROS 进程均退出，外部 ZLG 桥保留。

启动过程会逐项显示中文 `[启动]`、`[检查]` 和 `[成功]` 提示。进程启动成功会显示
项目名称、PID 和日志路径；点云、图像、融合、惯导、定位及任务状态的数据检查成功
会单独显示“已收到消息”。进程存活与收到消息是不同检查，不代表任务已发车。

失败时显示 `[失败]`、失败项目和具体检查原因。进程退出会显示退出码及最近 30 行
相关日志；话题等待失败会保留 ROS 命令输出，区分等待超时与命令报错，并显示相关
节点日志。其他命令失败会显示脚本行号、命令和退出码。运行中进程退出也会报告日志，
随后按原有清理流程停止本次脚本启动的进程。

全车模式自动启动只读任务监控，在状态变化时显示当前阶段、实际任务选择、雷达/定位
ready、任务机收到 GO 的累计次数、GO 是否仍待处理、圈数、设备自检，以及当前待满足
的启动/RACE门槛。默认 Trackdrive 不会被误报为已选择；GO 消费或切档清除后不再
显示为待处理。监控不发送任何模式、GO、急停或控制指令。
状态机超过 3 秒未广播时会提示。输出由启动日志入口统一保存到本次运行目录的
`manager.log`，只在信息变化时打印。`--sensor-only` 不启用任务监控。

此功能需要更新后的 `mission_manager` 广播门控诊断（复用 MissionState.description，
不改变消息结构或任务逻辑）。更新后先执行 `./start_full_fsd.sh --build-only`；若运行的是
旧版本任务机，监控会明确提示门控状态未知并要求重新构建。

任务机默认 YAML 已对齐当前 M1、华测与 ZED 话题，雷达 ready 直接来自 M1 点云心跳，不再依赖 `/system/lidar_ready`。

仿真验证使用独立入口 `./start_wuta_sim_validation.sh`（默认40秒、2米/秒），
与实车入口分开。`./run_wuta_sim_regression.sh` 执行基线和故障注入，记录写入
`logs/<赛项>/<运行时间>/`（`run.json` 标记 simulation）。连接方式、模型适配和已知覆盖边界见
[WUTA-SIM验证与启动](docs/WUTA_SIM验证与启动.md)。

## 日志目录与职责

运行先写 logs/pending/<运行时间>，停止并关闭日志后按实际赛项归档。
启动检查和只读任务监控统一写 manager.log；模块输出在 modules，
规划和实驶CSV在 jrajectory，配置在 configs，ROS日志在 ros，地图及惯导数据在 artifacts。
多赛项归入 multi_mission，仅传感器归入 sensor_only；未启动车辆时管理日志在 sessions。
轨迹后续段应递归读取 jrajectory/segments，不覆盖第一段数据。
详见 [日志设计与验证](docs/LOGGING.md)。

## 2026-10-12 更新：惯导在线与GNSS门控

桥的有效默认`min_gnss_status`已改为0：GNSS=0不因状态门槛停止发布，仍保留有限值及标准差过滤；自动ENU原点在状态4时固定。
任务机用`/chcnav/devpvt`判断设备心跳，与定位质量分开；普通任务及RACE要求新鲜状态4，车检允许新鲜0或4。CLI仍读取ROS话题，不额外打开串口；传感器页显示GNSS码和含义。
17:38的零上游帧、19:37的GNSS过滤是两种不同故障。后者发生在旧门槛1配置下；当前规则不能倒推为历史运行规则。
详见[10月12日工作记录](docs/工作记录_2026-10-12.md)和[惯导分层诊断](docs/惯导临时诊断记录.md)。

## 2026-10-13 文档与启动管理更新

终端已移除命令输入框和数字页面入口，使用Tab切换区域、方向键与Enter选页，窄屏始终保留菜单。
固定路径赛项采用共用start_pose/map转换，当前调试参数和历史运行快照见[今日工作记录](docs/工作记录_2026-10-13.md)。
