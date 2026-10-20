# 部署与运行


> 2026-10-13复核（同步当日工作）；本页命令供开发者执行。本次文档整理未启动实车、发车或改动运行配置。

## 1. 环境与文件

E22低带宽双向遥测与Windows Foxglove显示可选组件见[radio_link部署说明](../fsd-visualization-code/radio_link/README.md)。它独立启动，Linux只消费既有ROS显示/定位数据，Windows默认COM3；不由全车入口自动启用。Linux串口通过radio.yaml配置或启动参数覆盖，避免占用惯导端口；原生显示输入与bridge输入须按实际运行模式选择。当前要求串口115200 bps 8N1、模块空速62.5 kbps；软件仍按160字节/秒保守限速，仅提供降采样预览，首次接入需完成说明中的串口、ROS/TF、空口与Foxglove验收。

历史版本radio_link 0.2.0使用必要几何schema 2（当前0.3.0升级见末节），升级时先更新/重启PC接收端，再更新/重启Linux发送端；保留双端一致的fixed_frame和量化步长，PC样式在`radio.yaml:foxglove.styles`配置。旧PC不能解码新几何消息，新PC兼容旧消息，升级后应通过`/radio/layers`中各图层的`schema`字段确认已收到schema 2。

ROS 2 Humble、系统Python3.10、colcon/ament、PCL/Eigen/tf2、RViz及包内声明依赖；相机推理还需匹配 GPU/CUDA/TensorRT 与 engine 的依赖。构建产物不可跨机器直接视为可用。

| 项目 | 当前默认路径 / 来源 |
| --- | --- |
| ZED overlay | `/home/wuta/WUTA/zed/install/setup.bash`；ZED_SETUP可覆盖 |
| M1 overlay | `/home/wuta/WUTA/rslidar_sdk/install/setup.bash`；RSLIDAR_SETUP可覆盖 |
| 推理依赖 | `/home/wuta/lyx/WUTA/.hardware_deps`；HARDWARE_DEPS可覆盖 |
| 外部模拟器 | `/home/wuta/lyx/WUTA/WUTA-SIM/install/setup.bash`；SIM_SETUP可覆盖 |
| TensorRT模型 | `models/yolov8sp2-int8.engine`，1280×768；由相机模块配置相对路径解析 |
| 相机雷达标定 | `config/camera_lidar.yaml`，本地部署文件 |
| 车辆ZED外参 | `config/base_to_zed.yaml`，后轴中心base_link；示例仅供填充 |
| 惯导 / CAN / 雷达网络 | `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_BKEDc143M08-if00-port0` / can0 / enp8s0，192.168.1.102/24 |

本机文件存在不代表Git可提供部署资产；模型、标定、驱动、设备权限和环境需逐机检查。相机及雷达支架变化后重新标定。

## 2. 配置来源

实际参数来自 `WUTA-FSD/ros2_ws/src` 下各包 config；感知入口为 `fusion_modules_hardware.launch.py`。`field_tuning.yaml` 和旧根统一 YAML不是本入口参数总入口。修改源码 YAML并重启对应节点生效；修改消息或节点源码应重建。

```bash
cd /home/wuta/WUTA2026_FINAL
./start_full_fsd.sh --print-config
```

该命令只解析/检查配置并输出 JSON，不连接硬件。检查控制/CAN最大转角一致、感知话题链、自检输入、有限值与NDT关闭；它不能替代模型存在、设备或全部参数合法性检查。

显式覆盖支持 `--controller-config=FILE`、`--can-config=FILE`、`--model=FILE`、`--calibration=FILE`、`--base-to-zed=FILE` 和 `--ins-port=...`。模型覆盖不自动同步引擎输入尺寸；需核对相机配置。CAN_IFACE环境变量可选接口，外部CAN桥复用由启动器自动判断。

## 3. 构建与实车启动

```bash
./start_fsd_manager.sh --menu       # 推荐：单终端检查、状态与日志界面
./start_fsd_manager.sh --check      # 管理器并行只读预检查
./start_full_fsd.sh --check
./start_full_fsd.sh --build-only
./start_full_fsd.sh --skip-build --no-rviz
./start_full_fsd.sh --status
./start_full_fsd.sh --stop
```

`--check` 检查部署文件、设备/网络和已有资源，失败原因以输出为准。`--build-only` 构建华测msg_interfaces/chcnav、当前FSD各包、ZLG桥和显示包。默认自动构建仅在关键setup缺失时触发，已有install不保证源码修改已重建。

当前启动优先验证并复用同接口唯一外部ZLG桥，退出时保留它；无桥才创建并等待初始化，最多30秒。重复实例或接口不匹配报错，不强制停止外部桥。sudo通过本机本地askpass自动认证，串口缺权限时授予当前用户ACL。

管理器独立预检查并行，点云/图像/惯导首帧也并行；融合、定位及下游节点仍按依赖推进。CLI读取ROS话题，不额外读取串口字节。进程存活、收到首帧、持续在线、READY和实际发车是不同状态。

默认纯键盘Textual界面，X停止、Q清理退出；失败保留界面，可修复后重试。使用Catppuccin Mocha主题，无命令输入框或数字页面快捷键；Tab切换区域，方向键与Enter选择页面。GNOME登录自启配置和完整快捷键见[启动包说明](../fsd_startup/README.md)，维护方法见[开发指南](DEVELOPMENT.md#startup-development)。

CLI源码独立为 [WUTA-FSD-CLI](https://github.com/starry1N/WUTA-FSD-CLI)，宿主挂载路径仍为 `fsd_startup/`。新克隆或拉取本次目录拆分后，先在宿主根执行 `git submodule update --init --recursive`，再执行 `bash fsd_startup/install_ui.sh`。根启动入口和桌面自启路径不变；当前机器的本地askpass认证文件保留，但它不随Git分发，新部署需配置本地认证。

RViz默认开启，无图形桌面用 `--no-rviz`；关闭RViz会触发全栈进程监管停止。只看传感器/感知可使用 `--sensor-only`，仍连接真实CAN桥和传感器，但不启动控制链。`--reuse-ins`复用外部惯导并验证输出，退出时保留外部进程。

当前脚本不会伪造VCU选择或RES GO。启动前读取 ZLG 模式缓存，正常直接创建 `wuta_logs/<赛项>/<YYYYmmdd_HHMMSS>/`；缓存不可用时才先写 `wuta_logs/pending/<YYYYmmdd_HHMMSS>/` 并在收到实时模式后归档。传感器模式归入 sensor_only。`manager.log` 收集启动和任务诊断，FSD 模块输出在 fsd_modules，外部驱动/TF/RViz 在 external_log，轨迹在 trajectory（分段为 `segments/<赛项>/<YYYYmmdd_HHMMSS>_<序号>/`），配置副本在 configs，ZLG 在 zlgCAN_log，华测在 chcnav_log；未运行时管理日志在 wuta_logs/sessions。详见 [运行日志](LOGGING.md)。`.runtime/full_fsd`保存监管状态。Ctrl-C按脚本清理本次进程。

## 4. 运行检查

先source正确overlay，在单独终端只读检查：

```bash
source /opt/ros/humble/setup.bash
source humble-chcnav-cgi_ros2pkg/install/setup.bash
source WUTA-FSD/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic info -v /chcnav/velocity
ros2 topic echo --once /system/mission_state
ros2 topic echo --once /system/devices_inspection
ros2 run tf2_ros tf2_echo map base_link
```

惯导无输出先按[临时诊断](惯导临时诊断记录.md)检查字节、解析、CRC、devpvt到桥和拒绝计数。当前桥门槛为0，GNSS状态0本身不停止发布；任务自检用原始devpvt心跳，GNSS质量与设备在线分开。普通任务及RACE要求新鲜GNSS=4，车检不设GNSS门槛，仍保留设备心跳自检，GO仍需真实输入。

定位不ready时查华测→EKF→定位输出；地图为空时查融合颜色、时间、map←rslidar TF和最少命中；无路径时查红蓝数量、边界与方向；无法发车查自检、模式、GO是否已消费与状态门槛。监控超过3 s无状态广播会提示，具体信息见 `manager.log`。

## 5. 仿真验证

```bash
./start_wuta_sim_validation.sh --mission acceleration --duration 40 --speed 2
./start_wuta_sim_validation.sh --mission trackdrive --duration 60 --speed 4
./run_wuta_sim_regression.sh
```

默认ROS_DOMAIN_ID=157、ROS_LOCALHOST_ONLY=1，允许域101–232，通过项目域锁避免重入；用WUTA_SIM_DOMAIN覆盖，不能用硬件域0。source顺序是Humble→外部SIM→本仓库FSD local_setup。模拟器构建和源码维护属于外部依赖，不应在本仓库虚构WUTA-SIM目录。

回归覆盖分赛项PID直线、噪声、trackdrive片段、速度断流、定位断流、空路径、NaN路径与急停；单次记录在 `wuta_logs/<赛项>/<YYYYmmdd_HHMMSS>/`（元信息标记 simulation），包含配置、赛道副本、遥测、rosbag、节点日志与summary。批次索引和 results/report 保存到 `wuta_logs/sessions/batch_*`，场景链接指向赛项运行目录；单次 summary/telemetry/rosbag 在 validation 子目录。需要检查每个场景结果，不能只依据汇总脚本退出码。

详见 [仿真专题](WUTA_SIM验证与启动.md)。真实YOLO、真实CAN、标定、EBS制动和完整多圈不在上述短时回归的证明范围内。


### 固定路径启动参考更新（2026-10-13）

根全车入口仍读取`WUTA-FSD/ros2_ws/src/planning/path_generator/config/path_generator.yaml`。acceleration/skidpad/ebs当前anchor_frame均为start_pose，需重新构建path_generator并重启规划进程。Skidpad当前仍为U-turn测试，关闭u_turn_test.enabled才运行标准八字；新配置使用发车原点，不能直接配旧绝对地图锚点。Trackdrive地图/TF保持现有坐标策略。隔离回归：加载ROS及FSD install后用系统Python运行`tools/check_acceleration_start_pose.py`；不启动实车入口。

今日源码调参和实车历史快照分开记录，见[今日工作记录](工作记录_2026-10-13.md)。本次不重新启动设备或重跑回归。

## 独立显示仓库与E22部署（2026-10-17）

显示目录现为[starry1N/fsd-visualization-code](https://github.com/starry1N/fsd-visualization-code)私有子模块。新克隆或拉取拆分提交后在主仓库执行`git submodule update --init --recursive`，GitHub账号需具备该仓库读取权限；已有目录若含未提交改动，先保存本地内容再更新，勿强制覆盖。原有`start_full_fsd.sh`和显示挂载路径不变。

独立显示：`bash fsd-visualization-code/start.sh mode:=bridge`。本仓库内自动复用相邻已有FSD安装环境；外置部署设置`FSD_SETUP`。无线先执行`bash fsd-visualization-code/radio_link/setup_linux.sh`，再执行`bash fsd-visualization-code/radio_link/start_linux.sh --mode bridge`。两者独立，不启动算法或传感器。

本机E22默认by-id路径`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`写入`radio.yaml`，ttyUSB编号可能随插入顺序变化；惯导使用Prolific的by-id路径，不得混用。CH340无唯一序列号，多个同型号适配器需重新核对；BRLTTY两个服务本机已屏蔽。串口打开与TX增加不等于对端收到，需检查RX及peer会话并完成Ping/Pong。相机/雷达接口没有直接重叠，但未完成同时满载对照验证，详细证据见[共存检查](../fsd-visualization-code/docs/DEVICE_COEXISTENCE.md)。

## 2026-10-19：惯导固定设备路径

根`INS_PORT`默认、CLI预检查/权限处理、华测源码YAML、独立启动脚本及正式launch回退值已统一为`/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_BKEDc143M08-if00-port0`。当前解析为`/dev/ttyUSB1`，E22的CH340 by-id解析为`/dev/ttyUSB0`，不能按旧编号选择。`INS_PORT`和`--ins-port`仍可覆盖根入口，华测独立入口保留`--port`覆盖。现有独占检查按设备身份识别别名，不增加串口读者。

修改源码配置后重启相应启动入口生效，不修改install副本、不启动硬件验证；直接使用已安装launch的部署需先重建华测包以更新安装配置。已运行的CLI应重新打开以加载默认值。

## 无线0.3.0源时间升级

先更新并重启Windows接收端，再在Linux主项目根`git pull --ff-only`和`git submodule update --init fsd-visualization-code`，重新运行无线setup与原模式的start_linux.sh。沿用已核对的E22串口和native/bridge模式，勿同时开两个读者。`/radio/layers`确认schema=3后导入子模块`radio_link/foxglove_layout.json`；速度路径改为`/radio/speed.speed_m_s`。默认本机增加3秒乱序缓冲，不提高160字节/秒预算。全纳秒与逐对象时间会增加包长，原刷新间隔仅为调度目标，成本、字段、源时间缺失和混合时钟边界见[无线说明](../fsd-visualization-code/radio_link/README.md)。本轮仅本机开发验证，Linux由使用者拉取部署，不代表已完成实机ROS/TF与双端schema 3无线验收。

## CLI无线日志与传感器（0.5.1）

无线0.5.1必须同时更新CLI和显示子模块：在主项目根`git pull --ff-only`后执行`git submodule update --init --recursive`。按现场原流程重新打开CLI与无线发送程序，保持原模式/串口；旧进程不会自动加载筛选规则。默认144字节数据片要求模块240字节分包；UART总预算仍160 B/s，PC只响应轮询。导入radio_link/foxglove_layout.json，切到“CLI 日志与传感器”。出现`manager.radio.events.jsonl`且telemetry.json含sensors_data才具备新数据；CLI首次必要事件前可没有必要journal。检查`/fsd_radio/status.cli`和PC `/radio/cli/transfer`、`/radio/cli/health`，保留未确认journal和PC磁盘数据。旧全文journal不作为无线源，不回填旧版本未记录的事件；Linux完整日志继续保留。

本次日志布局重构需无线0.5.1；请同步主仓库锁定的全部子模块（`git submodule update --init --recursive`），确保拆分感知驱动/算法的FSD launch也配套更新。无线重新执行setup_linux.sh并按原模式重启；CLI重新打开后显示完整日志分类与感知驱动状态。历史目录及无线断点保留，新版可从归档目录续传。当前PC接收格式保持兼容，RViz/Foxglove几何和传感器源时间不因目录改名改变。

## 当前调试配置（2026-10-20）

保留本机已有调试参数：U-turn替代路径测试关闭（`u_turn_test.enabled: false`），未启用的测试参数为直线9 m、半径6.3 m、速度0.9 m/s。任务机当前配置关闭雷达和相机自检，惯导心跳自检开启；这不表示雷达/相机已通过检测，实车使用前需核对模块配置。车检不设GNSS质量门槛，其他赛项及RACE继续要求新鲜状态4。

## 无线0.6.0高速与当前运行升级（2026-10-20，当前配置）

此节替代上文0.3.0/0.5.1的160 B/s、3秒缓冲和历史归档补传说明。两端串口115200、8N1；空中62500 bit/s须预先在两端模块配置一致，程序不改写模块。应用双向总预算3000 UART B/s，50 ms采集调度，响应超时1.2秒、换向等待0.01秒；分片144字节、最大线帧226字节，仍适配模块240字节分包。车辆目标4 Hz、传感器快照2秒一次，日志优先使用其余调度机会；并非实测速率保证。PC源时间乱序缓冲0.5秒。

Linux在主仓库执行 `git pull --ff-only`、`git submodule update --init --recursive`，重新运行 `bash fsd-visualization-code/radio_link/setup_linux.sh`；关闭并重新打开CLI，然后沿用原模式执行 `bash fsd-visualization-code/radio_link/start_linux.sh --mode bridge`（原用native则继续native）。Windows同步升级后重新运行 `radio_link/start_windows.ps1`，默认COM3/115200，并导入新版foxglove_layout.json。CLI、Linux发送端和Windows接收端须配套升级；旧进程不会自动加载代码。

新CLI通过 `.runtime/fsd_manager/radio_run.json` 选择当前运行；无线不会自动补传历史目录。结束运行保留尾部续传，开始下一次运行切走旧队列；无需删除历史日志或断点。Foxglove会随scope更换重置会话，图表历史也随之清空。当前版仅完成源码修改与人工阅读，按用户要求未执行测试、构建、串口或实机验证。

0.6.1显示修复：Windows接收端更新并重启一次，以加载会话重置后的通道重公告逻辑。此后CLI重启/切换运行继续在原Foxglove连接显示，无需手动断开重连；Linux发送协议及模块参数无变化。未执行自动化验证。

## 无线0.6.2：当次完整日志（当前策略）

在主仓库git pull --ff-only后执行git submodule update --init --recursive，同步CLI与显示子模块，重启Linux CLI和无线发送程序。新版选择文件应指向当次manager.events.jsonl完整事件流；不再以manager.radio.events.jsonl作为新版运行源，不用删除历史或断点。Windows代码/布局同步更新，日志面板标题为“本次启动完整日志”，完整事件编码与接收格式未变化。115200、62500 bit/s与3000 UART B/s配置保持；不增加组件日志目录扫描。按要求本轮未执行验证。

## 启用最终规划路径（2026-10-20）

同步显示子模块后，重新部署并重启RViz与无线发送端。使用ROS安装副本启动RViz时先重新运行显示仓库build.sh更新安装模板；无线直接读取源码模板。沿用原native/bridge模式：native消费/planning/final_waypoints_viz，bridge消费/fsd_visualization/planned_path；显示桥须运行，不能混用模式。Foxglove现有/radio/scene/path图层已启用，无需重新建立连接。路径目标1秒、权重3，实际随日志、快照帧长及重试变化。未执行验证，待Linux拉取部署后生效。

## Foxglove原生传感器面板（2026-10-20）

本机更新显示子模块源码并重启Windows接收端，随后通过Foxglove布局菜单导入radio_link/foxglove_layout.json，切换“传感器概览”。仅使用原生Indicator/Gauge/Table，无需安装.foxe扩展包；原“CLI 日志与传感器”保留全文日志，增加“原始诊断”页。需要本版PC派生dashboard通道，Linux无需为此次界面调整重启或改参数。本轮未运行自动化验证。

布局样式调整仅需重新导入新版foxglove_layout.json：新鲜度、GNSS质量为背景样式，接收年龄为0～20秒条形显示。接收进程、COM3和Linux无需为此重启。当前交付文件及可恢复旧文件的位置见[目录索引](DELIVERY.md)。
