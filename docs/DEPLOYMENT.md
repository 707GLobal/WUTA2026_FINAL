# 部署与运行


> 2026-10-13复核（同步当日工作）；本页命令供开发者执行。本次文档整理未启动实车、发车或改动运行配置。

## 1. 环境与文件

E22低带宽双向遥测与Windows Foxglove显示可选组件见[radio_link部署说明](../fsd-visualization-code/radio_link/README.md)。它独立启动，Linux只消费既有ROS显示/定位数据，Windows默认COM3；不由全车入口自动启用。Linux串口通过radio.yaml配置或启动参数覆盖，避免占用惯导端口；原生显示输入与bridge输入须按实际运行模式选择。2.4kbps仅提供降采样预览，首次接入需完成说明中的串口、ROS/TF、空口与Foxglove验收。

radio_link 0.2.0使用必要几何schema 2，升级时先更新/重启PC接收端，再更新/重启Linux发送端；保留双端一致的fixed_frame和量化步长，PC样式在`radio.yaml:foxglove.styles`配置。旧PC不能解码新几何消息，新PC兼容旧消息，升级后应通过`/radio/layers`中各图层的`schema`字段确认已收到schema 2。

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
| 惯导 / CAN / 雷达网络 | `/dev/ttyUSB0` / can0 / enp8s0，192.168.1.102/24 |

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

当前脚本不会伪造VCU选择或RES GO。运行先写 `logs/pending/<运行时间>/`，结束并关闭文件后按实际赛项归档到 `logs/<赛项>/<运行时间>/`。未选赛项保留 pending，多赛项归入 multi_mission，传感器模式归入 sensor_only。`manager.log` 收集启动和任务诊断，模块输出在 modules，轨迹在 jrajectory，配置副本在 configs；未运行时管理日志在 logs/sessions。详见 [运行日志](LOGGING.md)。`.runtime/full_fsd`保存监管状态。Ctrl-C按脚本清理本次进程。

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

惯导无输出先按[临时诊断](惯导临时诊断记录.md)检查字节、解析、CRC、devpvt到桥和拒绝计数。当前桥门槛为0，GNSS状态0本身不停止发布；任务自检用原始devpvt心跳，GNSS质量与设备在线分开。普通任务及RACE要求新鲜GNSS=4，车检允许新鲜0或4，GO仍需真实输入。

定位不ready时查华测→EKF→定位输出；地图为空时查融合颜色、时间、map←rslidar TF和最少命中；无路径时查红蓝数量、边界与方向；无法发车查自检、模式、GO是否已消费与状态门槛。监控超过3 s无状态广播会提示，具体信息见 `manager.log`。

## 5. 仿真验证

```bash
./start_wuta_sim_validation.sh --mission acceleration --duration 40 --speed 2
./start_wuta_sim_validation.sh --mission trackdrive --duration 60 --speed 4
./run_wuta_sim_regression.sh
```

默认ROS_DOMAIN_ID=157、ROS_LOCALHOST_ONLY=1，允许域101–232，通过项目域锁避免重入；用WUTA_SIM_DOMAIN覆盖，不能用硬件域0。source顺序是Humble→外部SIM→本仓库FSD local_setup。模拟器构建和源码维护属于外部依赖，不应在本仓库虚构WUTA-SIM目录。

回归覆盖分赛项PID直线、噪声、trackdrive片段、速度断流、定位断流、空路径、NaN路径与急停；单次记录在 `logs/<赛项>/<运行时间>/`（元信息标记 simulation），包含配置、赛道副本、遥测、rosbag、节点日志与summary。批次索引和 results/report 保存到 `logs/sessions/batch_*`，场景链接指向赛项运行目录；单次 summary/telemetry/rosbag 在 validation 子目录。需要检查每个场景结果，不能只依据汇总脚本退出码。

详见 [仿真专题](WUTA_SIM验证与启动.md)。真实YOLO、真实CAN、标定、EBS制动和完整多圈不在上述短时回归的证明范围内。


### 固定路径启动参考更新（2026-10-13）

根全车入口仍读取`WUTA-FSD/ros2_ws/src/planning/path_generator/config/path_generator.yaml`。acceleration/skidpad/ebs当前anchor_frame均为start_pose，需重新构建path_generator并重启规划进程。Skidpad当前仍为U-turn测试，关闭u_turn_test.enabled才运行标准八字；新配置使用发车原点，不能直接配旧绝对地图锚点。Trackdrive地图/TF保持现有坐标策略。隔离回归：加载ROS及FSD install后用系统Python运行`tools/check_acceleration_start_pose.py`；不启动实车入口。

今日源码调参和实车历史快照分开记录，见[今日工作记录](工作记录_2026-10-13.md)。本次不重新启动设备或重跑回归。

## 独立显示仓库与E22部署（2026-10-17）

显示目录现为[starry1N/fsd-visualization-code](https://github.com/starry1N/fsd-visualization-code)私有子模块。新克隆或拉取拆分提交后在主仓库执行`git submodule update --init --recursive`，GitHub账号需具备该仓库读取权限；已有目录若含未提交改动，先保存本地内容再更新，勿强制覆盖。原有`start_full_fsd.sh`和显示挂载路径不变。

独立显示：`bash fsd-visualization-code/start.sh mode:=bridge`。本仓库内自动复用相邻已有FSD安装环境；外置部署设置`FSD_SETUP`。无线先执行`bash fsd-visualization-code/radio_link/setup_linux.sh`，再执行`bash fsd-visualization-code/radio_link/start_linux.sh --mode bridge`。两者独立，不启动算法或传感器。

本机E22默认by-id路径`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`写入`radio.yaml`，目前解析为ttyUSB1；惯导仍是Prolific ttyUSB0，不得混用。CH340无唯一序列号，多个同型号适配器需重新核对；BRLTTY两个服务本机已屏蔽。串口打开与TX增加不等于对端收到，需检查RX及peer会话并完成Ping/Pong。相机/雷达接口没有直接重叠，但未完成同时满载对照验证，详细证据见[共存检查](../fsd-visualization-code/docs/DEVICE_COEXISTENCE.md)。
