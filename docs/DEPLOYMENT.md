# 部署与运行

## 测试路径地图（2026-11-05）

正常全车启动自动在控制器/任务机前启动显示子模块只读mission_review，不另开RViz、不改变现有RViz坐标系及视角。测试图集中在根`test_maps/`，每个采集会话独立目录；浏览器直接打开map.svg，任务分段为segment_001.svg。活动时每2秒更新，结束/关闭也导出；CSV仍在原trajectory目录。传感器/台架/人工记录不自动采集。采集失败只告警，日志为本次运行`artifacts/mission_review.log`，ready文件给出地图目录。独立采集与旧记录恢复命令见[test_maps说明](../test_maps/README.md)，不得用历史目录冒充新测试。未因本功能启动硬件。

> 当前临时状态（2026-11-01）：按用户要求，工控机直接发送给制动电机的全部路径已用`WUTA_DIRECT_BRAKE_PAUSED`标记注释停用，包括NMT 0x000、0x603初始化/参数/启动/正常停止/异常和析构发送，以及初始化请求、应答解析和推进。原实现完整保留供恢复；target=2按中性处理。当前台架为真实GO→仅通过0x210请求5%微驱动5秒（可配置0～5%，0恢复中性等待）→50%制动10秒→中性/FINISH，不再等待直接初始化或3秒间隔；0x301心跳和台架超时/按路由限幅/转向归中保护保留。此限制只约束FSD自身，VCU或其他车端节点仍可能自行向电机发送报文。下文直控、双路流程为停用方案记录，以本条为准。未启动硬件。

> 主链路原核对日期2026-10-13；无线/CLI条目已于2026-10-20按0.7.4复核。带版本日期的旧章节保留历史条件；当前无线默认值以radio.yaml及下述环境说明为准。本次文档审计未启动或停止设备。

## 1. 环境与文件

CLI与无线联启：正常运行 `start_fsd_manager.sh` 时，CLI通过已有无线 `radio_link/start_linux.sh` 拉起独立会话，默认bridge；`--radio-mode native`切换新发送进程输入，已有进程原样复用。`--no-radio`仅运行CLI；`--check`、`--status`、`--stop`和`--build-only`不联启。有桌面时新发送端在独立GNOME终端运行，无桌面时回退独立后台进程；已有发送端打开日志查看窗口。输出追加到 `.runtime/fsd_radio/linux.log`，窗口适配为 `fsd_startup/radio_window.sh`，与CLI车辆进程登记解耦，CLI退出不停止无线。独立启动无线仍用原脚本；发送端持有 `sender.lock`防重，依赖/启动失败只提示，不阻断CLI。单独结束无线使用其PID的SIGINT；CLI下次启动可再次拉起，不会周期性自动重启。

E22低带宽双向遥测与Windows Foxglove显示可选组件见[radio_link部署说明](../fsd-visualization-code/radio_link/README.md)。它独立启动，Linux只消费既有ROS显示/定位数据，Windows默认COM3；不由全车入口自动启用。Linux串口通过radio.yaml配置或启动参数覆盖，避免占用惯导端口；原生显示输入与bridge输入须按实际运行模式选择。当前要求串口115200 bps 8N1、模块空速62.5 kbps；当前软件双向合计预算3000 UART B/s，显示仍采用采样几何，首次接入需完成说明中的串口、ROS/TF、空口与Foxglove验收。

历史版本radio_link 0.2.0使用必要几何schema 2（当前无线0.7.4、几何schema 3），升级时先更新/重启PC接收端，再更新/重启Linux发送端；保留双端一致的fixed_frame和量化步长，PC样式在`radio.yaml:foxglove.styles`配置。旧PC不能解码新几何消息，新PC兼容旧消息，升级后应通过`/radio/layers`中各图层的`schema`字段确认已收到schema 2。

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

### 3.1 FSD 编译规范（2026-11-09）

ROS 2 Humble 的构建工具和消息 Python 扩展统一使用系统 `/usr/bin/python3`（本机为3.10）。Conda 可以提供相机进程专用推理依赖，但不能作为 ROS 消息扩展的编译解释器、头文件或链接库。构建前停止待替换的运行实例，在未激活 Conda/venv 的终端操作；只修改 `src`，不编辑 `build/install` 副本，不清空整个工作空间或运行日志。

仅指定 `Python3_EXECUTABLE` 不足以约束 Humble 的全部消息生成链：还要指定旧接口 `PYTHON_EXECUTABLE`、`PYTHON_INCLUDE_DIR` 和 `PYTHON_LIBRARY`。系统工具目录应排在 PATH 前面，避免命中 `~/.local/bin/cmake` 或 Conda 工具；构建时设置 `PYTHONNOUSERSITE=1`，避免用户级 Python 包干扰。已有错误缓存用 `--cmake-clean-cache` 重新配置，必要时加 `--cmake-clean-first` 重编对应包；不能靠修改 `.so` 文件名修复 Python ABI。

本机标准命令如下。首次构建或修复消息产物时先执行消息包构建，成功后再构建业务包。华测消息依赖的已有安装环境按实际部署加载；首次部署需先完成华测工作空间构建。

```bash
cd /home/wuta/WUTA2026_FINAL/WUTA-FSD/ros2_ws
source /opt/ros/humble/setup.bash
source /home/wuta/WUTA2026_FINAL/humble-chcnav-cgi_ros2pkg/install/setup.bash
export PATH=/usr/bin:/bin:$PATH
export PYTHONNOUSERSITE=1
/usr/bin/python3 --version
command -v cmake

fsd_cmake_args=(
  -DPython3_EXECUTABLE=/usr/bin/python3
  -DPYTHON_EXECUTABLE=/usr/bin/python3
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.10
  -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so
)
/usr/bin/colcon build --base-paths src --symlink-install --parallel-workers 2 \
  --packages-select wuta_msgs autoware_msgs \
  --cmake-clean-cache --cmake-clean-first --cmake-args "${fsd_cmake_args[@]}"
source install/local_setup.bash
/usr/bin/colcon build --base-paths src --symlink-install --parallel-workers 2 \
  --packages-select camera_detection detection_fusion \
  --cmake-clean-cache --cmake-args "${fsd_cmake_args[@]}"
```

上例业务包依赖已安装时可用 `--packages-select`；首次构建应按实际依赖使用 `--packages-up-to`，或去掉包选择参数构建完整 FSD。日常增量构建可省略清缓存/清目标参数，保留系统 Python 参数。若修改消息定义，应同步重建实际消费者及相关 overlay，重启后使用同版消息；仅修复相同消息定义的 Python ABI 不代表接口发生变化。

当前感知运行组合为 **Python YOLO 检测 + C++ 双目深度适配 + C++ 融合**。`camera_detection` 是混合包，构建时会安装 Python 入口并编译可用的 C++ 目标；不能用“包编译成功”推断运行后端。当前 `camera_detection.yaml` 使用 `yolov8sp2-int8.engine`，硬件 launch 的 `detector_backend=auto` 选择 `yolov8_node`；`adapter_backend=cpp` 和 `fusion_backend=cpp` 分别选择 `stereo_detection_adapter_cpp`、`detection_fusion_node_cpp`。C++ 图像检测目标 `lwdetr_tensorrt_node` 仅支持 LW-DETR engine，不能直接替代 YOLO。模型或 launch 参数变更后重新核对实际选择。

构建完成后必须验证消息类型支持，不能只检查 Python 消息类能否 import。以下检查不创建 ROS 节点、不连接硬件：

```bash
source install/local_setup.bash
/usr/bin/python3 - <<'PY'
from rosidl_generator_py import import_type_support
from rclpy.type_support import check_is_valid_msg_type
from wuta_msgs.msg import ConeArray, CameraConeDetectionArray, MissionState
from autoware_msgs.msg import Lane, Command

for package in ('wuta_msgs', 'autoware_msgs'):
    print(import_type_support(package).__file__)
for message in (ConeArray, CameraConeDetectionArray, MissionState, Lane, Command):
    check_is_valid_msg_type(message)
print('ROS message type support OK')
PY
```

本机输出应指向当前工作空间的 `cpython-310` 扩展。若出现 `UnsupportedTypeSupport`、`No module named ...s__rosidl_typesupport_c`，先核对消息包 `CMakeCache.txt` 中的 `PYTHON_EXECUTABLE`、`PYTHON_INCLUDE_DIR`、`PYTHON_LIBRARY`、`PYTHON_SOABI` 以及实际扩展后缀，再按上述流程修复。2026-11-09 的启动失败正是消息扩展编成 `cpython-314`，而 YOLO/ros2cli 使用 Python3.10；重编两个业务包不会自动修复已有消息依赖。

2026-11-09 已用上述系统 Python 参数重编两个消息包，5种消息类型支持离线检查通过。相机/融合包构建通过；融合既有测试通过，相机有1项测试因测试环境未加载 ONNX Runtime 失败。此记录不代表实车全栈验证通过。

#### 2026-11-10：从备份恢复并全量重编

已将 `backup/WUTA-FSD` 中的源码、配置和说明恢复到当前 `WUTA-FSD`，按内容核对402个文件一致；未复制备份内的Git元数据，未删除原有历史日志或额外文档。恢复前的controller/path_generator两份本地调参文件及差异保存在 `.runtime/fsd_restore_20261110/pre_restore/`；当前参数采用备份值。

本次在 `env -i`、未激活Conda/venv的环境中，按3.1节同时指定新旧Python参数，使用系统Python3.10.12和 `/usr/bin/cmake`。先清缓存/清目标重建两个消息包并验证类型支持，再对全部17个包执行 `--cmake-clean-cache --cmake-clean-first` 全量构建，17包成功。原有12个Python3.14扩展文件/安装链接已删除，build/install内未发现 `cpython-314` 或 `python3.14` 残留。纯C++包报告未使用旧Python变量、TensorRT头文件报告弃用警告，不影响本次构建成功。

逐包核对17份CMake缓存中的解释器、头文件和库均为系统3.10；wuta_msgs、autoware_msgs及robot_localization的实际类型支持均加载当前工作空间的 `cpython-310` 扩展，22种消息/服务请求/响应类型通过类型支持及序列化往返检查。29个已安装ELF可执行文件的动态库检查无缺失。相机18项、融合11项Python用例通过；CAN只收/发车保险/制动台架3项及控制器PID 1项CTest通过，历史brake_bench_prepare保持禁用。未启动实车入口、传感器或真实CAN。

**依赖隔离注意：** 相机测试加载 `HARDWARE_DEPS` 下的ONNX Runtime；YOLO使用系统Python3.10，并由原模型后端加载 `miniconda3/envs/tensorrt/lib/python3.10/site-packages` 内的推理依赖，PyTorch/TensorRT/Ultralytics导入检查通过。Python融合检查和测试只加载系统NumPy/SciPy，不加载相机依赖目录；本机 `HARDWARE_DEPS/scipy` 与系统NumPy1.21.5不兼容，向Python融合全局注入该目录会导致导入失败。当前默认C++融合构建和动态库检查通过；本次未更改共享依赖目录或切换后端，若改用Python融合须先隔离该目录或解决依赖兼容性。

完整命令、构建/测试输出、恢复校验及逐包Python审计保存在 `.runtime/fsd_restore_20261110/`，全量colcon日志在 `WUTA-FSD/ros2_ws/log/restore_20261110_full/`。以上为本次离线构建和验证结果，不代表实车验收。

### 3.2 现有启动入口

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

当前根脚本的 FSD 构建分支只显式传入 `Python3_EXECUTABLE`，尚未自动落实3.1节全部参数与消息类型验证；遇到已有 Conda 缓存时，先按3.1节完成修复，不把 `--build-only` 成功当作 Python ABI 正确的证明。

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

## 无线0.3.0源时间升级（历史部署记录）

先更新并重启Windows接收端，再在Linux主项目根`git pull --ff-only`和`git submodule update --init fsd-visualization-code`，重新运行无线setup与原模式的start_linux.sh。沿用已核对的E22串口和native/bridge模式，勿同时开两个读者。`/radio/layers`确认schema=3后导入子模块`radio_link/foxglove_layout.json`；速度路径改为`/radio/speed.speed_m_s`。默认本机增加3秒乱序缓冲，不提高160字节/秒预算。全纳秒与逐对象时间会增加包长，原刷新间隔仅为调度目标，成本、字段、源时间缺失和混合时钟边界见[无线说明](../fsd-visualization-code/radio_link/README.md)。本轮仅本机开发验证，Linux由使用者拉取部署，不代表已完成实机ROS/TF与双端schema 3无线验收。

## CLI无线日志与传感器（0.5.1历史部署记录）

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

## 配置文件与CLI清单的边界（2026-10-20）

标准根入口默认从FSD源码config加载核心11模块，与CLI预检查清单同源；CLI“模块配置”仅显示上次预检查写入的manifest，不查询在线参数。外部修改YAML后先重做预检查；主要算法参数默认需要重启节点，ROS参数写入成功不能证明算法内部已更新。显式文件/模型/CAN/串口及运行目录覆盖、额外驱动/外参配置见[配置核对](FOXGLOVE_PARAMETERS.md)。Foxglove Parameters尚未接入Linux YAML读写，此文档仅给出设计，没有改动当前节点参数。

## 无线0.7.0：CLI参数与操作控制

Linux主仓库 `git pull --ff-only` 后执行 `git submodule update --init --recursive`，配套更新CLI、显示与宿主tools。重启CLI与独立无线程序；CLI保留原部署选项，菜单常驻可用 `bash start_fsd_manager.sh --menu --skip-build`，无终端可用 `--headless --menu --skip-build --no-rviz`。不要同时开启第二个CLI或串口读者；不带--menu仍按原行为自动启动。

Windows重启 `radio_link/start_windows.ps1`（默认COM3/115200），导入新版foxglove_layout.json并打开“CLI 控制与参数”。先等待在线状态/目录，再选择selection/module、编辑草稿、点击保存；保存不自动重启相关进程。检查/启动/停止按钮均调用原CLI业务；停止FSD会保留CLI和独立无线程序供后续启动，原管理入口--stop仍清理并退出管理器。

PC/Linux系统时间保持同步，CLI源心跳大于10秒或未来偏差超过5秒会拒绝新操作。radio_linux只保存Linux的radio.yaml，双端链路参数不自动同步。具体来源、字段、事务、超时和本地E键用法见 [参数与操作说明](FOXGLOVE_PARAMETERS.md)。本轮按用户要求未执行测试、构建、WSL或实机操作，未重启当前接收程序或现场CLI，需部署后加载。

### 0.7.1指令独立优先级

配套更新宿主、CLI与显示子模块，并重启CLI、Linux无线及Windows接收端。先等待当前操作完成；重启接收进程不会取消已发送指令。指令、其RPC确认和执行结果优先于日志与可视化；参数批量读取仍走普通数据队列。Windows单独升级不能改变旧Linux的发送优先级。预算仍为3000 UART B/s，原串口/模式保持，CLI既有检查和启动不被强行中断。按要求未运行测试、WSL或实车指令。

本机部署记录：Windows接收端已于2026-10-20 21:36重启加载0.7.1，沿用COM3/115200/native，Foxglove已自动重新连接；重启前只读确认无在途检查/启动/停止/配置保存请求，未发送实车操作。Linux由使用者拉取并重启后才具备对应优先级。

0.7.2 CLI状态指示：本机Windows接收端已重启加载PC派生display_state，COM3/115200/native保持；重启前只读状态为等待操作、无在途请求。Foxglove重新导入radio_link/foxglove_layout.json或output/FSD_E22_CLI与传感器.json后，CLI控制页顶部出现背景状态面板。Linux无需为此次面板升级或重启；未发送实车操作或执行测试。

当前只换布局无需重启程序：导入radio_link/foxglove_layout.json或output/FSD_E22_CLI与传感器.json；CLI状态/操作、车辆任务和RSSI在“CLI 日志与控制”，专用传感器信息在“传感器概览”，参数编辑/保存在“参数控制”。新增PC代码字段仍需加载新版接收端。

最终布局导入后，页名为“CLI 日志与控制”“传感器概览”“参数控制”：专用传感器表格和原始详情仅在传感器概览，CLI页已移除这些组件。无需重启程序，仅重新导入JSON。

0.7.3本机部署：Windows接收端已重启加载操作中显示，重启前只读确认CLI运行中、active为空且无pending请求。COM3/115200/native保持，未触发CLI启动/停止/检查或配置保存。导入新版JSON后橙色操作中规则生效；Linux无需为此显示变更重启。

### 0.7.4 RSSI指示器部署

部署时发现用户已于22:06重新启动旧Windows接收端；只读确认CLI已停止、active为空且无pending后，已重启接收端加载本版，沿用COM3/115200/native。导入最新版布局后，RSSI指示器在“CLI 日志与控制”的车辆任务表下方。未发送CLI操作或改写模块寄存器。Windows模块已开启数据RSSI，本版serial.append_rssi_pc=true；Linux模块开关未知，append_rssi_linux保留false，不能因Windows开启就假定Linux也开启。

两端模块分包必须保持240字节，serial.module_packet_bytes=240只记录设置，不自动读写硬件。解析前提为一条应用帧对应一个模块无线包，不支持在应用帧中间插RSSI或多个应用帧共用一个尾字节。关闭硬件RSSI时同步关闭软件解析开关。先显示原始0..255数值，包尾dBm换算和硬件追加粒度未实测，不显示虚构强弱等级。指示器等待/过期/断开有明确提示。Linux若不启用自身RSSI解析，无需为Windows本地指示器升级；其他既有功能升级要求不变。本轮未运行测试、WSL或实机操作。

部署只读观察：新版接收端/radio/link.rssi返回enabled=true、valid=true、raw=197（0xC5），同一快照samples=326、packets_rx=326、framing_errors=0。仅为当时接收观察，不代表覆盖所有分包/换算/断链场景的实机验收；未运行自动测试、WSL、构建或模块参数写入。

### 0.7.5 Windows自动识别串口（2026-10-21）

默认运行start_windows.ps1即可按CH340 1A86:7523寻找E22适配器，无需确认COM号；断开重连会重新枚举。多个同型号适配器时按错误列出的端口手动-Port覆盖，或用radio.yaml:serial.pc_usb.location/serial_number限定；无设备则等待，不扫描打开其他串口。匹配的是USB芯片，不认证E22射频型号。现有进程不会自动加载，下次正常停止后用新脚本启动，勿开第二个接收端。本轮未重启程序或执行设备操作。

0.7.6参数状态面板：正常停止Windows接收端后，用新版start_windows.ps1启动并重新导入布局；Linux无需为此显示变更重启。顶部参数状态区分读取、未保存、保存中、已保存、失败和超时，详情见页下部。现有接收进程本轮未重启，未触发参数修改或实车操作。

0.7.7退出码指示：更新主仓库及CLI/显示子模块，等待在途操作结束后按原流程加载新版CLI、Linux无线发送端、Windows接收端并导入布局。任务结束时CLI指示器显示退出码解释；未配套更新旧CLI时仅兼容phase明确带码的情况。本轮未重启任何程序或触发任务操作。

0.7.8 Windows自动寻找Python：日常直接运行start_windows.ps1，优先使用项目.venv，再筛选本机其他64位Python 3.10+且依赖齐全的环境；首次部署或缺少依赖运行setup_windows.ps1即可自动选择基础解释器并安装到项目.venv。支持-Python指定路径，启动输出实际解释器；不依赖当前终端目录或PATH中的python默认命令。无需为本次脚本更新重启已有接收端；本轮未执行测试、安装、WSL或硬件操作。

0.7.9 Windows exe：本机output/FSD_E22_Receiver.exe内置Python/PC依赖，仍需USB驱动与Foxglove。先等待在途操作结束并Ctrl+C退出原接收端，再双击exe；不可与脚本接收器同时占用COM/8765/8766。默认内置native配置，可--mode bridge覆盖。自定义时执行--export-config <目录>导出完整树，放exe旁config目录后自动读取config/radio_link/radio.yaml；也支持--config明确指定。不因更新exe覆盖现有外部配置。构建入口为显示radio_link/build_windows.ps1，打包会安装固定PyInstaller；本轮只构建、未启动exe或触发设备操作。

## CLI 静态制动台架（2026-10-21）

此入口操作真实CAN，当前默认包含5%正驱动请求，不能按“驱动电机不转”理解。pre_drive_opening=0可恢复中性等待。更新后重新打开CLI，在停止状态按 **M** 循环选择“正常→制动台架→有人跑动记录→正常”，顶部同时显示选择与运行模式；按 **C** 检查、**S** 启动。台架只启动ZLG桥、CAN接口、控制器、任务机和只读监控，不启动相机/雷达/惯导、定位、建图、规划或RViz。CLI选项只保留于当前会话，不写入正常配置；运行/日志未清理完成时禁止切换。

软件启动后，仍需真实VCU直线加速档位与之后的RES GO；CLI不发送选档、GO或驾驶Command。任务机台架检查结论只表示豁免了传感器，不表示设备在线。错误赛项保持等待。GO放行后以 **5%** 开度向VCU请求微驱动 **5秒**，再以 **50%** 开度向VCU请求制动 **10秒**；不等待直接电机初始化，最后持续中性并进入FINISH。全程目标速度及转向为0；参数唯一来源为控制器源码config的 `acceleration.bench_brake.{pre_drive_opening,delay_sec,hold_sec,opening}`，可通过CLI E键编辑，修改后重启生效。该开度是协议请求，不是实测制动力或电机行程百分比。

结束后按 **X** 停止并等待清理，按 **M** 循环选回正常模式，再 **C/S** 正常检查与启动。FINISH和重复GO不重复执行动作；再次台架测试也需停止后重启。重新打开CLI默认正常模式（显式传入`--brake-bench`除外）。根脚本的`--brake-bench`只供同一入口透传，不另建台架启动器；与`--sensor-only`互斥。`--brake-bench --print-config`为无硬件配置审计。

控制器只在观察到IDLE/READY后的有效EXPLORE边沿启动；活动中重启控制器不自动补做动作。急停、运行模式不一致、任务状态超过0.5秒未更新会中止并中性保持，失联恢复也不续做。CAN台架分支在路由、Command或MissionState超过0.5秒未更新时归中性（100ms周期检查，最长约0.6秒）；目标3最多允许5%正驱动，目标1仅允许负制动，转向强制归中。停止流程先停止控制器并保持CAN至少0.7秒，再清理其余进程；CAN正常析构再发中性。CAN进程被强杀、硬件掉电或通信断开时不能保证该帧送达，仍以VCU硬件行为为准。

本次新增 `MissionState.brake_bench_mode`，首次使用前必须重建消息及全部直接消费者：controller、mission_manager、can_interface、boundary_detector、path_generator、localization_manager、ndt_localization；其他overlay须使用同版wuta_msgs。正常全栈构建仍用既有build-only流程。台架按需构建只构建惯导消息类型和三个必要节点；不能据此认为其他消费者也已更新。已运行的CLI/ROS实例须停止后加载新环境。

离线验证入口：`fsd_startup/test_brake_bench.py`、`tools/test_brake_bench_ros.py`及can_interface的`brake_bench_can` CTest。ROS测试必须显式设置独立非零ROS_DOMAIN_ID和ROS_LOCALHOST_ONLY=1，不运行根硬件入口；CAN测试用内存设备替身验证真实编码/看门狗，无物理或虚拟总线。实际电机动作需另做实台验证。

初版台架历史离线结果（不作为当前微驱动验证结论）：CLI/配置/GNSS/日志共76项pytest通过（日志用隔离模式及模拟空模式缓存，避免读取宿主真实CAN会话）；独立ROS域的7项台架回归通过，包含真实5秒等待和10秒制动时序；controller PID与内存CAN两项CTest通过。消息与上述直接消费者构建成功；Shell/Python语法及差异空白检查通过。未启动实车入口、真实CAN桥或传感器，未验收制动电机实物动作。

## 人工驾驶记录模式（2026-10-22）

使用现有CLI：先停止并清理上一轮，按 **M** 循环选择正常→制动台架→有人跑动记录→正常，确认顶部为“有人跑动记录（CAN只收不发）”，按 **S** 启动。无需RES GO，车辆由人驾驶；结束按 **X**，待清理完成后按M继续循环切换模式。三种模式互斥，已移除R键入口。也可在CLI启动时传入 `./start_fsd_manager.sh --manual-record`；根脚本支持同名参数，仍复用原监管器。

传感器、定位和感知部署要求沿用正常入口；定位输出仍需通过华测桥自身发布条件，不降低GNSS原点/数据有效性要求。已有驾驶节点或ZLG桥会阻止本模式启动；请在所属入口停止，启动器不自动杀死或复用外部进程。本轮启动自己的只收桥并强制逐帧日志，正常/台架也不得复用只收桥。CAN接口和桥都禁止应用CAN发送，包括心跳，不仅是驾驶开度归零；这不是厂商电气静默模式配置。

日志位于 `wuta_logs/manual_record/<时间>/`，内容与缺失字段说明见[LOGGING](LOGGING.md)。本模式未新增rosbag。首次实车验收应核对CLI模式、VCU接收计数/0x501日志、实际轨迹及总线发送情况；离线回归不能替代硬件验收。

本次离线验证：ZLG桥make、can_interface/mission_manager定向colcon构建完成；CLI人工记录/台架/日志/启动/界面回归通过，源码模块配置检查在加载overlay并禁用用户Python包后通过；CAN桥重连编排与只收设备替代测试通过。CTest内存CAN验证接收解析、回调/周期/退出零发送及既有台架保护；隔离ROS域217回放验证IDLE人工记录持续写driven.csv、命令字段nan、GO/完成不激活任务，以及正常模式仍保留原门控。未启动实车传感器、USB桥或真实CAN；硬件只收与总线实测尚未执行。

### 历史：台架GO后的制动电机前置帧（2026-10-22，已停用）

> 以下是历史部署记录，不适用于当前台架。当前FSD不发送NMT 0x000或0x603直接电机报文，也不创建初始化握手接口；请按本页“台架制动前微驱动”部署。

> 2026-10-23早期临时状态（已由下述延时后初始化方案替代）：按用户要求，parseResFrame中调用prepareBrakeBench的入口已暂时注释，以下0x603前置流程仅作历史记录，当前不会执行。GO直接进入原任务机，台架仍按原控制器参数等待5秒、以50%制动10秒、释放中性；0x210和0x301保持，FSD不再额外发送0x603。


制动台架在新鲜的直线加速READY及台架自检ok后，收到真实RES GO变化沿时，由原can_interface向标准CAN ID **0x603** 连续写入三组A/B报文（DLC均为8，共6帧）：

```text
A: 2B 71 60 00 18 FC FF FF
B: 2B 40 60 00 1F 00 00 00
顺序：A → B → A → B → A → B
```

不额外插入帧间等待。全部写入成功后才向原任务机转发GO；任务机进入EXPLORE后原控制器开始5秒中性等待，再执行10秒50%制动，最后中性释放/FINISH。每次运行只发一组6帧，重复GO不重发；正常模式及人工记录模式不增加0x603。任一写入失败立即停止后续帧，不转发本次GO，并发布锁存急停；需检查日志、停止并按原急停流程重新开始。尚未READY或尚无自检结论的早到GO不保留，需就绪后重新按GO。

写入成功只表示SocketCAN发送调用成功，本流程未解析制动电机SDO应答，不表示电机已确认执行。实际总线发送和电机动作以0x603抓包及硬件反馈为准。更新后重新启动CLI车辆进程使新CAN节点生效；无需改变原5秒/10秒/50%参数。

无线0.7.10模式按钮：主仓库git pull --ff-only后git submodule update --init --recursive，等待在途操作完成，按原部署流程加载新版CLI与Linux无线。Windows退出旧接收端再运行新版脚本/exe，并重新导入新版布局。原检查/启动按钮行增加“切换模式”：正常→制动台架→有人跑动记录→正常；仅停止并清理完成后允许。模式附加在原CLI状态指示器，运行时显示active_mode，停止时显示selected_mode，未知/离线不假定正常。切换后按新模式重新检查再启动，切换不会自动启动。运行模式与赛项/VCU档位不同。旧CLI或旧发送端缺字段时显示未知模式并拒绝切换；不能仅更新Windows。本轮未测试、未启动接收器或执行远程操作；Windows exe仅重新构建。

### 历史：等待后初始化再请求VCU制动（2026-10-23，已停用）

> 以下是历史部署记录，不适用于当前台架。当前FSD不发送NMT 0x000或0x603直接电机报文，也不创建初始化握手接口；请按本页“台架制动前微驱动”部署。

CLI操作不变；GO后先保持5秒中性，再由FSD向制动节点3发送NMT及完整转矩模式初始化，逐帧等待0x583成功应答并回读6061/6041。初始化时清零目标转矩，0x210保持中性；确认后才开始原50%制动10秒，结束归中性/FINISH。初始化耗时不占用10秒保持，不再执行历史A/B交替六帧。

重建并部署controller和can_interface，停止旧进程、加载新overlay后再从CLI启动；仅编译build目录不会自动更新install。设备参数在can_interface.yaml的brake_bench.initialization：profile_velocity=10（P45=0时1rps，现场需核对P45），torque_slope=1000‰/s。延时/开度/保持仍沿controller.yaml单一配置。每步SDO超时500ms，写失败/abort/回读不符走急停并保持中性；controller等待结果超过8秒锁定中性。重复GO不重做，下一轮须停止清理并重启。

现场应明确同一0x603的控制归属：原日志显示车端还存在其他发送者，本功能不能禁止VCU并行SDO，也不能仅凭匹配应答证明初始化未被覆盖。正常模式不启用本初始化；人工记录保持只收。此次仅编译两个生产节点并维护离线测试源码，未执行测试、未操作硬件；上文历史测试通过结果不作为本次更改的验证结论。

### 历史：电机直控部署（已停用）

> 以下是历史部署记录，不适用于当前台架。当前FSD不发送NMT 0x000或0x603直接电机报文，也不创建初始化握手接口；请按本页“台架制动前微驱动”部署。

恢复初始化，controller.yaml新增post_init_delay_sec=3.0；仍delay_sec=5、hold_sec=10、opening=0.5。停止旧进程并重新加载已构建安装的controller/can_interface，通过原CLI台架入口启动。初始化成功后等待3秒，0x603发送500‰转矩+启动，10秒后清零+暂停；0x210全程中性。实际电机转向、释放行程及并行VCU发送者覆盖需现场确认，软件清零/暂停不代表主动回到机械中位。未运行测试或操作硬件。

历史最小代码部署（当前编译排除）：重建安装controller/can_interface并停止旧进程再从CLI启动。GO后5秒发送配置8帧（含NMT），最后启动帧延后3秒，随后原10秒保持。初始化不再清零转矩或读取6061/6041，速度02、广播01 00均按原文件保留；不要用旧profile_velocity/torque_slope配置推断报文。此轮只编译安装，未运行测试或硬件。

### 历史：双路台架部署（已停用）

> 以下是历史部署记录，不适用于当前台架。当前FSD不发送NMT 0x000或0x603直接电机报文，也不创建初始化握手接口；请按本页“台架制动前微驱动”部署。

参数delay_sec=5、hold_sec=10、opening=0.5、inter_stage_delay_sec=3；原post_init_delay_sec已替换。停止旧节点并加载新版controller/can_interface，经原CLI启动。先VCU50%10秒，释放后3秒间隔内配置电机，配置成功且间隔到期后才发EF并保持10秒。阶段2目标转矩仍为正1000，不受opening比例缩放；直接电机阶段VCU中性。仅编译安装并维护离线测试源码，未运行测试或硬件；其他总线控制器的并行SDO不由FSD禁止。

## 临时屏蔽惯导与恢复（2026-10-25）

启动菜单使用 `./start_fsd_manager.sh --menu --no-ins`。根入口也支持 `./start_full_fsd.sh --no-ins`，仅检查可用 `./start_fsd_manager.sh --check --no-ins`。该模式不启动华测驱动、不做串口读写/独占检查或串口ACL处理，不等待惯导及其依赖的定位首帧；界面显示跳过。构建仅保留其他节点需要的惯导自定义消息包，安装环境仍需存在。

恢复：去掉`--no-ins`，或重新以 `./start_fsd_manager.sh --menu --with-ins`启动。开关重复时最后一个生效，不修改惯导配置，也不停止外部惯导。`--no-ins`仅本次向任务机覆盖`check_imu=false`，不因缺少IMU心跳触发设备故障；`--with-ins`恢复读取YAML配置值。其他设备自检、GNSS与发车门控仍按真实输入运行，缺少定位/GNSS时仍可能等待；不等于允许发车。若雷达/相机自检也已关闭，任务机现有逻辑不会发布设备自检结论，不伪造自检通过或绕过CAN对结论的等待。本次只修改源码并检查语法，未操作硬件、未运行测试。


### 直线加速方向拟合部署（2026-10-25）

重新构建FSD中的boundary_detector、path_generator后，原start_full_fsd.sh入口继续加载这两个包源码config下的YAML。默认acceleration.use_cone_direction=true，需真实定位及同场地红左蓝右地图、每侧≥3个锥桶和≥4米重叠跨度；检查日志Acceleration lines及centerline_viz的左右边界/箭头。READY可预览拟合，活动后固定路径冻结。拟合失败时日志输出红蓝数量和验收要求，规划等待；不能把GO已接收当作路径已经有效。现有台架制动配置与本功能互相独立，上车前按原台架退出流程恢复正常车辆配置。

距离和速度保持path_generator.yaml中的acceleration.anchor/speed值；请按实际赛项另行校准，本功能不会将最远锥桶作为终点。需撤销此功能时将acceleration.use_cone_direction设为false即可恢复原锚点方向，边界显示不参与最终路径。无需改动惯导驱动或定位输出。

### 感知 Reliable 部署（2026-10-25）

更新主仓库与FSD、CLI、显示子模块后，构建FSD的`lidar_detection camera_detection detection_fusion mission_manager`及显示工作空间的`fsd_visualization`。使用现有依赖环境和构建参数；停止旧算法与显示进程、加载新overlay，再重新打开CLI/FSD及独立RViz。运行中的节点不会因源码或构建更新自动切换QoS。

实车RoboSense点云和ZED RGB/注册深度/CameraInfo已提供Reliable；仿真输入及旧标注图发布者也须提供Reliable才能匹配新订阅。无需修改相机频率、深度模式或匹配窗口。完整深度见[ROS_INTERFACE](ROS_INTERFACE.md#感知链路-qos2026-10-25)。

隔离回归（从主仓库根目录执行，加载现有ROS依赖环境）：

```bash
source /opt/ros/humble/setup.bash
source WUTA-FSD/ros2_ws/install/setup.bash
source fsd-visualization-code/install/setup.bash
/usr/bin/python3 -m pytest -q tools/test_sensor_log_routing.py fsd_startup/test_startup.py fsd-visualization-code/tests/test_interfaces.py::test_configuration_profiles
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m pytest -q tools/test_camera_reliable_ros.py WUTA-FSD/ros2_ws/src/perception/camera_detection/test/test_stereo.py WUTA-FSD/ros2_ws/src/perception/camera_detection/test/test_yolo.py
```

本次相关构建通过，两组回归分别14与21项通过。合成相机回归不启动硬件/CAN/控制器；隔离ROS端点已核对融合及任务机的感知输入为Reliable。未重启实车全栈，仍需重启后核对实际帧率、处理覆盖率和帧龄，不能将离线结果作为实车不丢帧保证。

Mac接收端：GitHub Actions构建两套macOS15原生产物，Apple芯片选arm64、Intel选x86_64；完整解压到可写目录，先退出其他接收程序，再双击Start_Receiver.command。无需Python，仍需Foxglove及必要USB驱动；连接地址/布局与现有PC相同，终端可--mode bridge或--port /dev/cu.usbserial-XXX覆盖，Ctrl+C退出。配置导出与日志目录保持原PC接口。未Developer ID签名/公证，首次可能需系统隐私与安全性手动允许；不关闭全局保护。构建状态与Mac Release以GitHub实际结果为准，未启动应用或验收设备。

## 独立只读复盘（2026-10-31）

正常全车启动已由根脚本在控制器/任务机前自动启动只读采集器（--no-rviz），沿原观察进程退出管理；无需另开第二个采集器。仅在需要独立采集时，于运行已创建但GO之前执行`bash fsd-visualization-code/review.sh --run-dir <本次含run.json的绝对目录>`。沿用相同ROS_DOMAIN_ID和已安装ROS/FSD消息；独立命令默认额外打开专用RViz，可加--no-rviz。采集结果统一写入工作空间test_maps下的独占目录，可用--maps-dir覆盖；来源run目录只保留artifacts/mission_review.log和mission_review.ready。独立采集由其终端Ctrl-C停止，只关闭自己打开的RViz；不要将旧run目录用于新运行。

整套系统停止后可运行`bash fsd-visualization-code/review.sh --replay <采集目录/observations.jsonl>`，默认最后任务段，--segment指定段号。此入口只读，不启动CAN、驱动、控制器或任务机，原发送配置不改。停稳判断只是复盘收尾；无输入不能假装通过。完整说明见显示子模块README“独立任务复盘”。

## 台架制动前微驱动（2026-11-01）

当前controller.yaml的`acceleration.bench_brake.pre_drive_opening=0.05`，真实GO后前`delay_sec=5.0`秒请求5%驱动，而后`opening=0.5`制动`hold_sec=10.0`秒并完成。该参数允许0～0.05，0.0恢复原中性等待。微驱动不走惯导速度PID；开度不代表实测转矩/车速。必须同时构建部署controller和can_interface，否则旧CAN会将正开度或目标3归中性。沿原CLI制动台架入口启动，任务机/根启动器不修改；GO前中性，直接电机阶段继续暂停。

微驱动目标3独立受5%硬上限限制，制动目标1仍禁止正驱动。原500ms门控、急停、自检失败与析构中性保留；路由切换先清旧命令，可有短暂中性帧。离线验证使用配置审计、控制器ROS测试和CAN内存替身，不连接CAN设备。需实台核对5%是否克服VCU死区及实际作用。

2026-11-05离线验证：配置及复盘纯逻辑28项、复盘ROS 4项、台架ROS 11项及CAN内存替身测试通过。Python检查禁用用户级包以避开本机NumPy/SciPy冲突；ROS使用独立非零domain及localhost。使用现有C++构建产物，本次未重新编译、未操作硬件。复盘的两个已知问题及验证边界见[test_maps说明](../test_maps/README.md#已知限制2026-11-05核对)。
