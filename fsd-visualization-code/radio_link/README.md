# FSD E22 低带宽双向遥测

工控机 Linux 从现有 ROS 2 读取显示数据，经 E22 透传无线链路发送；Windows 从 COM3 接收，在本机启动 Foxglove 官方 SDK 服务。Foxglove 连接 `ws://127.0.0.1:8765` 即可显示，无需在 Windows 安装 ROS。两端均可收发独立 JSON 扩展消息。

无线模块按用户确认的**透传、9600 8N1、空速 2.4 kbps**使用。程序不发送模块配置命令，不修改频率、密钥、地址、工作模式或功率。Linux 串口没有默认值，必须明确指定无线模块设备。

## 配置检查结果

本目录读取原有 `../src/fsd_visualization/config/topics.yaml` 和 `fsd.rviz`，复用原 `configuration.py` 渲染 native / bridge 的实际话题，保持原文件内容不变。

| 检查项 | 结果与处理 |
| --- | --- |
| 系统 | ROS 2 Humble；不是 ROS 1 rosbridge |
| 固定帧 | `map`；非 map 的几何数据在 Linux 按源时间戳查询 TF 后转换 |
| RViz Frame Rate=30 | 仅是本地重绘目标，不能据此要求无线每秒发送30帧 |
| 原始点云 | RViz 模板是 `/hesai/pandar`，YAML 是 `/rslidar_points`，启动渲染会覆盖模板；两者都不订阅或传输 |
| 相机图像 | `/camera/yolo/image_annotated`，完全排除 |
| 速度 | 原 YAML 为 `/localization/velocity`，项目当前实车接口是 `/chcnav/velocity`；只在 radio.yaml 覆盖，不改原显示配置 |
| native 模式 | 直接消费给定配置中的7个 MarkerArray 图层 |
| bridge 模式 | 复用原配置生成器映射 `/fsd_visualization/*`，必须已有显示桥在运行 |

运行配置检查，不打开串口、不启动 ROS：

```powershell
.\.venv\Scripts\python.exe -m fsd_radio.cli inspect
.\.venv\Scripts\python.exe -m fsd_radio.cli inspect --mode bridge
```

### 传输内容

| 图层 | native 输入话题 | 默认限量 |
| --- | --- | --- |
| guided | `/perception/fused/guided_clusters_viz` | 最多6个 marker，每个最多12个派生簇采样点 |
| fused | `/perception/fused/cones_viz` | 最多12个 marker，每个最多12点 |
| map | `/mapping/cone_map_viz` | 最多48个 marker，每个最多64点 |
| centerline | `/planning/centerline_viz` | 最多4个 marker，每条最多48点 |
| path | `/planning/final_waypoints_viz` | 最多4个 marker，每条最多48点 |
| trajectory | `/planning/driven_trajectory_viz` | 最多3个 marker，每条最多48点 |
| control | `/control/target_viz` | 最多4个 marker，每个最多12点 |
| vehicle | `/localization/pose` 与 `/chcnav/velocity` | 位姿、平面速度；本机绘制车体箭头 |

Grid 无需无线传输，在 Foxglove 的3D面板本地开启网格。速度从真实 TwistStamped 计算 `hypot(vx,vy)`，不保留倒车符号，和原显示桥含义一致。

原生/桥接话题必须确实有生产者。无线程序不启动算法、不启动 RViz，也不凭空生成缺失锥桶、引导簇或控制目标。根 `start_full_fsd.sh` 默认用 bridge 显示；如果要对齐该运行视图，两端启动均加 `--mode bridge`。bridge 模式的原始自定义消息转换仍由原显示桥负责，本程序只订阅其 MarkerArray，不另外复制建图或规划。

项目文档提示某些部署不发布原生 fused/guided 显示话题；这时 native 图层不会有数据，不能靠无线配置修复。特别是现有显示桥的地图输入类型需与实际 `/mapping/cone_map` 核对：接口文档为 `ConeMap`，旧显示桥按 `ConeArray` 订阅；遇到该差异时使用原生 `/mapping/cone_map_viz`，或先修正原显示桥，不能把无数据误判成无线丢包。

## 带宽与显示边界

2.4 kbps 理想原始上限约为300字节/秒，LoRa 分包、收发切换和干扰还会消耗空口时间。默认把**双向合计串口字节预算设为160字节/秒**，计入帧头、Base64、CRC、ACK、轮询和重试，并保留空口开销余量。它是主机发送节拍限制，不代表已经测得160字节/秒的无线有效吞吐。

Linux 主轮询，PC 只在收到有效轮询后回复，从协议上避免两个应用各自全速发送。一次串口应用帧最多162字节，96字节数据片，默认适用于模块240字节分包；**请核对模块分包长度为240**。这是配置前提，程序不自动改写。其他分包长度应调小 fragment_bytes 并重新实测，首版不承诺其定点/分包行为。

在全部图层开启时，地图/路径的完整低带宽快照可能需要数秒至数十秒，大场景或多次重试时可能超过一分钟。radio.yaml 的 interval 是发送资格的最小间隔，不是保证更新频率。车辆位姿权重较高，但同样受链路占用影响。大幅减少 markers / points 或关闭非必要图层，才能提高其它图层更新速度；禁用输入可在个人 topics.yaml 中置空相应键，再把 radio.yaml 指向它。

几何坐标默认5厘米量化。点集按均匀索引抽样，LINE_STRIP 保留首尾，LINE_LIST 按完整线段抽样，派生引导簇显示为少量采样球点。文字最多64字符。各图层压缩快照最多1200字节，未压缩JSON最多4096字节；超限继续抽稀或舍弃部分对象，并把数量记录到 `/radio/layers`。因此本程序提供**可辨识的低带宽预览，不是 RViz 原消息的无损复制**。

目前支持 ROS Marker 的 ARROW、CUBE、SPHERE、CYLINDER、LINE_STRIP、LINE_LIST、CUBE_LIST、SPHERE_LIST、POINTS、TEXT_VIEW_FACING。保留位置、姿态、尺寸、主要颜色、有限的逐点颜色与文字；不传 MESH_RESOURCE、TRIANGLE_LIST 或外部模型资源。缺TF、空frame或非法几何数据会丢弃该对象并计数，不把传感器坐标冒充map。

## Windows 安装和运行

使用64位 Python 3.10及以上。保留整个 `fsd-visualization-code` 目录，不能只复制 fsd_radio 包，因为需要复用原有配置和生成器。

```powershell
cd D:\ebyte\WUTA2026_FINAL\fsd-visualization-code\radio_link
.\setup_windows.ps1
.\start_windows.ps1                      # COM3，native
```

若本机脚本执行策略阻止运行，可在当前会话调用 `powershell -ExecutionPolicy Bypass -File .\setup_windows.ps1`，或直接执行等价命令：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[pc]"
.\.venv\Scripts\python.exe -m fsd_radio.cli pc --port COM3
```

对齐已经运行的 bridge 视图：

```powershell
.\start_windows.ps1 -Port COM3 -Mode bridge
```

如果只需先检查 Foxglove，不插无线模块：

```powershell
.\.venv\Scripts\python.exe -m fsd_radio.cli demo
```

demo 只生成明确标记的本机测试图形，不打开COM3、不连接ROS、不发无线数据。正常运行不用 demo。Ctrl+C 退出当前程序，不停止既有 FSD 节点。

### Foxglove 面板设置

1. 打开已安装的 Foxglove，选择 Open connection / Foxglove WebSocket，地址 `ws://127.0.0.1:8765`。
2. 添加3D面板，固定显示帧设为 `map`，勾选 `/radio/scene/*`，按需本地开启网格。所有图形已在Linux转到固定帧，本机不需要额外ROS TF桥。
3. 添加 Plot 或 Gauge，用 `/radio/state.speed_m_s` 看速度；`null` 表示不可用，不是0车速。
4. 添加 Raw Messages，查看 `/radio/link`、`/radio/layers`、`/radio/extension/rx`，检查连接、新鲜度、抽样和双向应答。
5. 若 fixed_frame 在原 YAML 改为 odom/rslidar，3D面板也使用该帧。

SDK 锁定 `foxglove-sdk==0.29.2`。当前 SDK 握手使用 `foxglove.sdk.v1`，本项目的真实 WebSocket 测试已验证；若现有旧版 Foxglove 只支持历史协议而报握手失败，应更新 Foxglove。程序不重新安装或改动现有 Foxglove。

## Linux 工控机安装和运行

复制整个 `fsd-visualization-code` 到工控机。使用 ROS 2 Humble 对应的系统 Python 3.10，不用与ROS不匹配的 Conda/Python解释器。

```bash
cd /path/to/fsd-visualization-code/radio_link
# 若缺少系统venv模块，先由机器管理员安装 python3-venv。
bash setup_linux.sh

source /opt/ros/humble/setup.bash
# 按现有项目实际路径 source FSD 的 install/setup.bash；不在这里启动全车。
source /path/to/WUTA-FSD/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42   # 改成真实算法正在使用的domain

ls -l /dev/serial/by-id/
bash start_linux.sh /dev/serial/by-id/无线模块的真实设备名
```

上面的 `/path/to`、domain 和串口均须按实机填写。**不要直接猜 `/dev/ttyUSB0`：项目惯导可能正在使用它。** 推荐使用 by-id 的稳定名称。用户需有对应串口权限；本程序不修改权限、不占用已有惯导串口、不关闭其他进程。两端无线模块必须同频段、匹配信道/NETID/空速/密钥，绿色传输模式、RSSI尾字节追加关闭、模块分包240；这些条件通过厂商工具或此前配置确认。

根入口当前使用 bridge，已有显示桥时：

```bash
bash start_linux.sh /dev/serial/by-id/无线模块设备 --mode bridge
```

如果没有运行显示桥但希望看原生 marker，使用默认 native，并确认算法启用了对应 `_viz` 输出。订阅 QoS 为 Best Effort / Volatile / depth1；不会自动接收订阅前仅发布一次的历史 marker。不存在输入时不生成替代图形。

ROS bag 测试可添加 `--use-sim-time`，由现有bag发布 `/clock`、TF和数据；无线调度仍使用单调墙钟。没有相机或原始点云订阅。该包不依赖 `wuta_msgs` / `autoware_msgs` 解码，因为传输输入是标准 MarkerArray/PoseStamped/TwistStamped。

只读检查当前输入：

```bash
ros2 topic info /planning/final_waypoints_viz -v
ros2 topic info /chcnav/velocity -v
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /fsd_radio/status --once
```

## 双向扩展消息

链路扩展消息是最多约500字节的JSON对象（整个内部封装最多512字节）。建议提供唯一业务id，实现业务层幂等。队列最多8条；满时拒绝，不无限积压。扩展不会转发到 `/control/command`、任务模式、GO或急停接口；Linux只发布独立 `/fsd_radio/extension/rx`，后续新功能通过显式消费者接入。

### 本机发往工控机

Windows程序运行后，在另一个终端发送Ping：

```powershell
cd D:\ebyte\WUTA2026_FINAL\fsd-visualization-code\radio_link
.\.venv\Scripts\python.exe -m fsd_radio.cli send
.\.venv\Scripts\python.exe -m fsd_radio.cli send --message '{"type":"note","id":"test-001","text":"hello Linux"}'
```

命令返回 `queued: true` 仅表示本机排队成功，不表示远端收到或执行。轮询到来后才上行；Linux停止时PC不自发占用无线。Linux接收的所有扩展发布为 `std_msgs/msg/String` 的JSON：

```bash
ros2 topic echo /fsd_radio/extension/rx
```

Ping 会由 Linux 返回同id的 `pong`，本机控制台和Foxglove `/radio/extension/rx` 可见。图层多或链路重试时Ping可能需要较长时间，不适合作为低延迟控制通道。

也可在 Foxglove 的 Publish 面板，topic 设为 `/fsd_radio/extension/tx`、编码 JSON，发布 `{"type":"ping","id":"foxglove-001"}`。SDK只接受这个明确话题，其他客户端发布话题不会被传到ROS。

### 工控机发往本机

```bash
ros2 topic pub --once /fsd_radio/extension/tx std_msgs/msg/String \
  '{data: "{\"type\":\"note\",\"id\":\"linux-001\",\"text\":\"hello PC\"}"}'
```

本机从 `/radio/extension/rx` 接收。任意本机程序还可以连接 `127.0.0.1:8766`，发送一行JSON加LF，读取一行排队结果；默认只监听回环。该本机接口和Foxglove客户端发布共用同一个有界上行队列。

```python
import json, socket
with socket.create_connection(("127.0.0.1", 8766), timeout=3) as s:
    s.sendall(json.dumps({"type":"ping", "id":"python-001"}).encode()+b"\n")
    print(s.makefile("rb").readline().decode())
```

无线帧ACK仅证明对端链路收到了该分片，不证明业务已处理。串口重连、对端重启或序号会话切换时，待发送扩展清空并记录“投递未知”，不会悄悄重放旧扩展。内存去重也不提供跨进程重启的业务恰好一次语义。

## 时间 截断与断线行为

Linux保持有界 Marker `(namespace,id)` 缓存，处理ADD/DELETE/DELETEALL和ROS时间寿命；ROS时钟回退会清空历史。frame_locked marker按最新TF处理，其他按源时间戳查TF，不用最新车辆位置替代历史检测。

无线传输的是低频场景快照：全部分片到齐、CRC正确才原子更新该图层，半幅地图不先行覆盖。未完成重组最多12组、120秒超时，消息与片数也有硬上限。实时观测在本机保留最多约10秒，便于低速链路观看，**这与RViz原生1秒marker寿命不同**。地图/规划/轨迹保留到下一快照或清除；保留不代表数据新鲜。

本机用接收墙钟给SDK显示消息标时，并每秒重发缓存，让较晚打开的Foxglove也能看见；原ROS时间戳保存在metadata与 `/radio/layers` 中。`receive_age_s`、`source_age_lower_bound_s`、`stale` 独立暴露。来源年龄为下限，不包含完整无线排队/传输延迟；两台机器时钟未同步时，不把它当作准确端到端延迟。

输入中断后，Linux上报的源年龄增长，动态层超时清除。单个长快照仍会保留其开始时的采样时间；大图层在无线途中已经变旧是带宽约束，不能把本机每秒重发当成每秒无线更新。对端会话改变时清空本机缓存；单纯COM拔出时原图形按上述时效规则显示，`/radio/link.connected` 和 peer_age 明确反映断线。

## 协议和实现索引

```text
fsd_radio/configuration.py   复用原生显示配置解析
fsd_radio/ros_sender.py      Linux ROS订阅、采样TF、扩展ROS出口
fsd_radio/geometry.py        marker缓存、量化、抽样和限量
fsd_radio/protocol.py        Base64帧、CRC32、压缩、分包重组
fsd_radio/link.py            主轮询/从应答、ACK重试、限速和有界队列
fsd_radio/transport.py       pyserial独占读写路径、短写处理
fsd_radio/worker.py          串口重连与会话恢复
fsd_radio/foxglove_view.py   官方SDK SceneUpdate与Publish面板接入
fsd_radio/pc_receiver.py     PC接收与回环扩展接口
fsd_radio/cli.py             linux / pc / inspect / send / demo
```

无线帧 `~ + Base64(binary) + LF`；原始特殊模块命令前缀不出现在Base64业务正文中。二进制头为大端 `>BBIHIHHBB`：版本、角色、会话u32、序号u16、被确认会话u32、被确认序号u16、消息id u16、片序u8、总片数u8；随后最多96字节数据与大端CRC32。CRC覆盖头和数据。CRC32使用标准ISO-HDLC，`123456789`的结果为CBF43926。

完整消息为 `J + JSON`，或压缩更短时 `Z + zlib(JSON)`。ACK同时用于轮询/应答，没有数据时 count=0。每向一个未确认分片，最多补发2次；序号与会话去重，图层采用最新值快照，扩展采用FIFO。链路仅支持一对主从无线模块，不是多节点MAC协议。它与前一份开发文档中建议的短消息帧不同，属于针对可视化数据实现的独立协议，两端必须使用本目录同一版本。

## 验证与实机接入

Windows离线回归：

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[pc,test]"
.\.venv\Scripts\python.exe -m pytest -q
```

包含逐字节/粘包、坏CRC、解压上限、消息重组、重复包、丢ACK双向收发、联合限速、TF与量化、marker删除和过期、真实pyserial TCP串口、官方SDK实际WebSocket订阅与客户端发布。TCP只是测试替身，不是实际无线配置；功能加速测试使用更快虚拟时钟预算，限速用例单独按160字节/秒测试。

`tests/test_ros_smoke.py`在有ROS 2的Linux环境额外启动隔离domain 191、本机TCP串口替身及合成Marker/pose/velocity，验证真实ROS订阅和扩展Ping/Pong；没有rclpy时明确跳过。该测试不打开实物串口、不启动FSD算法、不发布驾驶话题。工控机加载ROS后可安装测试依赖并运行该单项：`python -m pytest -q tests/test_ros_smoke.py`。

实机按顺序验收：

1. 核对双端模块通信条件、COM3与Linux无线串口身份，关闭占用相同端口的串口调试工具。
2. Windows先启动接收端，Linux明确指定无线串口启动；检查 `/radio/link` 的 peer_age 与收发字节。
3. 从本机执行 `send`，确认Linux收到相同id的ping、本机收到pong，再开场景图层。
4. 先只看车辆和路径，记录数据年龄、重试和失败；再逐个加入地图、融合和引导簇。
5. 拔插任一无线模块、断开对端、停止某输入，验证连接和stale状态、动态层清除、扩展不会自动回放。
6. 连续记录实际首发成功率、重试后完整快照率、更新周期与双向Ping往返时间，再调整分包和速率预算。

当前交付在Windows完成离线和真实本机WebSocket/虚拟串口测试；**没有ROS 2运行环境、没有连接COM3或无线模块，Linux ROS订阅/真实TF/空口时序及现有Foxglove界面尚需实机验收**。不因此宣称2.4kbps可无损实时传输全部RViz数据。

参考：[Foxglove SDK WebSocket](https://docs.foxglove.dev/docs/sdk/websocket-server)、[Foxglove消息类型](https://docs.foxglove.dev/docs/sdk/schemas)。模块行为依据上级工作目录所附E22 USB手册；本程序只使用已配置好的数据透传通道。
