# FSD 独立可视化代码包

ROS 2 Humble + RViz 的独立工作区，适用于已运行的 WUTA-FSD 算法、实车传感器或 ROS bag。启动时只打开 RViz 和显示辅助节点，不启动算法、传感器驱动或仿真器，不发布驾驶、发车、急停、任务或 TF 消息。

主包名：`fsd_visualization`，另含 `fsd_rviz_plugins` 图像显示插件。版本：0.1.0。依赖 ROS 2、RViz、PyYAML，以及 WUTA 的消息定义（兼容桥接模式和接口测试需要 `wuta_msgs` / `autoware_msgs`）。源码不包含模型、FSD 算法源码或机器专用的安装环境。

2026年10月17日新增可选[ E22无线遥测与Foxglove组件](radio_link/README.md)：Linux按既有显示话题量化抽样，Windows从COM3接收并经Foxglove官方SDK显示；双向JSON扩展独立于驾驶话题。无线组件独立安装/启动，原有build.sh/start.sh行为不变，Linux无线串口必须显式指定。

## 1. 解压、构建与打开 RViz

将交付的压缩包解压到任意目录，进入 `fsd-visualization-code`。需要已有 ROS 2 Humble；FSD 自定义消息由你现有的 FSD 安装空间提供。

```bash
cd /path/to/fsd-visualization-code
export FSD_SETUP=/path/to/fsd_ws/install/setup.bash
export ROS_DOMAIN_ID=42  # 改为正在运行的算法所用 domain；脚本不固定为 42
bash build.sh
bash start.sh          # 默认自动打开 RViz
```

这里的 `/path/to/...` 都需要换成实际路径。若终端已 source 所需算法环境，可以不设置 `FSD_SETUP`。该环境变量只加载已有安装空间，不发现源码、不编译 FSD、不启动任何算法。`build.sh` 只构建本代码包内的两个包。也可将 `src/` 下两个包一起放进你自己的 ROS 工作区后构建。

FSD 算法需通过原有入口单独运行，并提供有效 TF。初次连接没有图层时，先检查 topic、消息类型和 TF；可视化不会自动发车，也不会创建测试锥桶。

### 模式 A：直接查看算法原生显示输出

```bash
bash start.sh mode:=native use_sim_time:=false  # 自动打开 RViz，实车/墙钟
```

适合当前算法已经发布 `*_viz`、相机标注图像的情况。默认 `native`，不会额外订阅锥桶/航点并重绘。显示辅助节点只订阅速度、发布速度文字及状态。

### 模式 B：算法没有 `_viz` 时使用桥接显示

```bash
bash start.sh mode:=bridge use_sim_time:=false  # 自动打开 RViz，转换算法输出
```

桥接读取 `ConeArray`、`Lane`、`PoseStamped`，产生独立 `/fsd_visualization/*` 图层。不会把桥接输出送回算法；没有来源信息时不生成 camera_guided 等来源标签，也不生成引导簇或控制预瞄点。

此模式仍直接显示已有的相机标注图像、原始雷达、引导簇和控制目标；对应算法未发布时可在配置中关闭。桥接轨迹来自定位估计，采用距离抽样、有界缓存和跳变重置，未经原算法轨迹平滑，和原生轨迹可能存在差异。

### ROS bag / 使用仿真时钟的数据源

先启动显示，再在另一个已加载相同消息环境、使用相同 domain 的终端回放：

```bash
bash start.sh mode:=bridge use_sim_time:=true  # 自动打开 RViz，等待 /clock
```

另一个终端：

```bash
ros2 bag play /path/to/bag --clock
```

Bag 应包含所需输入和 `/tf`、`/tf_static`；回放暂停时 ROS 时间也会暂停。速度过期检查使用墙钟定时器，仍会显示 STALE。定位时间戳或 ROS 时钟回退时清理轨迹，避免连接两轮回放。若 bag 含车辆控制话题，应在独立 domain 回放，确保没有实车执行节点接收这些记录。

### 速度源改用 Odometry

```bash
bash start.sh mode:=native velocity_type:=odometry use_sim_time:=false  # 自动打开 RViz
```

默认速度源是 `/localization/velocity` 的 `geometry_msgs/msg/TwistStamped`；odometry 模式改为配置的 `nav_msgs/msg/Odometry`。显示 `hypot(vx, vy)`，单位 m/s 和 km/h，是输入速度的平面大小，不保留倒车符号，不等于独立真值或指令速度。

### 停止

在启动终端 Ctrl+C，或关闭该次 RViz 窗口，launch 会同时停止显示辅助节点。FSD 算法与驱动由它们原来的入口管理，继续运行。默认使用机器现有的图形设备；需要本机 NVIDIA PRIME 时可选：

```bash
FSD_VIS_NVIDIA=1 bash start.sh mode:=native  # 自动打开 RViz，请求 NVIDIA
```

## 2. 修改 config（先配置，再看 Displays）

### 2.1 话题、坐标系、超时和轨迹参数

编辑 [topics.yaml](src/fsd_visualization/config/topics.yaml)：

```bash
cp -n src/fsd_visualization/config/topics.yaml src/fsd_visualization/config/topics.yaml.bak
nano src/fsd_visualization/config/topics.yaml
bash start.sh mode:=bridge  # 关闭旧显示后执行，打开 RViz 并加载修改
```

| 字段 | 含义 |
|---|---|
| `fixed_frame` | 三维数据统一显示的固定坐标系，默认 `map` |
| `target_frame` | RViz 观察参考系和速度文字附着的坐标系，默认 `base_link`；速度文字会随此真实 TF 移动 |
| `frame_rate` | RViz 重绘目标，默认 30，不是推理频率 |
| `stale_seconds` | 速度输入超过此墙钟间隔后显示 STALE，默认 1 秒 |
| `trajectory_max_points` | 桥接轨迹最多保留点数，默认 5000 |
| `trajectory_min_distance` | 轨迹采样最小三维间距，默认 0.05 m |
| `trajectory_max_step` | 相邻位置超过此三维距离则重置轨迹，默认 5 m；应按定位频率/速度调整 |
| `topics.*` | 输入话题名；要求绝对话题名，空字符串禁用相应可选输入/原生图层 |

如果你的系统使用 `odom` 而没有 `map`，将 fixed_frame 改为 `odom`，并检查所有所需数据能通过 TF 转换到它。没有 `base_link` 时也需修改 target_frame，例如先改为 `odom` 做固定视角检查；本包不会伪造 map→odom 或车体 TF。

原生图层使用 `*_markers` 配置；桥接图层读取 `fused_cones`、`cone_map`、`centerline`、`final_waypoints`、`pose` 等输入。把原生话题设为空并不等于禁用桥接转换；要禁用桥接输入，需把对应算法输入键设为空。

### 2.2 显示样式与个人配置

编辑 [fsd.rviz](src/fsd_visualization/config/fsd.rviz)，或在 RViz 中调整后使用 **File → Save Config As** 保存个人配置。

启动时从模板和 topics.yaml 生成临时 `.rviz` 文件；**直接 Save Config 只写入临时文件，下次启动不保留，退出时该临时目录会被清理。** 需另存到永久路径，再用：

```bash
bash start.sh mode:=native rviz_config:=/absolute/path/my-view.rviz  # 自动打开 RViz
```

生成器仍会用 topics.yaml 覆盖已知图层的 Topic、Fixed Frame、Target Frame 和 Frame Rate。持久改这些字段请改 topics.yaml；其他图层开关、点大小、透明度和窗口布局保留模板值。自定义模板需保留原显示项名称，才能匹配自动话题映射。另存时若改名，需手工管理改名项的 topic。

将项目话题配置另存为个人文件时：

```bash
bash start.sh mode:=bridge topics_config:=/absolute/path/my-topics.yaml  # 自动打开 RViz
```

### 2.3 算法端要启用的调试输出

- 相机节点需要发布标注图像；当前 Python YOLO 支持启动参数 `publish_annotated_image: true`、`annotated_topic`。
- 当前 C++ 融合节点的引导簇/来源输出需要启动时 `publish_fusion_visualization: true`。在算法自己的 launch/参数 YAML 中启用并重启该算法节点；本包不修改其参数，也不启动它。
- 旧版 FSD 若没有这些输出，使用 bridge 模式。基础 ConeArray 不包含来源点集，因此不能由本包还原真实引导聚类；控制预瞄也必须来自控制器真实输出。
- 修改原算法参数仍按你的工作区构建/重启规则处理，与 `.rviz` 显示设置分开。

## 3. RViz 各项信息详解

| 图层 | 默认原生话题 | 意义 |
|---|---|---|
| Grid | 无 | 每格 1 m 的空间参照，与道路/地图内容无关 |
| Camera detections | `/camera/yolo/image_annotated` | 同帧相机检测框、类别、置信度；画面已有文字完全由相机节点提供 |
| Camera-guided clusters (fusion) | `/perception/fused/guided_clusters_viz` | 算法实际引导簇点集与包围框；仅原生输出，本包不重算 |
| Raw LiDAR (optional) | `/hesai/pandar` | 整帧原始点云，默认关闭，按 Z 着色 |
| Fused targets and sources | `/perception/fused/cones_viz` | 本帧融合目标与来源。原生图层颜色由算法决定，当前 FSD 通常绿色匹配、橙色引导、青色未匹配 |
| Cone map | `/mapping/cone_map_viz` | 跨帧累计、关联确认的地图锥桶 |
| Centerline | `/planning/centerline_viz` | 边界检测输出的中心参考路线 |
| Final planned path | `/planning/final_waypoints_viz` | 路径生成输出，控制器准备跟踪的航点几何路线 |
| Driven trajectory | `/planning/driven_trajectory_viz` | 定位系统估计的历史行驶路线，不是外部真值 |
| Control target | `/control/target_viz` | 控制器真实预瞄目标/范围，仅显示它实际发布的内容 |
| FSD localization speed | `/fsd_visualization/speed` | FSD 定位速度，输入过期或非有限值后显示 STALE |

Bridge 模式将融合目标、地图、中心线、最终路径和行驶轨迹改为 `/fsd_visualization/*` 输出。此时融合目标名字明确标注“semantic colors; sources unavailable”：红/黄/蓝/橙代表锥桶类别，白色代表未知，不表示融合来源。桥接中心线为绿色、最终路径为青色、轨迹为金黄色，方便区分。

### Displays 属性速查

| 属性 | 含义 |
|---|---|
| Enabled / 复选框 | 是否显示该项，不影响车辆控制 |
| Topic | 输入话题；需匹配插件的消息类型 |
| Status | 显示/消息/TF 状态，OK 不证明算法正确或数据新鲜 |
| Namespaces | Marker 中各组图形的显示开关 |
| Fixed Frame | 统一三维坐标基准；默认 map |
| Views / Target Frame | 观察参考系；默认 base_link |
| PointCloud2 / Decay Time=0 | 只显示最新点云，增加时会叠加历史扫描并增加开销 |
| Points / Size (Pixels) | 原始点云的屏幕点大小；其他样式使用 Size (m) |
| AxisColor / Axis=Z | 高度伪彩，不是相机语义颜色 |
| Intensity / Channel Name | 改为反射强度着色时需要输入包含 intensity 字段 |
| Alpha | 透明度，0 透明、1 不透明 |
| Image / Normalize Range | 数值图像的范围显示，不是 YOLO 阈值；普通彩色图中通常隐藏 |
| Best Effort / Keep Last / Depth 1 | 优先最新传感器数据，允许丢帧，不积累大队列 |
| Volatile | 等待新消息，不自动补发订阅前的历史数据 |

原生 MarkerArray 颜色、尺寸、文字由算法发布者决定。模型置信度不等于三维位置精度。图层没消息、引导簇为空、停车时无轨迹或控制点，都需要结合真实算法条件判断。

## 4. 输入契约与保护显示语义

| bridge 输入 | 消息类型 |
|---|---|
| fused_cones / cone_map | `wuta_msgs/msg/ConeArray`，含 header、cones、position、color |
| centerline / final_waypoints | `autoware_msgs/msg/Lane`，含 header 和 waypoints 的 pose |
| pose | `geometry_msgs/msg/PoseStamped` |
| velocity | `geometry_msgs/msg/TwistStamped` |
| odometry | `nav_msgs/msg/Odometry` |

源 frame_id 和采样 stamp 保留；空 frame 的几何输入丢弃，不擅自改成 map。航点有不同 frame 时清空该条路径，避免将不同坐标系直接连线。非有限坐标跳过；空锥桶/路径消息使用 DELETEALL 主动清除旧图形。实时融合圆柱设置 ROS 时间 1 秒寿命，地图/规划/轨迹保留到更新或清理；这些静态图层保留不能用于证明输入仍新鲜。

轨迹按距离采样，缓存有上限；位置跃迁、frame 变化、时间回退会开始新段，避免错误长线。速度过期使用墙钟，ROS 暂停时也会更新状态。

`/fsd_visualization/status` 是 JSON 字符串，包括速度源、速度是否过期、各被订阅输入的墙钟帧龄（尚未收到为 null）、轨迹点数和模式。它只检查辅助节点实际订阅的输入：native 模式不会替 RViz 监测所有原生图层；此时需检查图层话题自身。辅助节点自身异常退出或 RViz 退出，launch 会停止整个显示会话。

## 5. 诊断与测试

在已经加载 ROS 和算法消息环境的终端：

```bash
ros2 topic list -t
ros2 topic info /localization/velocity -v
ros2 topic info /perception/fused/cones_viz -v
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /fsd_visualization/status --once --qos-reliability best_effort
```

有话题但没有数据：核对 ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY、发布端与显示端 QoS、use_sim_time、/clock 和 TF。Best Effort 发布者与 Reliable 订阅者不兼容；保持显示订阅 Best Effort 可接收 Reliable 或 Best Effort 发布者。

运行接口回归：

```bash
bash check.sh
```

测试默认独立 domain 179、仅本机通信；不要将 FSD_VIS_TEST_DOMAIN 设成真实算法/车辆 domain。测试会发布合成锥桶、路径、定位和速度输入，但不发布控制指令。它不启动 RViz GUI；检查真实消息接口、5 m/s 速度计算、速度超时/NaN、空帧清理、坐标帧冲突、轨迹回退/跳变、安装后的 launch 启停，以及辅助发布者没有控制 topic。

有可用图形桌面时，可进一步进行窗口检查（每条命令打开并关闭一个 RViz 窗口）：

```bash
bash check-gui.sh --mode native
bash check-gui.sh --mode bridge
```

图形检查默认独立 domain 180，会生成合成图像、Marker、定位和测试用静态 TF，检查最终 Best Effort 订阅、进程持续运行和干净退出。测试 TF 仅由测试工具发布，正常 `start.sh` 不发布 TF。该检查不评估相机检测精度、屏幕实际绘制内容或真实驾驶性能。加载图层时可能短暂出现默认 Reliable 订阅的 QoS 警告；检查要求最终订阅都为 Best Effort。

只有自动化检查需要无图形启动时使用 `rviz:=false`；所有上面的用户显示启动默认打开 RViz。默认图像使用 `fsd_rviz_plugins/SafeImage`，保留标准 Image 渲染，修正 Humble 中重复释放订阅连接的问题。长期运行和其他机器上的显卡表现仍需现场验证。

## 6. 目录与移植

```text
fsd-visualization-code/
├── build.sh / start.sh / check.sh / check-gui.sh
├── scripts/environment.sh       # 只加载 ROS 与已有 FSD 环境
├── src/fsd_visualization/
│   ├── package.xml / setup.py / setup.cfg
│   ├── launch/view.launch.py    # 默认 RViz + 辅助节点
│   ├── config/topics.yaml       # topic/frame/超时配置
│   ├── config/fsd.rviz          # 11 个显示项模板
│   └── fsd_visualization/       # 转换节点与配置生成
├── src/fsd_rviz_plugins/        # 独立 SafeImage 显示插件
├── tests/                      # 隔离域 ROS 接口测试
├── VALIDATION.md                # 本次验证范围
└── LICENSE
```

压缩包只包含源码、配置、脚本、文档和测试；不含 build/install/log，不依赖交付机器的绝对路径。可以复制整个目录在另一台有 ROS/FSD 消息的机器构建，或将两个 ROS 包放入现有工作区，运行 `ros2 launch fsd_visualization view.launch.py`（默认打开 RViz）。

接口以当前 WUTA 消息结构为目标，不保证自动兼容任意第三方 FSD 的消息定义。第三方栈仅使用标准 MarkerArray/Image/PointCloud2 时可用 native 模式并映射话题；如果基础算法消息结构不同，需要改桥接适配代码。

RViz 和 launch 用法参考 [ROS 2 RViz Humble 源码与指南](https://github.com/ros2/rviz/tree/humble/rviz2/doc)、[launch_ros Node 参数接口](https://github.com/ros2/launch_ros/blob/humble/launch_ros/actions/node.py)。
