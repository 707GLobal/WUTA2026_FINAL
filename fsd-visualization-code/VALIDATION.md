# 验证记录

日期：2026-10-10（Asia/Shanghai）。目标：独立 FSD 可视化代码包 0.1.0。

## 已验证

- ROS 2 Humble 本机实际 colcon 构建成功：`fsd_rviz_plugins` 与 `fsd_visualization`，两个包均完成。
- `bash check.sh`：**4 passed in 3.25s**。使用 DDS domain 179 和合成消息，检查配置生成、真实 ROS 消息转换、源 frame/stamp 保留、5 m/s 平面速度、Odometry 模式、非有限速度、时钟暂停时的速度过期、空帧 DELETEALL、航点 frame 冲突、定位时间回退与位置跳变清理。
- 通过安装后的 `start.sh mode:=bridge rviz:=false` 启动辅助节点，接收到状态输出，退出干净。
- 检查辅助节点发布的话题仅为 `/fsd_visualization/*` 和 ROS 系统话题，无控制/发车/急停输出。
- 图形检查 `bash check-gui.sh --mode native`：PASS。实际打开 RViz，提供合成 RGB 图像、7 个原生 MarkerArray 和定位速度，运行 10 秒，最终 RViz 订阅的相关话题均为 Best Effort，正常退出。
- 图形检查 `bash check-gui.sh --mode bridge`：PASS。实际打开 RViz，提供合成图像、基础锥桶、航点、定位和速度，桥接图层订阅正常，运行 10 秒后正常退出。
- GUI 检查使用独立 DDS domain 180、仅本机通信；仅测试工具发布合成静态 TF，正常辅助节点不发布 TF。
- Python/Bash 语法、XML/YAML 解析及 README 本地链接检查通过；运行源码无 CARLA Python import、无 `/sim/` 输入话题。

## 发现并修正

最初使用标准 Humble Image 插件时，收到合成图像后退出 RViz 出现 mutex 断言、exit code -6。独立包迁入 SafeImage 插件，先断开并清空订阅回调连接，再释放订阅；修正后上述两种模式均干净退出。没有修改系统 ROS 库或算法工作区。

RViz 加载配置时先建立默认 Reliable 订阅，再切换为配置的 Best Effort，可能出现短暂 QoS 警告。GUI 检查通过 ROS graph 检查最终订阅策略，而不是把启动瞬间的警告当作最终状态。

辅助节点关闭时也处理终端 SIGINT 与 launch 转发的重复信号，避免在资源清理期间再次抛 KeyboardInterrupt。

## 验证环境与范围

构建及接口验证使用本机已有 ROS Humble 和 WUTA 消息安装空间；没有启动 CARLA、FSD 算法、传感器驱动或驾驶控制。没有改变现有算法配置。

本次未对真实 FSD 输入做现场接入、屏幕像素检查、感知精度或驾驶验收。两次 GUI 检查仅证明短时进程运行、实际订阅策略及退出流程；不证明长时稳定、另一台机器的显卡性能或端到端实时性。速度显示是定位输出，不是独立真值。

## 复现

加载自己的 FSD 消息环境后，在源码包根目录执行：

```bash
bash build.sh
bash check.sh
bash check-gui.sh --mode native
bash check-gui.sh --mode bridge
```

最后两条需要图形桌面，并自动打开/关闭 RViz。自动测试 domain 不应与实车或算法 domain 重合。原始图形检查日志副本在 `validation/gui-native.log`、`validation/gui-bridge.log`；重新运行的日志位于 `log/`。
