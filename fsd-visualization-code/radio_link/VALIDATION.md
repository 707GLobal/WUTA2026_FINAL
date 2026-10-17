# 无线遥测验证记录

日期：2026年10月17日。范围仅为新增 radio_link，未运行原有RViz/算法全栈测试。

已执行：

- Windows独立Python虚拟环境，可编辑安装项目及 `foxglove-sdk==0.29.2`、pyserial 3.5、PyYAML。
- `python -m pytest -q`：29 passed，1 skipped，约4.7秒。
- 跳过项：`test_ros_smoke.py`，本机无rclpy；该文件提供工控机可执行的隔离ROS domain 191和虚拟串口验收。
- 真实pyserial API经TCP串口替身进行双向场景/JSON分包传输，故意使用7字节TCP片段验证流式解析。
- 官方SDK本机WebSocket服务：实际连接握手、广告、订阅速度JSON、客户端Publish上行通过；全部10种受支持Marker转换完成SDK序列化。
- 丢ACK、最终一次重试成功、重复包、会话变化与序号回绕、有界重组与解压、双向160字节/秒节拍测试通过。
- `inspect`正确解析给定的两个显示配置，识别被排除的图像/原始点云、native/bridge映射和实车速度覆盖；个人topics YAML外置回归通过。
- `demo --duration 1.5`启动并停止本地SDK服务，未打开COM3。
- Python源码编译、Ruff F级静态检查、PowerShell脚本语法解析、Git Bash `bash -n`、`git diff --check`通过。

未执行：Linux ROS 2运行、真实TF与算法话题接入、COM3收发、双模块空口试验、实际已安装Foxglove的界面查看和长期稳定性测试。未启动FSD、惯导、CAN或车辆控制，也未改写无线模块参数。

后续应先按README完成带id的双向Ping/Pong，再逐层增加显示数据，并记录实际数据年龄、重试与往返延迟。当前结果不能证明无线实际吞吐、全部Linux驱动兼容性或全部图层的实时刷新率。
