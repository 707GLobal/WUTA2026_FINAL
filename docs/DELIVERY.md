# 当前源码与交付入口

2026-10-20按无线0.7.4核对。当前参数以radio.yaml、实际模块设置和CLI选定文件为准；本页不把历史版本说明当作现场生效证据。

## 当前功能和布局

串口115200 8N1、模块空速62500 bit/s、软件双向合计3000 UART B/s；指令/相应ACK/结果走独立优先级，批量参数读取走普通数据队列。当次CLI主日志全文转发，不补历史或组件独立文件。

| 选项卡 | 内容 |
| --- | --- |
| 车辆与场景 | 3D几何、车速与曲线 |
| CLI 日志与控制 | 完整日志、CLI状态/操作中、检查/启动/停止/查询、车辆任务；vehicle.rows表格下方RSSI原始值 |
| 传感器概览 | 五类设备状态、新鲜度、GNSS质量、快照年龄、dashboard.rows表格和原始传感器详情 |
| 原始诊断 | 日志传输和车辆任务原始数据，不重复放专用传感器组件 |
| 参数控制 | Parameters读取/草稿、保存、重新读取；保存不热更新或自动重启 |

RSSI本机显示原始0..255值，未确认包尾dBm换算；Windows软件解析开启，Linux默认关闭，分别匹配硬件附加RSSI开关。真实模块分包240字节且一应用帧一无线包为前提。详细接口与限制见[无线README](../fsd-visualization-code/radio_link/README.md)、[CLI/参数说明](FOXGLOVE_PARAMETERS.md)。

## 源码与维护

| 路径 | 用途 |
| --- | --- |
| fsd-visualization-code/radio_link/radio.yaml | 两端各自读取的串口、RSSI解析开关、预算和目标间隔 |
| fsd-visualization-code/radio_link/foxglove_layout.json | 唯一当前布局源 |
| fsd-visualization-code/radio_link/fsd_radio/ | 无线、CLI接口适配和PC显示 |
| fsd_startup/ | CLI管理、配置保存、日志生命周期 |
| tools/check_fsd_module_configs.py | CLI共享配置目录与原一致性审计 |

CLI及显示各是独立Git子模块，修改在子模块提交推送后，主仓库提交对应指针。默认Windows COM3；Linux无线设备不能复用惯导串口。

## 本机交付文件

工作目录D:\ebyte，主仓库WUTA2026_FINAL。output保留四个交付文件：

- E22_Windows_Android_开发文档.md/pdf：原模块首版开发设计资料，包含旧方案与未来Android方向，不是当前FSD无线部署配置。
- FSD_E22_CLI与传感器.json：当前布局导入副本；文件名保留兼容，内部选项卡按上表拆分。
- FSD_E22_Foxglove_源码.zip：显示子模块源码包，不包含CLI子模块、宿主tools/docs、虚拟环境、缓存或现场日志；完整部署应拉取主仓库及全部子模块。

源码ZIP与fsd-visualization-code/SHA256SUMS从当前Git源码生成。tmp为本机辅助脚本和接收日志；archive/20261020_obsolete保留可恢复归档的旧布局、截图与45个临时文件。归档不参与当前交付，不删除用户日志和断点。

## 加载与验证范围

完整升级：先等待在途管理操作完成，主仓库git pull --ff-only、git submodule update --init --recursive；Linux配套加载CLI与无线程序，Windows加载接收端并导入新版JSON。仅修改布局重新导入即可；PC显示/RSSI解析代码更新需加载Windows新版，Linux不启用自身RSSI时无需因本地RSSI指示器重启。运行代码不会随git pull自动更新。

本机0.7.4接收端已加载，并只读观察过RSSI RAW 197、326个接收样本、framing_errors=0；只是当时观察，不是全场景验收。本次文档审计没有启动/停止设备或执行测试、WSL、构建、CLI操作。旧版本验证证据见显示子模块VALIDATION.md，不能替代当前实机验收。
