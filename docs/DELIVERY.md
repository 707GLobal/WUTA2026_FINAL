# 工作目录与当前交付

## 源码入口

| 路径（主仓库内） | 用途 |
| --- | --- |
| `fsd-visualization-code/radio_link/radio.yaml` | 串口、无线总预算与刷新目标 |
| `fsd-visualization-code/radio_link/foxglove_layout.json` | 唯一当前Foxglove布局 |
| `fsd-visualization-code/radio_link/fsd_radio/` | Linux发送、Windows接收与本机显示 |
| `fsd-visualization-code/radio_link/README.md` | 无线部署与接口细节 |
| `fsd_startup/` | CLI、日志生产与当前运行选择 |

Foxglove“传感器概览”使用原生Indicator/Gauge/Table；五类设备状态、数据新鲜度与GNSS质量使用背景色，快照接收年龄使用0～20秒条形仪表。原始字段保留在“原始诊断”页。无需扩展包。

## 本机目录

Windows工作目录为 `D:\ebyte`，Git主仓库为其下 `WUTA2026_FINAL`。

| 目录 | 保留内容 |
| --- | --- |
| `WUTA2026_FINAL/` | 主项目源码与Git子模块 |
| `output/` | 当前开发说明、当前布局及源码交付包 |
| `tmp/` | 接收端重定向日志、诊断记录和仍可能使用的本机辅助环境 |
| `archive/20261020_obsolete/` | 从tmp/output移出的45个过期截图、旧布局、临时补丁与脚本，以及空扩展草稿目录 |
| 根目录说明书和串口/模块配置工具 | 原始参考及现场工具 |

旧文件采用可恢复归档，未永久删除。清理未触及串口工具、PDF说明书、正在使用的无线`.venv`、当前接收日志或用户运行journal/断点。归档不参与源码打包，也不提交Git。

## 当前交付文件

`D:\ebyte\output`只保留以下四项：

- `E22_Windows_Android_开发文档.md`：原模块开发说明。
- `E22_Windows_Android_开发文档.pdf`：上述说明的PDF交付。
- `FSD_E22_CLI与传感器.json`：从唯一布局源复制的当前导入文件。
- `FSD_E22_Foxglove_源码.zip`：显示子模块的源码交付包。

后两项从当前显示源码生成；压缩包不包含虚拟环境、缓存、构建/安装产物或现场日志。`fsd-visualization-code/SHA256SUMS`随源码变更更新。

导入布局：在Foxglove布局菜单选择导入JSON，选择当前布局文件，再打开“传感器概览”。此次样式调整无需重启接收程序或Linux。

## 0.7.0配置与CLI控制更新

当前布局新增“CLI 控制与参数”页，原生Parameters及Publish按钮接入CLI配置读取/保存、检查、启动、停止。详细用法见 [FOXGLOVE_PARAMETERS](FOXGLOVE_PARAMETERS.md)。本机output布局与源码ZIP随本版源码更新；CLI实现与宿主tools/docs通过各自Git仓库交付，不包含在显示子模块ZIP内，部署应拉取主仓库并更新全部子模块。

此次需更新并重启CLI、Linux无线和Windows接收端，然后重新导入布局。先前“样式调整无需重启”只指0.6.3样式变更，不能用于本版。按用户要求未运行测试、构建、WSL或串口验证，也未重启当前程序。

0.7.1：源码交付包和SHA256SUMS更新为指令/数据双队列版本；布局无变化。两端无线与CLI需配套更新，独立指令队列状态见/radio/link.command_queue。回归用例已写入源码，按要求未执行。

0.7.2：当前布局/源码ZIP增加CLI控制页顶部背景状态Indicator；更新Windows接收端后重新导入布局，Linux无需为本次面板变更重启。

当前布局已分为CLI操作/日志/传感器与独立参数控制：output导入JSON、源码ZIP和SHA256SUMS同步更新，仅需重新导入布局，无需重启两端或改无线配置。

最新交付布局：CLI页去除传感器表格、更名“CLI 日志与控制”；专用传感器信息统一放入“传感器概览”，参数控制独立。当前JSON、ZIP和校验清单同步。

0.7.3布局、源码ZIP、SHA256SUMS同步更新，CLI状态增加橙色操作中；Windows接收端需加载新版，重新导入布局，Linux无需为此显示变更重启。

0.7.4当前布局/源码ZIP/校验清单同步：增加包尾RSSI可选解析与CLI页车辆任务表下方的原始值指示器，Windows解析配置已开启，Linux默认关闭；240字节模块包为前提。发现旧接收端已被重新启动后，确认无在途操作并重启加载新版；需导入新版布局，未运行测试或实机验收。
