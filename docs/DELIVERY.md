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
