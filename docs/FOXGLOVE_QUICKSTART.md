# Foxglove 简明操作指南

## 1. 启动与连接

Windows也可直接双击 `D:\ebyte\output\FSD_E22_Receiver.exe`，无需Python，仍需USB串口驱动。先用Ctrl+C退出旧接收程序，再启动exe；Foxglove连接和布局操作相同。exe命令行指定串口用 `--port COM3`，输入模式可用 `--mode bridge`。脚本方式保留如下。

Linux端运行 `bash start_fsd_manager.sh --menu`，CLI会在独立终端联启无线（默认bridge），已有发送进程不重复启动。CLI和无线可独立退出，`--no-radio`关闭联启；无线仍可单独执行 `bash fsd-visualization-code/radio_link/start_linux.sh --mode bridge`。没有桌面时无线在后台运行，日志为 `.runtime/fsd_radio/linux.log`。Windows接收程序未运行时，在主仓库目录执行：

```powershell
.\fsd-visualization-code\radio_link\start_windows.ps1
```

启动脚本自动寻找本机可用的Python，优先项目虚拟环境。首次使用或提示缺少依赖时，先执行 `.\fsd-visualization-code\radio_link\setup_windows.ps1`；需要64位Python 3.10以上。可加 `-Python 'D:\Python\python.exe'` 手动指定解释器。

默认自动识别USB适配器，无需填写COM号；同时插入多个同型号适配器时，按提示选择，或用 `-Port COM3` 手动指定。

Foxglove选择“打开连接 → Foxglove WebSocket”，连接：

```text
ws://127.0.0.1:8765
```

首次使用或布局更新后，在布局菜单导入[foxglove_layout.json](../fsd-visualization-code/radio_link/foxglove_layout.json)。

## 2. 选择页面

| 页面 | 用途 |
| --- | --- |
| 车辆与场景 | 查看车辆、路径、锥桶及车速 |
| CLI 日志与控制 | 查看日志和CLI状态，检查、启动、停止；车辆表下方显示RSSI原始值 |
| 传感器概览 | 查看传感器在线状态、GNSS质量、频率和参数 |
| 参数控制 | 读取、修改和保存Linux YAML参数 |

## 3. 检查、启动、停止

在“CLI 日志与控制”页等待CLI在线，按需点击“检查”“启动”或“停止”。显示“操作中”时等待结果，不重复点击；失败原因看日志和操作结果。

任务结束后，CLI状态指示器显示退出码及含义；0为正常结束，1/2看失败日志，130或-2可能是主动停止，其他码看操作详情。

“运行中”表示全栈启动检查已完成。“停止”停止FSD，保留CLI与无线程序，之后可以再次启动。

## 4. 修改参数

在“参数控制”页修改 `selection/module` 选择模块，编辑 `yaml/` 下的参数，再点击“保存配置”。以 `meta/status` 的Linux保存结果为准；仅编辑数值仍是未保存草稿。

顶部指示器显示读取中、未保存、保存中、已保存或失败；错误原因见页下部参数操作详情。保存后需重启相关进程才生效。“丢弃草稿并重新读取”会丢弃未保存修改。

## 5. 断线与退出

CLI离线时检查Linux端CLI、无线程序是否仍在运行；Foxglove连接或参数源未恢复时，断开再连接上述地址。

退出Windows接收程序，在其终端按 `Ctrl+C`。修改无线模块参数前先退出接收程序、释放COM3，改完再启动。RSSI显示的是原始值，不能直接当作dBm。

模式切换：在“CLI日志与控制”的检查/启动按钮行点“切换模式”，等同CLI的M键，顺序为正常→制动台架→有人跑动记录→正常。先停止并等待清理完成；切换后重新检查，再按需启动。当前模式直接显示在原“CLI状态”背景指示器内，须等Linux确认。这里是运行配置，不是VCU赛项或GO。新版CLI、Linux无线、Windows接收端及布局需配套更新。
