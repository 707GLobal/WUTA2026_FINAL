# Foxglove 简明操作指南

## 1. 启动与连接

Linux端保持CLI和无线发送程序运行。Windows接收程序未运行时，在主仓库目录执行：

```powershell
.\fsd-visualization-code\radio_link\start_windows.ps1
```

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

“运行中”表示全栈启动检查已完成。“停止”停止FSD，保留CLI与无线程序，之后可以再次启动。

## 4. 修改参数

在“参数控制”页修改 `selection/module` 选择模块，编辑 `yaml/` 下的参数，再点击“保存配置”。以 `meta/status` 的Linux保存结果为准；仅编辑数值仍是未保存草稿。

保存后需重启相关进程才生效。“丢弃草稿并重新读取”会丢弃未保存修改。

## 5. 断线与退出

CLI离线时检查Linux端CLI、无线程序是否仍在运行；Foxglove连接或参数源未恢复时，断开再连接上述地址。

退出Windows接收程序，在其终端按 `Ctrl+C`。修改无线模块参数前先退出接收程序、释放COM3，改完再启动。RSSI显示的是原始值，不能直接当作dBm。
