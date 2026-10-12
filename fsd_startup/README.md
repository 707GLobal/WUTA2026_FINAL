# FSD 交互启动包

入口是根目录 `./start_fsd_manager.sh`，复用原 `start_full_fsd.sh` 的部署配置和有依赖的启动顺序。
默认使用 Rich + Textual 的纯键盘终端界面，操作、检查、车辆状态、传感器参数和日志均在一个终端内。
默认立即尝试启动；首次建议用菜单模式先查看检查结果。界面关闭鼠标操作，无可点击按钮。

```bash
./start_fsd_manager.sh --menu                    # 单终端菜单，暂不启动车辆
./start_fsd_manager.sh --check --no-rviz         # 并行只读检查，返回汇总退出码
./start_fsd_manager.sh --skip-build              # 启动实车全栈，默认含 RViz
./start_fsd_manager.sh --sensor-only --no-rviz   # 传感器链，无规划控制
./start_fsd_manager.sh --build-only              # 强制构建，不连接硬件
./start_fsd_manager.sh --status
./start_fsd_manager.sh --stop                    # 请求管理器清理并退出
```

原脚本的模型、串口、外参、控制器和 CAN 配置等参数继续透传。
管理包自己的选项为 `--menu`、`--plain`、`--no-window`、`--no-color`、`--check`、`--status`、`--stop`。
需要交互终端启动；非交互环境可执行只读 `--check`。没有图形桌面时使用 `--no-rviz`，终端内仍可查看状态。
`--no-window` 在新界面中禁用后台状态采集；`--plain` 保留旧逐行菜单及旧 Tk 窗口。
界面依赖独立安装在 `.runtime/fsd_manager/ui_deps`，不会添加到 ROS 子进程的 Python 搜索路径。
重新安装依赖执行 `bash fsd_startup/install_ui.sh`；版本在 `requirements-ui.txt` 固定。

## 开机自启

本机已安装 `~/.config/autostart/fsd-manager.desktop`。GNOME 桌面登录后等待10秒，
通过 `autostart.sh` 打开彩色管理终端，自动执行 `start_fsd_manager.sh --skip-build`，
状态与日志显示在同一终端，空间可视化仍由默认 RViz 展示。系统现有 GDM 配置已自动登录 wuta，因此开机即可触发。
自启复用已构建产物，失败保留菜单，运行中故障不自动重启控制链。
管理器已运行时不会打开第二个终端。q 退出后，本次桌面会话不会自动重新启动它。

禁用自启可将 `~/.config/autostart/fsd-manager.desktop` 中
`X-GNOME-Autostart-enabled=true` 改为 `false`。
更改自启参数可修改 `autostart.sh`；等待时间默认10秒，可用环境变量 `FSD_AUTOSTART_DELAY` 覆盖。

## 菜单

界面左侧用 ↑↓ 选择页面，Enter 打开；Tab / Shift-Tab 切换焦点。
检查表用 ↑↓ 选择模块，Enter 显示完整错误堆栈和原因；日志可用 PageUp / PageDown 滚动。
组件日志选择器用 Enter 展开、↑↓ 选择、Enter 确认。无需鼠标。

| 按键 | 操作 |
|---|---|
| s | 启动 / 修复后重试 |
| c | 并行预检查，汇总所有错误 |
| p | sudo 认证及华测串口权限处理 |
| x / Ctrl-C | 停止本次运行并清理登记进程，保留终端界面 |
| q / Ctrl-Q | 清理运行和采集进程并退出 |
| / | 聚焦命令输入框 |
| Space | 暂停 / 恢复日志刷新 |

命令输入框支持 `/check`、`/start`、`/permissions`、`/stop`、`/quit`，
以及 `/checks`、`/vehicle`、`/sensors`、`/logs`、`/modules`、`/config` 页面切换。
在输入框内字母是输入文本；全局快捷键应在菜单、检查表或日志区域使用。
启动、检查、清理使用同一个后台工作线程串行操作，独立检查内部并行；
耗时操作期间界面可继续浏览，停止和退出会排队执行，不会与启动操作同时修改进程。

交互模式发生检查、启动或运行错误时，显示异常堆栈、命令、退出码及原脚本的组件日志。
原脚本发生致命启动错误仍会停止其运行栈，管理器继续保留菜单供修复和重试。
不会自动重启控制器，也不会继续启动依赖失败组件的下游节点。
`--check` 是专用于脚本/CI的只读模式，完成全部检查后按成功/失败返回 0/1。

## 并行与依赖

预检查已按 `start_full_fsd.sh` 的实际条件扩展为11组：模块配置、ROS部署、华测串口、
M1网络、CAN硬件、外参与TF、RViz部署与桌面、工作空间产物、CAN桥占用、原启动器单实例、
只读任务监控文件。独立项目在线程池中并行执行，任一失败不会取消其他检查。
`--reuse-ins` 仍检查指定串口可读写，仅跳过独占检查，与原脚本一致。
RViz 开关和 build/skip-build 采用最后一个参数生效的原脚本规则。
未构建产物在自动构建模式显示“待构建”，不是“构建通过”；skip-build 缺产物则失败。
任务监控脚本缺失与原脚本一致仅警告，不阻止车辆模块启动。

同一检查表还列出 sudo、工作空间构建、环境加载、CAN初始化、全部模块进程、
只读任务监控，以及雷达、图像、惯导、融合、定位、任务状态六个数据检查。
这些启动阶段检查直接读取原脚本的进度输出，未执行时显示“待启动验证”，
只读预检查不会为了验证它们启动硬件。进程“已启动”不等于话题“已收到消息”。
sensor-only、reuse-ins、no-rviz、build-only 对不执行项显示“跳过”及原因。
运行停止后进程显示“已停止”，保留启动验证历史供查看。

启动包设置 `FSD_PARALLEL_CHECKS=1`，让原脚本的
M1、相机、惯导首帧检查并行进行，之后检查融合输出；定位、规划控制仍保留原来的依赖顺序。
管理包的 sensor-only 模式也检查已启动的惯导输出。
直接运行旧入口时仍使用原来的顺序检查。

## 权限与退出

已配置本机 `sudo_askpass.local.sh`，通过 `sudo -A` 自动认证，无需键入密码。
该文件权限为 700，已排除在 Git 提交之外；复制启动包到其他机器时需配置该本地文件。
密码不会进入命令行参数或日志。启动、p 操作、串口 ACL 和 root 残留清理均使用该认证。
CAN 沿用已有的 root 监管器创建/启用 vcan，
停止所选接口的旧桥并重新连接 USB CAN 设备。CAN 接口存在不表示已收到 VCU 报文；
请在组件日志中观察桥初始化和 VCU->FSD 计数。

华测串口按设备真实路径检查。如果当前用户没有读写权限，执行
`sudo setfacl -m u:<当前UID>:rw <串口>`，不设置全用户 666 权限。
需要系统提供 `setfacl`（Ubuntu 的 acl 包）；失败时保留错误详情和菜单。
ACL 用于当前设备节点，重插后可能需要重新处理。`--reuse-ins` 不授予外部串口权限，
但旧入口仍要求指定的串口存在且当前用户可读写；请传入外部链路实际使用的 `--ins-port`。

管理器持有 `flock` 单实例锁，每 250ms 跟踪启动子树，将 PID 与 Linux 进程启动时间保存在
`.runtime/fsd_manager/processes.json`。退出先请求原脚本正常清理，再对登记残留依次发送
INT、TERM、KILL；root 进程使用自动认证的 `sudo -A kill`。旧会话崩溃后，下次进入菜单会清理仍匹配
身份的登记残留。保留 `--reuse-ins` 的外部链路及其他不属于本次启动的 ROS 进程。
无法清理时保留登记记录和菜单，可用 p 重新认证后再 x / q，不将失败清理报告为成功。
本功能不使用全局 pkill；无法承诺清理登记前已脱离父进程的极短生命周期子树。

## 终端内实时状态与日志

页面包括启动检查、车辆状态、传感器参数、实时日志、组件日志、模块配置。
沿用终端自身的默认背景和文字色，不设置固定深灰底色；选中行使用反色高亮，边框弱化；
仅以柔和红色标记错误、淡黄色标记等待/超时、低饱和绿色标记成功/在线。
检查表的状态逐项更新，Enter 展开完整详情；错误不会把操作菜单冲出屏幕。
80×24 的终端也可使用，更窄时隐藏左侧菜单，可用命令切换页面。

车辆页显示任务门控、模式、GO/急停广播、设备自检、控制请求。
控制请求值不代表执行器实测反馈，任务门控以状态机诊断为准。
传感器页显示点云数量、图像分辨率/编码、惯导位置速度、融合锥桶数、
最近5秒订阅接收频率与距末帧时间。后台 ROS 进程只有订阅者，不发送控制消息。
不显示图像、点云或轨迹，与 RViz 的空间可视化分工。
状态采集在独立系统 Python 进程中运行，ROS 依赖与 Textual 依赖隔离；
采集错误及超过3秒未更新会在终端内显示。

日志沿用运行归档结构，组件日志页自动读取当前 `module_log_dir`。
配置页读取模块清单，不将清单声称为节点实际加载参数。

## 验证

```bash
/usr/bin/python3 -m pytest -q fsd_startup/test_startup.py fsd_startup/test_terminal_ui.py fsd_startup/test_check_alignment.py \
  tools/test_can_reconnect.py tools/test_privileged_group.py tools/test_chcnav_serial_guard.py
bash -n start_fsd_manager.sh start_full_fsd.sh fsd_startup/window.sh
```

测试覆盖并行检查的全错误收集、并行话题检查不触发 supervisor 清理、
失败后保留菜单、PID复用防护、脱离进程组的残留清理和外部进程保留。
测试不打开 USB CAN、不写串口、不发送车辆控制帧。

## 统一日志与归档

每次 s 启动（含预检查失败）生成独立运行目录；未选赛项先写 pending，进程停止且
输出收集结束后归档为 logs/<赛项>/<运行时间>。未运行时管理事件写入 sessions。
管理事件和任务监控统一进入 manager.log，不再生成 mission_monitor.log。
窗口读取 module_log_dir；菜单、重试、权限处理和PID清理流程保持不变。
完整目录和职责说明见 [运行日志](../docs/LOGGING.md)。
