# 运行日志

统一根目录为项目的 `logs/`。历史目录不删除、不自动迁移。

```text
logs/<赛项>/<YYYYmmdd_HHMMSS_微秒>/
  manager.log              启动、检查、任务诊断、停止与清理
  run.json                 运行标识、实车/仿真、退出原因及目录索引
  mission_history.json     只读监控观察到的实际赛项及选择时间
  configs/                 输入配置副本及部署参数清单
  jrajectory/              planned.csv、driven.csv 及分片
  modules/                 逐进程标准输出和错误输出
  ros/                     本次 ROS 日志
  artifacts/               锥桶地图、华测数据日志等结果
  validation/              仿真遥测、评估结果和 rosbag
```

赛项名称为 acceleration、skidpad、trackdrive、inspection、ebs_test。
未选择赛项使用 pending；多个赛项使用 multi_mission；仅传感器使用 sensor_only。
运行中不移动目录，结束并关闭写入者后归档。归档后内部绝对 latest 链接改为相对链接。
如果监控失败未观察到赛项，保留 pending，并在 manager.log 告警，不猜测默认赛项。

## 生命周期与职责

- `fsd_startup/log_context.py`：仅管理文件目录和运行元信息，不导入 ROS 或操作车辆进程。
- `manager.py`：创建每次启动上下文、通过环境传给全栈脚本、收集事件、原有进程监管。
- `tools/run_logged.py`：直接启动时唯一的输出收集入口，保留 stdin 和退出码，转发停止信号。
- `start_full_fsd.sh`：保留启动顺序，模块输出到 modules，传递地图和轨迹输出参数。
- `monitor_mission.py`：只读订阅，输出变化事件，单独维护 mission_history，不写 manager.log 或 run.json。
- `mission_diagnostics.py`：仅展示格式化，等待原因来自任务机；旧字段不足显示未知。
- `mission_manager`：保留当前硬件话题/QoS/点云就绪/发车复查，仅整理只读诊断。
- dashboard/终端界面：显示状态和读取目录，不作为任务诊断文件的写入者。

管理器会话可多次启动。未启动车辆时日志在 `logs/sessions/<时间>/manager.log`；
每次按 s 创建运行目录，预检查失败也保留记录。运行结束后返回会话日志，记录归档位置。
模块末尾故障摘要可在 manager.log 出现，模块全文仍保留在各自文件。
菜单 l 只显示尾部，不把历史尾部再次写入文件。

直接使用 `start_full_fsd.sh` 与使用 `start_fsd_manager.sh` 都只有一个管理日志写入者。
FSD_RUN_DIR/FSD_LOG_OWNER 是内部上下文；通常由入口生成，不需用户设置。
`--status`、`--stop`、`--print-config` 不创建新运行目录。
异常中断时目录保持原地；管理器下次恢复清理后标记、归档上次未结束的运行。
归档失败保留原目录，不改变车辆进程退出结果。

## 轨迹与外部数据

全栈提供 `record.run_dir`，规划节点在 jrajectory 保存 CSV。独立启动使用
`record.dir: logs/{mission}/{stamp}/jrajectory`，相对项目根目录解析。
赛项切换或同次运行内规划节点重启，另开 segments 子目录，避免覆盖原 CSV。
保留原 CSV 字段、源时间戳、刷新周期和分片机制。

华测通过已有 --log-dir 参数写入 artifacts/chcnav；驱动原有内部时间目录保留。
使用 --reuse-ins 时不改动外部已有进程，其历史输出不自动复制进本次目录。
当前实车入口未启用 NDT/map_saver，重构不额外启动它们；启用时需要成对传入
map_save_path/map_path 指向 artifacts，不能只改写入端。
EKF debug 保持默认关闭；如显式启用，应提供本次目录中的 debug_out_file。

## 仿真

仿真默认按赛项创建目录，run.json 标记 simulation。--output 仍支持显式路径，
保持指定位置不移动。数据分别进入 configs/modules/jrajectory/artifacts/validation。
回归批次在 logs/sessions/batch_* 保存索引与报告，各场景为指向赛项运行目录的链接。
汇总与几何复核工具兼容旧布局和新布局。隔离 ROS 域和故障注入规则保持原样。

配置副本表示输入文件；命令行覆盖可见于部署清单或 processes.json，不等同于运行参数查询。

## 本次验证（2026-10-12）

- mission_manager、path_generator 已完成定向编译。
- 启动管理、日志生命周期、终端界面、模块配置、权限/CAN辅助工具及启动检查对齐共56项无硬件测试通过。
- 隔离域 222 的任务机心跳/质量及轨迹切换/重启防覆盖共 6 项通过。
- 隔离域 221 的直线运行、急停故障仿真各运行 22 秒，原有指标检查通过。
- 伪终端验证直接日志入口保留 /dev/tty，并正常响应 Ctrl-C。
- 与实施前备份对比：发车、状态转换、GO、急停、模式选择、自检、计圈 7 个控制回调未变。

未启动实车桥、未重连 CAN、未执行实车动作验证。未启用的独立 NDT 入口未改写。
历史日志保留原地。实车复用的外部惯导日志仍由外部进程管理。
第一段轨迹位于 jrajectory 根目录；后续赛项切换/关闭重开/节点重启的记录在
segments/<赛项>/<时间_序号>/。分析工具应递归读取 CSV，不只读取根目录文件。
