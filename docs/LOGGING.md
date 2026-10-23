# 运行日志

统一根目录为项目的 `wuta_logs/`。历史 `logs/` 不删除、不自动迁移。

```text
wuta_logs/
├── pending/<YYYYmmdd_HHMMSS>/          # 仅模式缓存不可用时兜底
├── <赛项>/<YYYYmmdd_HHMMSS>/           # 正常运行
│   ├── manager.log
│   ├── manager.events.jsonl            # 同一写入者追加的 CLI 事件与 Unix 纳秒时间
│   ├── manager.radio.events.jsonl      # 必要 CLI 事件与原源时间（0.5.0 无线增量源）
│   ├── run.json
│   ├── configs/
│   ├── trajectory/
│   ├── fsd_modules/
│   ├── external_log/
│   ├── zlgCAN_log/
│   ├── chcnav_log/
│   ├── ros/
│   ├── artifacts/
│   └── validation/                     # 仅仿真
├── sensor_only/<YYYYmmdd_HHMMSS>/
└── sessions/
    ├── <YYYYmmdd_HHMMSS>/manager.log
    └── batch_*/
```

赛项名称为 `acceleration`、`skidpad`、`trackdrive`、`inspection`、`ebs_test`。

## 赛项目录来源

正常实车流程：

```text
VCU 选择模式
  → CAN 0x501
  → ZLG 桥写 /run/zlgcan_bridge/<iface>.mode
  → 启动入口读取并校验缓存
  → 创建 wuta_logs/<赛项>/<YYYYmmdd_HHMMSS>/
```

模式映射：2=acceleration，3=trackdrive，4=skidpad，5=ebs_test，6=inspection。
0、1、7+ 或缓存无效时先落 `pending`，收到实时模式后由只读监控补记
`run.json.mission`，结束时移动目录。

`pending` 只用于兜底；正常流程不经过 `pending`，也不生成
`mission_history.json`。一次运行只认一个赛项，不生成 `multi_mission`。

## 生命周期与职责

- `fsd_startup/log_context.py`：仅管理文件目录和运行元信息，不导入 ROS。
- `fsd_startup/mode_cache.py`：启动前读取 ZLG 模式缓存，仅用于日志目录分类，
  不发布任务状态。
- `fsd_startup/manager.py`：创建启动上下文、读取模式缓存、收集事件、监管进程。
- `tools/run_logged.py`：直接启动时唯一的输出收集入口，保留 stdin、退出码和信号。
- `start_full_fsd.sh`：启动顺序、模块输出、设备日志路径和轨迹参数。
- `monitor_mission.py`：只读订阅；仅缓存兜底时补写 `run.json.mission`。
- `mission_manager`：唯一任务状态、正式圈次和门控诊断来源；日志目录不复制该状态。

管理器会话可多次启动。未启动车辆时日志在 `wuta_logs/sessions/<YYYYmmdd_HHMMSS>/manager.log`；
每次按 `s` 创建运行目录，预检查失败也保留记录。

直接使用 `start_full_fsd.sh` 与使用 `start_fsd_manager.sh` 都只有一个管理日志写入者。
`FSD_RUN_DIR`/`FSD_LOG_OWNER` 是内部上下文，通常由入口生成。

`--status`、`--stop`、`--print-config` 不创建新运行目录。异常中断时目录保持原地；
管理器下次恢复清理后标记、归档上次未结束的运行。归档失败保留原目录。

## 目录内容

| 目录/文件 | 内容 |
| --- | --- |
| `manager.log` | 启动、检查、任务诊断、停止与清理 |
| `manager.events.jsonl` | 同一事件写入者的完整 CLI 事件、级别与 Unix 纳秒时间（本地全文） |
| `manager.radio.events.jsonl` | 必要 CLI 事件与原源时间（0.5.0 无线增量源） |
| `run.json` | 运行标识、模式、赛项来源、VCU 模式、退出原因及目录索引 |
| `configs/` | 输入配置副本及部署参数清单 |
| `trajectory/` | `planned.csv`、`driven.csv` 及分片 |
| `fsd_modules/` | FSD 包节点输出，按 perception/localization/mapping/planning/control/system 分类 |
| `external_log/` | ZED/雷达驱动、静态 TF、RViz/显示桥 |
| `zlgCAN_log/` | ZLG 控制台日志和逐帧报文日志 |
| `chcnav_log/` | 华测控制台、数据日志和分层诊断 |
| `ros/` | 本次 ROS 日志 |
| `artifacts/` | 锥桶地图等业务结果 |
| `validation/` | 仿真遥测、评估结果和 rosbag |

运行目录、会话目录和批次目录的时间戳精确到秒（`YYYYmmdd_HHMMSS`）；RunLog 在同一秒内
重名时追加 `_01`、`_02` 等序号，不再使用微秒。

`trajectory/segments/<赛项>/<YYYYmmdd_HHMMSS>_<序号>/` 用于赛项切换或规划节点重启后的
新记录段，分析工具应递归读取。

## 设备日志

- 华测通过已有 `--log-dir` 参数写入 `chcnav_log`；驱动内部时间目录和 `latest`
  链接保留。
- ZLG 桥由本次启动时通过 `--log-dir` 写入 `zlgCAN_log`；复用外部桥时，报文日志
  路径由外部桥配置决定，FSD 只在 `zlgCAN_log/bridge.log` 记录复用信息。
- 使用 `--reuse-ins` 时不改动外部华测进程，其历史输出不自动复制进本次目录。

## 仿真

仿真默认按赛项创建目录，`run.json` 标记 `simulation`。`--output` 仍支持显式路径，
保持指定位置不移动。数据分别进入 `configs/fsd_modules/external_log/trajectory/
artifacts/validation`。回归批次在 `wuta_logs/sessions/batch_*` 保存索引与报告，
各场景为指向赛项运行目录的链接。

## 历史

旧 `logs/` 目录及其中 `jrajectory/`、`modules/`、`mission_history.json` 作为历史证据
保留，不自动迁移；新代码只写 `wuta_logs/`。

### 日志重构阶段验证（2026-10-12，历史范围）

- mission_manager、path_generator 已完成定向编译。
- 启动管理、日志生命周期、终端界面、模块配置、权限/CAN 辅助工具及启动检查对齐
  共 56 项无硬件测试通过。
- 隔离域 222 的任务机心跳/质量及轨迹切换/重启防覆盖共 6 项通过。
- 隔离域 221 的直线运行、急停故障仿真各运行 22 秒，原有指标检查通过。
- 伪终端验证直接日志入口保留 `/dev/tty`，并正常响应 Ctrl-C。
- 与实施前备份对比：发车、状态转换、GO、急停、模式选择、自检、计圈 7 个控制
  回调未变。

上述验证未启动实车桥、未重连 CAN、未执行实车动作验证；这不表示 10 月 12 日整天
没有其他实车诊断。未启用的独立 NDT 入口未改写。历史日志保留原地；实车复用的外部
惯导日志仍由外部进程管理。历史轨迹第一段位于当时的 `jrajectory` 根目录，后续段在
`segments/<赛项>/<当时时间_序号>/`（当时含微秒）；分析历史数据时仍应递归读取。

### 后续惯导诊断与门控变更

华测分层诊断 JSONL 跟随运行目录归档，当前位于 `chcnav_log/diagnostics/`
（`parser.jsonl`、`protocol.jsonl`、`bridge.jsonl`），覆盖定位输入、解析和发布各层；
记录语义见[惯导诊断](惯导临时诊断记录.md)。历史运行中的
`artifacts/chcnav_diagnostic/` 位置保持不变。

GNSS 默认发布门槛由 1 改为 0 及状态机门控调整属于后续代码变更，不应引用日志重构
阶段“控制回调未变”来证明其无变化。最终 GNSS 改动仅静态核对和定向编译，未做当前
配置完整实车验收。各阶段证据和时间点统一见
[10月12日工作记录](工作记录_2026-10-12.md)。

## CLI无线增量日志与传感器（2026-10-19）

Manager.emit在同一事件临界区追加原manager.log及manager.events.jsonl；直接全栈入口由原tools/run_logged.py输出收集器调用同一journal函数。追加函数位于CLI子模块log_context.py，不创建新日志生命周期。事件保留CLI记录时刻的Unix纳秒时间和完整原文，脚本报错与堆栈走同一路径。源端journal与PC续传断点需保留；删除源文件不能靠无线恢复。旧版未记录的源时间不补造。

CLI dashboard复用原订阅，将GNSS、惯导、雷达、相机、融合及车辆/任务摘要写入结构化telemetry.json字段；没有Header时标未知，接收时间单列，不替代采集时间。无线组件读取文件，不自行判定驾驶任务或增加原始传感器发布者。

0.5.0按用户要求只将必要事件写入独立`manager.radio.events.jsonl`：错误/警告及连续诊断上下文、任务监控变化、启动/退出汇总。菜单、窗口、逐项成功和普通构建输出不占无线。规则由同一log_context.py负责；manager.log及完整journal不删减，新子集使用独立stream ID。

PC持久化收到的必要事件与其完整文字，业务确认后Linux推进字节偏移；重传去重、进程重启续传、积压及Foxglove有界缓存说明见[radio_link](../fsd-visualization-code/radio_link/README.md)。关键事件仍可能排队；未筛选的0.4.0 journal不再自动传输，全部CLI历史仍保留在Linux。

0.5.1扫描新`wuta_logs/`和旧`logs/`下的必要journal，补充状态文件的当前/已结束运行路径；CLI切回会话和pending移动归档时，按stream ID重新定位，磁盘确认无需等待下一次周期扫描。两端断点继续有效，不需要清队列。非标准自定义路径在无线重启后仍需被状态文件引用；无状态的直接全栈标准目录可自动发现。

CLI两种组件日志界面共用`dashboard.component_log_files`，覆盖`fsd_modules`、`external_log`、`chcnav_log`、`zlgCAN_log`，支持旧目录并保留分类标签。感知驱动组单列进程状态；点云/图像首帧失败摘要读取`external_log/perception_drivers.log`，融合失败仍读取算法组日志。组件日志列表扩大不增加无线全文传输范围。

## 0.6.0：只转发当前运行（2026-10-20）

此版本替代0.5.1的历史目录发现策略：CLI/RunLog原子发布 `.runtime/fsd_manager/radio_run.json`，无线只读取其选定的manager.radio.events.jsonl。CLI重新打开先清空旧选择，运行创建后切换到本次scope；运行结束保留该选择以发送尾部，pending归档更新路径。下一次运行丢弃旧运行的内存待发块，历史文件与字节断点保留在磁盘，不继续占用链路。

当前运行的必要日志仍完整保留原文、错误堆栈、源时间和业务落盘确认；普通info/ok级GNSS状态行不再重复传文字，GNSS最新详情由传感器快照提供。CLI实时日志与无线选择同步，PC只回放当前运行，Foxglove新scope清理前一轮日志和图表会话。当前运行产生速度持续高于实际链路吞吐时仍会积压，不能承诺无限日志零延迟。本轮按用户要求未运行测试或现场验证。

0.6.1保持新运行清空旧日志策略，同时在Foxglove会话重置后重新公告通道，修复CLI重启后所有面板等待数据的问题；完整历史和当前运行断点仍保留。

## 0.6.2：完整转发当次CLI日志（当前策略）

按用户确认，从当前RunLog的manager.events.jsonl读取全部事件，取消无线必要信息筛选；普通GNSS状态、检查进度、普通构建输出、菜单提示和完整错误堆栈，只要由CLI主日志记录，均携带原源时间顺序发送。manager.radio.events.jsonl保留为旧兼容产物，其筛选不影响新版无线源。组件独立日志文件没有进入CLI主日志的部分不另行上传。仍仅选择当前运行，新scope切走旧队列；PC全文落盘、显示有界缓存和持久化确认继续沿用。

## CLI管理请求与配置事务（无线0.7.0）

`.runtime/fsd_manager/control/`是管理会话状态，独立于RunLog归档：heartbeat.json含boot/运行/活动请求；inbox/<UUID>.json为请求；results/<UUID>.json先写executing再写最终ok/result/error/source_stamp_ns；transactions/<UUID>.json含配置原内容备份和提交阶段，config_state.json保守记录待确认加载项。配置保存/接口失败仍由原Manager.emit记入当前CLI日志，不另建业务日志生命周期。

结果UUID、请求内容hash和持久executing标记用于去重。执行途中CLI崩溃可能留下未知结果，不擅自重新执行；可用control_api.py cli.result --request-id查询。未完成请求/事务存在时不要删除该目录；事务恢复发现外部文件冲突会报错待人工处理。启动/停止不替换原运行journal、scope或完整日志传输规则。

## 人工驾驶记录（2026-10-22）

CLI `M` 循环选择“有人跑动记录”后按 `S` 启动、`X` 停止，保存于 `wuta_logs/manual_record/<YYYYmmdd_HHMMSS>/`。沿用正常日志生命周期和文件格式：manager日志/journal、run.json（mode=manual_record）、配置快照、模块/ROS输出、华测数据及诊断、ZLG逐帧接收日志、trajectory CSV及地图产物（满足原保存条件时）。不新增rosbag或另一套日志收集器，原始点云/图像不因这个模式自动落盘。

实际轨迹从有效 `/localization/pose` 到达开始记录，无需VCU选档或RES GO；速度仍来自华测。未启动控制器，CSV命令字段及时间保持nan，不能推断人工驾驶的油门/转角。自动任务未激活时planned.csv可只有表头，不生成虚构计划或圈次。只收CAN桥强制启用现有帧日志，VCU原始报文在 `zlgCAN_log/<桥时间戳>/vcu2fsd/0x501.log` 等文件中；本模式不产生硬件发送日志。自检故障/急停持续记录，CLI停止后沿原清理/落盘流程结束归档。运行目录不会被默认trackdrive任务字段重新分类。
