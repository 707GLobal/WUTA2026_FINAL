"""Display-only formatting; authoritative gates come from mission_manager."""
WAIT_LABELS = {'lidar': '雷达ready', 'localization': '定位ready', 'mode': '任务选择', 'go': '本次任务GO', 'map_closed': '地图闭合', 'map_quality': '地图质量', 'centerline': '全局中心线', 'pose': '新鲜定位位姿', 'confidence': '定位置信度', 'ndt': 'NDT地图', 'lap': '首圈完成', 'emergency': '急停锁存，GO不能解除', 'finished': '任务已完成'}

STATES = {
    0: 'IDLE 等待就绪', 1: 'READY 已就绪', 2: 'INSPECTION 车检执行中',
    3: 'EXPLORE 任务执行中', 4: 'MAPPING_DONE 建图完成',
    5: 'RACE 比赛中', 6: 'FINISH 任务完成', 7: 'EMERGENCY 急停',
}
MODES = {0: 'trackdrive 高速循迹', 1: 'skidpad 八字绕环',
         2: 'acceleration 直线加速', 3: 'inspection 车检', 4: 'ebs_test EBS测试'}
REQUIRED = {'mode_selected', 'go_count', 'go_pending', 'lidar_ready', 'localization_ready'}


def describe(state, mode, loc, description):
    """Build stable display lines from authoritative mission-manager diagnostics."""
    fields = {}
    for token in description.split():
        key, sep, value = token.partition('=')
        if sep:
            try:
                fields[key] = int(value)
            except ValueError:
                fields[key] = value
    lines = {'阶段': STATES.get(state, f'未知状态 {state}'),
             '定位模式': {0: 'EKF通道（LOC_KISS_ICP）', 1: 'NDT'}.get(loc, str(loc)),
             '圈数': str(fields.get('lap', '未知'))}
    if not REQUIRED.issubset(fields):
        lines['门控'] = '当前状态机未提供门控诊断；任务选择和 GO 状态未知'
        lines['任务模式'] = MODES.get(mode, str(mode)) + '（仅模式字段，不能确认已选择）'
        return lines
    selected = fields['mode_selected'] == 1
    lines['任务选择'] = MODES.get(mode, str(mode)) if selected else '尚未选择（默认模式不代表已选择）'
    lines['就绪输入'] = (f"雷达ready={'是' if fields['lidar_ready'] else '否'}，"
                       f"定位ready={'是' if fields['localization_ready'] else '否'}")
    count = fields['go_count']
    pending = fields['go_pending'] == 1
    lines['GO'] = (f"累计收到 {count} 次，当前{'有待处理GO' if pending else '无待处理GO（已消费或清除）'}"
                   if count else '尚未收到')
    if 'wait' in fields:
        reasons = fields['wait']
        lines['下一步'] = '等待：' + '、'.join(WAIT_LABELS.get(k, k) for k in reasons.split(',')) if reasons != 'none' else '当前无待满足门槛'
        return lines
    lines['下一步'] = '当前版本未提供权威等待原因'
    return lines

