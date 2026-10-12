from pathlib import Path
import json,sys
p=Path(sys.argv[1]);items={}
for case in p.iterdir():
 if not case.is_dir():continue
 f=case/'validation/summary.json'
 if not f.exists():f=case/'summary.json'
 if f.exists():items[case.name]=json.loads(f.read_text())
for case in p.iterdir():
 if not case.is_dir():continue
 f=case/'validation/failure.json'
 if not f.exists():f=case/'failure.json'
 if f.exists():items[case.name]={'pass':False,**json.loads(f.read_text())}
(p/'summary.json').write_text(json.dumps(items,ensure_ascii=False,indent=2))
s='# WUTA-SIM 仿真验证记录\n\n仅有限时长片段，未覆盖全圈、真实YOLO和真实CAN。\n\n| 工况 | 判定 | 最高车速 m/s | 最大横向绝对坐标 m | 最大转角 ° | 未通过项 |\n|---|---|---|---|---|---|\n'
for k,v in items.items():s+=f"| {k} | {'通过' if v.get('pass') else '失败'} | {v.get('max_actual_speed')} | {v.get('max_abs_y')} | {v.get('max_angle')} | {[x for x,y in v.get('checks',{}).items() if not y] or v.get('error','')} |\n"
for f in list(p.glob('*/geometry_review.json')) + list(p.glob('*/validation/geometry_review.json')):
 v=json.loads(f.read_text());s+=f"\n{f.parent.parent.name if f.parent.name == 'validation' else f.parent.name} 的单独几何复核：{'通过' if v['pass'] else '失败'}，最大线段距离 {v['max_distance_to_planned_polyline_m']:.3f} m；原离散点判定保留。\n"
s+='\n横向绝对坐标只在沿地图x轴的直线工况可解释为横向误差。每个工况目录保留配置快照、遥测CSV、节点日志、轨迹和rosbag。\n'
(p/'report.md').write_text(s);print(s)
