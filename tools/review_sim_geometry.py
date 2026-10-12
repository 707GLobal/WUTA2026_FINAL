"""离线复核规划前向窗口：距离到线段，而不是距离到离散前方航点。保留原判定。"""
import math,json,sys
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from autoware_msgs.msg import Lane
from nav_msgs.msg import Odometry
p=Path(sys.argv[1]);p=p/'validation' if (p/'validation/rosbag').exists() else p;reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(p/'rosbag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''));pts=[];n=0;maxd=0;maxyaw=0;extrapolated=0
while reader.has_next():
 topic,data,stamp=reader.read_next()
 if topic=='/planning/final_waypoints':
  lane=deserialize_message(data,Lane);pts=[(w.pose.pose.position.x,w.pose.pose.position.y) for w in lane.waypoints]
 if topic!='/sim/ground_truth' or len(pts)<2:continue
 o=deserialize_message(data,Odometry)
 if o.twist.twist.linear.x<.2:continue
 x,y=o.pose.pose.position.x,o.pose.pose.position.y;q=o.pose.pose.orientation;yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z);best=None
 for i,((ax,ay),(bx,by)) in enumerate(zip(pts,pts[1:])):
  vx,vy=bx-ax,by-ay;den=vx*vx+vy*vy
  if den<1e-9:continue
  t=((x-ax)*vx+(y-ay)*vy)/den
  t=max(-5/math.sqrt(den) if i==0 else 0,min(1,t))
  d=math.hypot(x-ax-t*vx,y-ay-t*vy)
  if best is None or d<best[0]:best=(d,math.atan2(vy,vx),i,t)
 if best:
  n+=1;maxd=max(maxd,best[0]);err=abs(math.atan2(math.sin(yaw-best[1]),math.cos(yaw-best[1])));maxyaw=max(maxyaw,err);extrapolated+=int(best[3]<0)
result={'samples':n,'max_distance_to_planned_polyline_m':maxd,'max_heading_error_deg':math.degrees(maxyaw),'forward_window_back_extension_samples':extrapolated,'definition':'仅第一段允许向后延伸最多5米（当前循迹最大前视距离），其他线段严格限定端点；原离散点检查保留不覆盖','pass':n>100 and maxd<1.5 and maxyaw<math.pi/2,'scope':'相对算法规划路径，不是独立道路/车辆碰撞几何证明'}
(p/'geometry_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
