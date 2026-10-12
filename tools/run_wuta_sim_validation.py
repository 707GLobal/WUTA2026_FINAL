"""隔离ROS域内运行当前FSD和WUTA-SIM，保存原始记录及明确的通过/失败结论。"""
import argparse,csv,json,math,os,signal,subprocess,time,hashlib
from pathlib import Path
from sim_metrics import path_distance
import yaml,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped,PoseStamped
from autoware_msgs.msg import Command,Lane,Waypoint
from wuta_msgs.msg import MissionState,ConeArray,ConeMap
ROOT=Path(__file__).resolve().parents[1];SRC=ROOT/'WUTA-FSD/ros2_ws/src'
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--duration',type=float,default=40);ap.add_argument('--mission',choices=['acceleration','trackdrive','skidpad'],default='acceleration');ap.add_argument('--fault',choices=['none','velocity_dropout','pose_dropout','empty_path','nan_path','emergency'],default='none');ap.add_argument('--speed',type=float,default=2.);ap.add_argument('--start-y',type=float,default=.3);ap.add_argument('--start-yaw',type=float,default=.05);ap.add_argument('--noise',type=float,default=.05);ap.add_argument('--output',default=None);ap.add_argument('--stationary',action='store_true');ap.add_argument('--legacy-colors',action='store_true');a=ap.parse_args()
    if a.duration<15 or not 0<a.speed<=20:ap.error('duration>=15；0<speed<=20')
    out=Path(os.environ['FSD_RUN_DIR']);procs=[];observers=[];logs=[];snap={}
    def start(name,pkg=None,exe=None,params=None,config=None,remaps=None,script=None):
        cmd=['/usr/bin/python3',str(script)] if script else [str(Path(get_package_prefix(pkg))/'lib'/pkg/exe)]
        cmd+=['--ros-args']
        for i,f in enumerate(config or []):
            copy=out/'configs'/(name+'_config'+str(i)+'.yaml');copy.write_text(Path(f).read_text());cmd+=['--params-file',str(copy)]
        for k,v in (params or {}).items():cmd+=['-p',k+':='+('true' if v is True else 'false' if v is False else str(v))]
        for k,v in (remaps or {}).items():cmd+=['-r',k+':='+v]
        f=(out/'modules'/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,p));snap[name]={'command':cmd,'pid':p.pid,'executable_sha256':hashlib.sha256(Path(cmd[0]).read_bytes()).hexdigest()};return p
    config=yaml.safe_load((SRC/'control/controller/config/controller.yaml').read_text());vp=config['controller_node']['ros__parameters']
    if a.stationary:
        for mission in ['acceleration','ebs','skidpad','trackdrive']:
            vp[mission]['throttle']['max_opening']=0.
            vp[mission]['brake']['max_opening']=0.
    planning=yaml.safe_load((SRC/'planning/path_generator/config/path_generator.yaml').read_text())
    pp=planning['path_generator_node']['ros__parameters']
    if a.mission in ('acceleration','skidpad'):
        pp[a.mission]['speed']['velocity']=a.speed
    else:
        for group in ('explore','race'):
            pp['trackdrive']['speed'][group]['min_velocity']=min(pp['trackdrive']['speed'][group]['min_velocity'],a.speed)
        pp['trackdrive']['speed']['explore']['max_velocity']=a.speed
        for key in ('lap2_max_velocity','lap3_max_velocity'):
            pp['trackdrive']['speed']['race'][key]=a.speed
        pp['trackdrive']['speed']['launch']['velocity']=min(pp['trackdrive']['speed']['launch']['velocity'],a.speed)
        pp['trackdrive']['speed']['degraded_velocity']=min(pp['trackdrive']['speed']['degraded_velocity'],a.speed)
    (out/'configs'/'planning.yaml').write_text(yaml.safe_dump(planning,allow_unicode=True,sort_keys=False))
    (out/'configs'/'control.yaml').write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False))
    (out/'configs'/'settings.json').write_text(json.dumps({'args':vars(a),'domain':os.environ.get('ROS_DOMAIN_ID'),'source_hash':hashlib.sha256((SRC/'control/controller/config/controller.yaml').read_bytes()).hexdigest(),'note':'模拟相机检测，不运行YOLO；不启动真实CAN；开度驱动车辆适配模型'},ensure_ascii=False,indent=2))
    track_source=Path(get_package_prefix('lidar_sim'))/'share/lidar_sim/tracks'/(a.mission+'.yaml')
    track=yaml.safe_load(track_source.read_text());body=track.get('track',track)
    converted=False
    if not a.legacy_colors and not body.get('red_cones') and body.get('blue_cones') and body.get('yellow_cones'):
        body['red_cones']=body.pop('blue_cones');body['blue_cones']=body.pop('yellow_cones');converted=True
    track_path=out/'configs'/'track.yaml';track_path.write_text(yaml.safe_dump(track,sort_keys=False))
    (out/'configs'/'track_contract.json').write_text(json.dumps({'source':str(track_source),'converted_legacy_blue_yellow_to_red_blue':converted},indent=2))
    rclpy.init();n=Node('wuta_validation_recorder');latest={};counts={};samples=[];states=set();fault_at=None;fault_pub=None
    velpub=n.create_publisher(TwistStamped,'/chcnav/velocity',10)
    def odom(m):
        latest['ins']=m
        if not (fault_at is not None and a.fault=='velocity_dropout'):
            v=TwistStamped();v.header=m.header;v.twist=m.twist.twist;velpub.publish(v)
    n.create_subscription(Odometry,'/chcnav/odometry',odom,10)
    def cb(key):
        def receive(m):latest[key]=m;counts[key]=counts.get(key,0)+1
        return receive
    n.create_subscription(Odometry,'/sim/ground_truth',cb('truth'),10);n.create_subscription(Command,'/control/command',cb('command'),10);n.create_subscription(PoseStamped,'/localization/pose',cb('pose'),10);n.create_subscription(Lane,'/planning/final_waypoints',cb('path'),10);n.create_subscription(ConeArray,'/perception/fused/cones',cb('fusion'),10);n.create_subscription(ConeMap,'/mapping/cone_map',cb('map'),10)
    n.create_subscription(MissionState,'/system/mission_state',lambda m:(latest.update(state=m),states.add(m.state)),10)
    if a.fault in ['empty_path','nan_path']:fault_pub=n.create_publisher(Lane,'/planning/final_waypoints',10)
    if a.fault=='emergency':
        from std_msgs.msg import Bool
        fault_pub=n.create_publisher(Bool,'/system/emergency',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        discovery=time.monotonic()+1.
        while time.monotonic()<discovery:rclpy.spin_once(n,timeout_sec=.05)
        others=[name for name in n.get_node_names() if name!=n.get_name()]
        if others:raise RuntimeError('仿真域已有其他节点，拒绝混接：'+str(others))
        start('vehicle',script=ROOT/'tools/wuta_sim_effort_vehicle.py',params={'wheel_base':vp['vehicle']['wheel_base'],'max_steer_angle':vp['vehicle']['max_steer_angle'],'start_x':-.3 if a.mission=='acceleration' else -15. if a.mission=='skidpad' else 0.,'start_y':a.start_y,'start_yaw':a.start_yaw})
        start('ins','ins_simulator','ins_simulator',params={'output_topic':'/chcnav/odometry','frame_id':'odom','position_noise_std':a.noise,'yaw_noise_std_deg':.5,'seed':42})
        start('lidar','lidar_sim','lidar_simulator_node',params={'track_file':str(track_path)})
        start('bridge','simulator_bringup','simulation_bridge',params={'mission_mode_cmd':a.mission,'publish_truth_localization':False,'publish_start_command':True})
        start('camera','simulator_bringup','simulated_stereo_detections',params={'track_file':str(track_path)})
        for name,xyz,q,parent,child in [('lidar_tf',[0,0,1],[0,0,0,1],'base_link','lidar'),('map_tf',[0,0,0],[0,0,0,1],'map','odom'),('cam_tf',[0,0,.5],[-.5,.5,-.5,.5],'base_link','sim_camera_left_optical_frame')]:
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher']
            for k,v in zip(['x','y','z','qx','qy','qz','qw'],xyz+q):cmd+=['--'+k,str(v)]
            cmd+=['--frame-id',parent,'--child-frame-id',child];f=(out/'modules'/(name+'.log')).open('w');logs.append(f);procs.append((name,subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)))
        start('ekf','robot_localization','ekf_node',config=[SRC/'localization/localization_manager/config/ekf.yaml'],remaps={'__node':'ekf_node'})
        loc=start('localization','localization_manager','localization_manager_node')
        start('detector','lidar_detection','lidar_detection_node',config=[SRC/'perception/lidar_detection/config/lidar_detection.yaml'],params={'input_topic':'/hesai/pandar','output_topic':'/perception/lidar/cones_raw','use_ransac':False,'ground_z_threshold':-.95,'voxel_leaf_size':.1,'voxel_before_ground':False})
        start('fusion','detection_fusion','detection_fusion_node_cpp',config=[SRC/'perception/detection_fusion/config/fusion.yaml'],params={'fixed_frame':'odom','guided_clustering':False,'pointcloud_topic':'/hesai/pandar','fuse_positions':False,'publish_unmatched_lidar':False,'min_color_probability':.6})
        start('mapping','cone_map_builder','cone_map_builder_node',config=[SRC/'mapping/cone_map_builder/config/cone_map_builder.yaml'],params={'assign_colors':False,'map_save_path':str(out/'artifacts'/'cone_map.yaml')},remaps={'/perception/lidar/cones':'/perception/fused/cones'})
        start('boundary','boundary_detector','boundary_detector_node',config=[SRC/'planning/boundary_detector/config/boundary_detector.yaml'])
        planner=start('planning','path_generator','path_generator_node',config=[out/'configs'/'planning.yaml'],params={'record.run_dir':str(out)})
        start('controller','controller','controller_node',config=[out/'configs'/'control.yaml'])
        start('mission','mission_manager','mission_manager_node',config=[SRC/'system/mission_manager/config/mission_manager.yaml'],params={'lidar_topic':'/hesai/pandar','imu_topic':'/chcnav/odometry','check_camera':False})
        observers.append(('mission_observer', subprocess.Popen(
            ['/usr/bin/python3', '-u', str(ROOT/'tools/monitor_mission.py'), '--run-dir', str(out)],
            start_new_session=True)))
        f=(out/'modules'/'rosbag.log').open('w');logs.append(f);procs.append(('bag',subprocess.Popen(['ros2','bag','record','-o',str(out/'validation'/'rosbag'),'/hesai/pandar','/perception/camera/cones','/perception/camera/camera_info','/tf','/tf_static','/system/emergency','/sim/ground_truth','/chcnav/odometry','/chcnav/velocity','/localization/pose','/control/command','/planning/final_waypoints','/mapping/cone_map','/perception/fused/cones','/system/mission_state'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)))
        (out/'validation'/'processes.json').write_text(json.dumps(snap,ensure_ascii=False,indent=2));t0=time.monotonic();last_sample=0
        with (out/'validation'/'telemetry.csv').open('w',newline='') as file:
            w=csv.writer(file);w.writerow(['elapsed','x','y','yaw','actual_speed','target_speed','angle_deg','effort','state','fault','distance_to_planned_path','cross_track_polyline_distance'])
            while time.monotonic()-t0<a.duration:
                rclpy.spin_once(n,timeout_sec=.02);t=time.monotonic()-t0
                dead=[name for name,p in procs if p.poll() is not None]
                if dead:raise RuntimeError('节点提前退出:'+str(dead))
                if t>a.duration*.55 and a.fault!='none' and fault_at is None:
                    fault_at=t
                    if a.fault=='pose_dropout':os.kill(loc.pid,signal.SIGSTOP)
                    if a.fault in ['empty_path','nan_path']:os.kill(planner.pid,signal.SIGSTOP)
                if fault_at is not None and fault_pub:
                    if a.fault=='emergency':
                        m=Bool();m.data=True
                    else:
                        m=Lane()
                        if a.fault=='nan_path':wp=Waypoint();wp.pose.pose.position.x=float('nan');m.waypoints=[wp]
                    fault_pub.publish(m)
                if t-last_sample<.05 or 'truth' not in latest or 'command' not in latest:continue
                last_sample=t;o=latest['truth'];c=latest['command'];q=o.pose.pose.orientation;yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
                row=[t,o.pose.pose.position.x,o.pose.pose.position.y,yaw,o.twist.twist.linear.x,c.speed,c.angle,c.throttle_brake,latest['state'].state if 'state' in latest else -1,int(fault_at is not None)];path=latest.get('path');dist=None
                if path and path.waypoints and all(math.isfinite(wp.pose.pose.position.x) and math.isfinite(wp.pose.pose.position.y) for wp in path.waypoints):dist=min(math.hypot(row[1]-wp.pose.pose.position.x,row[2]-wp.pose.pose.position.y) for wp in path.waypoints)
                row.append(dist);row.append(path_distance(row[1],row[2],[(wp.pose.pose.position.x,wp.pose.pose.position.y) for wp in path.waypoints]) if dist is not None else None);samples.append(row);w.writerow(row);file.flush()
        steady=[r for r in samples if r[0]>8];post=[r for r in samples if fault_at is not None and r[0]>fault_at+1]
        checks={'commands_present':len(steady)>20,'motion_observed':any(r[4]>.2 for r in steady) if not a.stationary else all(abs(r[4])<.05 for r in steady),'finite':all(all(math.isfinite(v) for v in r[1:8]) for r in samples),'target_limit':all(0<=r[5]<=a.speed+1e-5 for r in samples),'angle_limit':all(abs(r[6])<=vp['vehicle']['max_steer_angle']+1e-5 for r in samples),'effort_limit':all(abs(r[7])<=1.00001 for r in samples),'zero_target_no_drive':all(r[7]<=1e-6 for r in samples if r[5]<=1e-6),'actual_speed_bound':all(-.05<=r[4]<=a.speed+.6 for r in steady),'sensor_chain':all(counts.get(k,0)>0 for k in (['fusion','map','pose'] if a.stationary else ['fusion','map','pose','path']))}
        if a.fault=='none' and not a.stationary:checks['path_tracking_distance']=bool(steady) and all(r[11] is not None and r[11]<1.5 for r in steady)
        if a.stationary:checks['stationary_no_false_closure']='map' in latest and not latest['map'].is_closed
        if a.mission=='acceleration':checks['straight_lateral_error']=all(abs(r[2])<1.5 for r in steady)
        if a.fault=='velocity_dropout':checks['dropout_no_drive']=bool(post) and all(r[7]<=1e-6 for r in post)
        if a.fault in ['pose_dropout','empty_path','nan_path']:checks['invalid_input_zero_target']=bool(post) and all(abs(r[5])<1e-6 and abs(r[6])<1e-6 for r in post)
        if a.fault=='emergency':checks['emergency_zero_command']=bool(post) and all(abs(r[5])+abs(r[6])+abs(r[7])<1e-6 for r in post)
        summary={'pass':all(checks.values()),'checks':checks,'counts':counts,'states':sorted(states),'samples':len(samples),'fault_at':fault_at,'max_actual_speed':max((r[4] for r in samples),default=None),'max_abs_y':max((abs(r[2]) for r in samples),default=None),'max_cross_track_distance':max((r[11] for r in steady if r[11] is not None),default=None),'max_path_distance':max((r[10] for r in steady if r[10] is not None),default=None),'max_angle':max((abs(r[6]) for r in samples),default=None),'coverage':'有限时长片段，不表示全圈完赛或实车安全；未覆盖YOLO与真实CAN'}
        (out/'validation'/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print(json.dumps(summary,ensure_ascii=False));return 0 if summary['pass'] else 1
    except Exception as e:
        (out/'validation'/'failure.json').write_text(json.dumps({'error':str(e)},ensure_ascii=False));raise
    finally:
        for _,p in reversed(procs + observers):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGCONT);os.killpg(p.pid,signal.SIGINT)
        for _,p in procs + observers:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':
    if not os.environ.get('FSD_LOG_OWNER'):
        import sys
        environment = dict(os.environ, FSD_RUN_MODE='simulation')
        os.execve('/usr/bin/python3', ['/usr/bin/python3', str(ROOT/'tools/run_logged.py'),
                   '/usr/bin/python3', str(Path(__file__).resolve()), *sys.argv[1:]], environment)
    raise SystemExit(main())
