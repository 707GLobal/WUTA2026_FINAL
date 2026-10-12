"""无硬件检查：模块配置来源、管线话题和共享车辆参数，输出可审查JSON。"""
from pathlib import Path
import argparse,json,math,yaml
ROOT=Path(__file__).resolve().parents[1];SRC=ROOT/'WUTA-FSD/ros2_ws/src'
def parameters(p):
    document=yaml.safe_load(Path(p).read_text())
    return next(v['ros__parameters'] for v in document.values() if isinstance(v,dict) and 'ros__parameters' in v)
def manifest(controller=None,can=None,model='',can_device=''):
    files={'controller':Path(controller or SRC/'control/controller/config/controller.yaml'),
      'can':Path(can or SRC/'system/can_interface/config/can_interface.yaml'),
      'camera':SRC/'perception/camera_detection/config/camera_detection.yaml',
      'adapter':SRC/'perception/camera_detection/config/stereo_adapter.yaml',
      'lidar':SRC/'perception/lidar_detection/config/lidar_detection.yaml',
      'fusion':SRC/'perception/detection_fusion/config/fusion.yaml',
      'mapping':SRC/'mapping/cone_map_builder/config/cone_map_builder.yaml',
      'boundary':SRC/'planning/boundary_detector/config/boundary_detector.yaml',
      'planning':SRC/'planning/path_generator/config/path_generator.yaml',
      'ekf':SRC/'localization/localization_manager/config/ekf.yaml',
      'mission':SRC/'system/mission_manager/config/mission_manager.yaml'}
    v={k:parameters(p) for k,p in files.items()}
    def need(ok,message):
        if not ok:raise ValueError(message)
    for mission in ('acceleration','ebs','skidpad','trackdrive'):
        config=v['controller'][mission]
        need('output_limit' not in config['pid'],mission+'.pid.output_limit已移除，请将原值迁移到throttle.max_opening和brake.max_opening')
        for actuator in ('throttle','brake'):
            opening=config.get(actuator,{}).get('max_opening',1.0)
            need(isinstance(opening,(int,float)) and not isinstance(opening,bool) and math.isfinite(opening) and 0<=opening<=1,mission+'.'+actuator+'.max_opening必须为[0,1]内的有限值')
    # CAN回退到54a6038后只读取steer.max_steer_angle；核对实际被节点消费的参数。
    need(v['controller']['vehicle']['max_steer_angle']==v['can']['steer']['max_steer_angle'],'控制器与CAN最大转角必须相同')
    for a,b,msg in [(v['camera']['output_topic'],v['adapter']['boxes_topic'],'图像检测→立体'),(v['adapter']['output_topic'],v['fusion']['camera_topic'],'立体→融合'),(v['adapter']['output_info_topic'],v['fusion']['camera_info_topic'],'CameraInfo→融合'),(v['lidar']['output_topic'],v['fusion']['lidar_topic'],'雷达检测→融合'),(v['lidar']['input_topic'],v['fusion']['pointcloud_topic'],'点云输入')]:need(a==b,msg+'话题不一致')
    driver=yaml.safe_load((SRC/'perception/detection_fusion/config/rsm1_hardware.yaml').read_text())['lidar'][0]['ros']
    need(driver['ros_send_point_cloud_topic']==v['lidar']['input_topic'],'M1驱动与雷达检测输入话题不一致')
    need(v['mission']['lidar_topic']==v['lidar']['input_topic'],'任务自检雷达话题不一致')
    need(v['mission']['camera_topic']==v['camera']['image_topic'],'任务自检相机话题不一致')
    need(not v['mission']['use_ndt_race_localization'],'当前M1全栈不启动NDT，不能启用NDT比赛定位')
    def finite(x):
        if isinstance(x,dict):
            for value in x.values():finite(value)
        elif isinstance(x,list):
            for value in x:finite(value)
        elif isinstance(x,float):need(math.isfinite(x),'模块参数包含非有限数值')
    finite(v)
    selected=Path(model or v['camera']['model_path']).expanduser()
    if not selected.is_absolute():selected=(files['camera'].parent/selected).resolve()
    return {'files':{k:str(p.resolve()) for k,p in files.items()},'parameters':v,
      'deployment':{'model_path':str(selected),'can_device':can_device or v['can']['can']['device'],
        'lidar_topic':v['lidar']['input_topic'],'image_topic':v['camera']['image_topic'],
        'fusion_topic':v['fusion']['output_topic'],'ins_topic':v['mission']['imu_topic']},
      'explicit_overrides':{'model':bool(model),'can_device':bool(can_device)},
      'policy':'模块源码config唯一来源；旧统一配置不加载；显式部署选项除外'}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--controller');p.add_argument('--can');p.add_argument('--model',default='');p.add_argument('--can-device',default='');a=p.parse_args()
    print(json.dumps(manifest(a.controller,a.can,a.model,a.can_device),ensure_ascii=False,indent=2))
