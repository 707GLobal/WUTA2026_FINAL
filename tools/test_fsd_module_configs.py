"""无硬件配置回归：模块文件变更、管线一致性与launch参数来源。"""
import importlib.util,json
from pathlib import Path
from unittest.mock import patch
import yaml,pytest
from launch import LaunchContext
from check_fsd_module_configs import ROOT,SRC,manifest,parameters
MODULE=SRC/'perception/detection_fusion/launch/fusion_modules_hardware.launch.py'
def load_launch():
    spec=importlib.util.spec_from_file_location('module_hardware',MODULE);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def test_control_and_actuator_parameters_have_separate_module_sources():
    m=manifest()
    control_path=SRC/'control/controller/config/controller.yaml'
    can_path=SRC/'system/can_interface/config/can_interface.yaml'
    assert m['files']['controller']==str(control_path.resolve())
    assert m['files']['can']==str(can_path.resolve())
    assert not control_path.is_symlink() and not can_path.is_symlink()
    control=m['parameters']['controller'];can=m['parameters']['can']
    assert 'steer' not in control and 'longitudinal' not in control
    assert 'command_timeout_sec' not in control['control']
    assert 'speed_control' not in can and 'speed_control' not in control
    assert 'control' not in can and 'longitudinal' not in can
    assert 'calibration' not in can['steer']
    assert control['vehicle']['max_steer_angle']==can['steer']['max_steer_angle']

def test_module_edits_are_visible_and_mismatch_rejected(tmp_path):
    base=parameters(SRC/'control/controller/config/controller.yaml');base['acceleration']['pid']['kp']=.7
    f=tmp_path/'controller.yaml';f.write_text(yaml.safe_dump({'controller_node':{'ros__parameters':base}}))
    assert manifest(controller=f)['parameters']['controller']['acceleration']['pid']['kp']==.7
    base['vehicle']['max_steer_angle']=27.;f.write_text(yaml.safe_dump({'controller_node':{'ros__parameters':base}}))
    with pytest.raises(ValueError,match='最大转角'):manifest(controller=f)

def test_hardware_launch_loads_source_configs_without_tuning_overrides():
    m=load_launch();context=LaunchContext();context.launch_configurations.update(config_root=str(SRC),model_path='',calibration_path=str(ROOT/'config/camera_lidar.yaml'),detector_backend='auto',adapter_backend='cpp',fusion_backend='cpp',start_drivers='false',zed_publish_tf='false',zed_publish_map_tf='false')
    nodes=[]
    def node(**kw):nodes.append(kw);return kw
    with patch.object(m,'Node',node),patch.object(m,'get_package_share_directory',return_value='/tmp/wuta_unused_driver'):
        m.setup(context)
    for package in ['lidar_detection','detection_fusion']:
        n=next(n for n in nodes if n['package']==package)
        assert len(n['parameters'])==1 and n['parameters'][0].startswith(str(SRC))
    c=next(n for n in nodes if n['executable']=='yolov8_node')
    assert set(c['parameters'][1])=={'model_path'}
    assert c['parameters'][1]['model_path']==str((ROOT/'models/yolov8sp2-int8.engine').resolve())
    assert parameters(c['parameters'][0])['model_input_width']==1280

def test_pipeline_and_snapshot_defaults():
    m=manifest()
    assert m['parameters']['fusion']['fuse_positions'] is False
    assert m['parameters']['fusion']['min_color_probability']==.6
    assert m['parameters']['lidar']['use_ransac'] is True
    assert m['parameters']['lidar']['voxel_leaf_size']==.08
    assert not m['explicit_overrides']['model'] and not m['explicit_overrides']['can_device']

def test_new_launcher_does_not_override_edited_perception_values(tmp_path):
    import shutil
    config_root=tmp_path/'src'
    for relative in ['perception/camera_detection/config/camera_detection.yaml','perception/camera_detection/config/stereo_adapter.yaml','perception/lidar_detection/config/lidar_detection.yaml','perception/detection_fusion/config/fusion.yaml','perception/detection_fusion/config/rsm1_hardware.yaml','perception/detection_fusion/config/zed_hardware.yaml']:
        dst=config_root/relative;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(SRC/relative,dst)
    lidar=config_root/'perception/lidar_detection/config/lidar_detection.yaml';doc=yaml.safe_load(lidar.read_text());doc['lidar_detection_node']['ros__parameters']['voxel_leaf_size']=.12;lidar.write_text(yaml.safe_dump(doc))
    fusion=config_root/'perception/detection_fusion/config/fusion.yaml';doc=yaml.safe_load(fusion.read_text());doc['detection_fusion_node']['ros__parameters']['min_color_probability']=.73;fusion.write_text(yaml.safe_dump(doc))
    m=load_launch();context=LaunchContext();context.launch_configurations.update(config_root=str(config_root),model_path=str(ROOT/'models/yolov8sp2-int8.engine'),calibration_path=str(ROOT/'config/camera_lidar.yaml'),detector_backend='auto',adapter_backend='cpp',fusion_backend='cpp',start_drivers='false',zed_publish_tf='false',zed_publish_map_tf='false')
    nodes=[]
    with patch.object(m,'Node',lambda **kw:nodes.append(kw)),patch.object(m,'get_package_share_directory',return_value='/tmp/wuta_unused_driver'):
        m.setup(context)
    for pkg,key,wanted in [('lidar_detection','voxel_leaf_size',.12),('detection_fusion','min_color_probability',.73)]:
        n=next(x for x in nodes if x['package']==pkg)
        assert len(n['parameters'])==1
        assert parameters(n['parameters'][0])[key]==wanted


def test_independent_throttle_and_brake_limits(tmp_path):
    base=parameters(SRC/'control/controller/config/controller.yaml')
    base['acceleration']['throttle']['max_opening']=.2
    base['acceleration']['brake']['max_opening']=.8
    f=tmp_path/'controller.yaml';f.write_text(yaml.safe_dump({'controller_node':{'ros__parameters':base}}))
    config=manifest(controller=f)['parameters']['controller']
    assert config['acceleration']['throttle']['max_opening']==.2
    assert config['acceleration']['brake']['max_opening']==.8
    assert config['trackdrive']['throttle']['max_opening']==1.0

@pytest.mark.parametrize('opening',[-.1,1.1,float('nan'),float('inf')])
def test_invalid_openings_rejected_before_launch(tmp_path,opening):
    base=parameters(SRC/'control/controller/config/controller.yaml')
    base['ebs']['brake']['max_opening']=opening
    f=tmp_path/'controller.yaml';f.write_text(yaml.safe_dump({'controller_node':{'ros__parameters':base}}))
    with pytest.raises(ValueError,match='ebs.brake.max_opening'):
        manifest(controller=f)

def test_legacy_pid_output_limit_is_not_silently_ignored(tmp_path):
    base=parameters(SRC/'control/controller/config/controller.yaml')
    base['skidpad']['pid']['output_limit']=.2
    f=tmp_path/'controller.yaml';f.write_text(yaml.safe_dump({'controller_node':{'ros__parameters':base}}))
    with pytest.raises(ValueError,match='skidpad.pid.output_limit'):
        manifest(controller=f)
