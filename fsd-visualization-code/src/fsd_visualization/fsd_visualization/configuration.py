"""Generate a viewer configuration with independently configurable input topics."""
from copy import deepcopy
import math

DISPLAY_TOPICS = {
    'Camera detections': 'camera_image',
    'Camera-guided clusters (fusion)': 'guided_clusters',
    'Raw LiDAR (optional)': 'raw_lidar',
    'Fused targets and sources': 'fused_markers',
    'Cone map': 'map_markers',
    'Centerline': 'centerline_markers',
    'Final planned path': 'path_markers',
    'Driven trajectory': 'trajectory_markers',
    'Control target': 'control_target',
}
BRIDGE_TOPICS = {
    'Fused targets and sources': '/fsd_visualization/fused_cones',
    'Cone map': '/fsd_visualization/cone_map',
    'Centerline': '/fsd_visualization/centerline',
    'Final planned path': '/fsd_visualization/planned_path',
    'Driven trajectory': '/fsd_visualization/trajectory',
}


def validate(settings):
    if not settings.get('fixed_frame') or not settings.get('target_frame'):
        raise ValueError('fixed_frame and target_frame must be non-empty')
    for key in ('stale_seconds', 'trajectory_min_distance', 'trajectory_max_step'):
        value = float(settings[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if int(settings['trajectory_max_points']) < 2:
        raise ValueError('trajectory_max_points must be at least 2')
    if not 1 <= int(settings['frame_rate']) <= 120:
        raise ValueError('frame_rate must be in [1,120]')
    for key, value in settings['topics'].items():
        if not isinstance(value, str) or (value and not value.startswith('/')):
            raise ValueError(f'topics.{key} must be an absolute topic or empty')


def render(template, settings, mode):
    validate(settings)
    if mode not in ('native', 'bridge'):
        raise ValueError('mode must be native or bridge')
    output = deepcopy(template)
    manager = output['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = settings['fixed_frame']
    manager['Global Options']['Frame Rate'] = int(settings['frame_rate'])
    manager['Views']['Current']['Target Frame'] = settings['target_frame']
    for display in manager['Displays']:
        name = display['Name']
        if name in DISPLAY_TOPICS:
            topic = settings['topics'][DISPLAY_TOPICS[name]]
            if mode == 'bridge' and name in BRIDGE_TOPICS:
                topic = BRIDGE_TOPICS[name]
                display.pop('Namespaces', None)
            display['Topic']['Value'] = topic
            if not topic:
                display['Enabled'] = display['Value'] = False
            if mode == 'bridge' and name == 'Fused targets and sources':
                display['Name'] = 'Fused targets (semantic colors; sources unavailable)'
    return output
