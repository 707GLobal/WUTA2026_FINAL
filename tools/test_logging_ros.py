"""Run only in an isolated ROS domain; no hardware interfaces are started."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'WUTA-FSD/ros2_ws/test'))
from test_cone_colors_ros import ros
from geometry_msgs.msg import PoseStamped
from wuta_msgs.msg import MissionState


def test_planner_switch_keeps_previous_csv_and_new_segments(ros, tmp_path):
    run = tmp_path/'run'
    ros.start('path_generator', 'path_generator_node', {
        'record.run_dir': str(run), 'record.flush_interval_sec': .1,
    }, {'/localization/pose': 'pose', '/system/mission_state': 'state'})
    pose = ros.publisher(PoseStamped, 'pose')
    state = ros.publisher(MissionState, 'state')
    ros.wait(lambda: pose.get_subscription_count() and state.get_subscription_count())
    def send_pose(x):
        msg = PoseStamped()
        msg.header.stamp = ros.stamp()
        msg.pose.position.x = x
        msg.pose.orientation.w = 1.
        pose.publish(msg)
    original = run/'jrajectory/driven.csv'
    ros.wait(lambda: original.exists() and len(original.read_text().splitlines()) > 1,
             lambda: send_pose(1.))
    mode = MissionState()
    mode.state = MissionState.IDLE
    mode.mission_mode = MissionState.MISSION_ACCELERATION
    state.publish(mode)
    ros.spin(.15)
    original_content = original.read_text()
    ros.wait(lambda: list((run/'jrajectory/segments/acceleration').glob('*/driven.csv')),
             lambda: send_pose(2.))
    ros.spin(.2)
    assert original.read_text() == original_content
    segments = list((run/'jrajectory/segments/acceleration').glob('*/driven.csv'))
    assert any('2.000000' in p.read_text() for p in segments)
    # Returning to a previously recorded mission must not reopen/truncate root CSV.
    mode.mission_mode = MissionState.MISSION_TRACKDRIVE
    state.publish(mode)
    ros.spin(.15)
    ros.wait(lambda: list((run/'jrajectory/segments/trackdrive').glob('*/driven.csv')),
             lambda: send_pose(3.))
    assert original.read_text() == original_content


def test_restarted_planner_preserves_existing_run_files(ros, tmp_path):
    run = tmp_path/'run'
    folder = run/'jrajectory'
    folder.mkdir(parents=True)
    original = folder/'driven.csv'
    original.write_text('existing-payload\n')
    (folder/'planned.csv').write_text('existing-planned\n')
    ros.start('path_generator', 'path_generator_node', {'record.run_dir': str(run)},
              {'/localization/pose': 'pose'})
    pose = ros.publisher(PoseStamped, 'pose')
    ros.wait(lambda: pose.get_subscription_count())
    def send():
        msg = PoseStamped()
        msg.header.stamp = ros.stamp()
        msg.pose.orientation.w = 1.
        pose.publish(msg)
    ros.wait(lambda: list((folder/'segments/trackdrive').glob('*/driven.csv')), send)
    assert original.read_text() == 'existing-payload\n'
    assert (folder/'planned.csv').read_text() == 'existing-planned\n'
