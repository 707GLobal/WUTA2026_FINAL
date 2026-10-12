from glob import glob
from setuptools import setup

setup(
    name='fsd_visualization', version='0.1.0', packages=['fsd_visualization'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fsd_visualization']),
        ('share/fsd_visualization', ['package.xml']),
        ('share/fsd_visualization/launch', glob('launch/*.launch.py')),
        ('share/fsd_visualization/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='WUTA', maintainer_email='wuta@example.invalid',
    description='Display-only FSD ROS visualization', license='Apache-2.0',
    entry_points={'console_scripts': ['display_bridge = fsd_visualization.bridge:main']},
)
