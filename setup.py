from glob import glob

from setuptools import find_packages, setup

package_name = 'csen934_project_pkg'

NODES = {
    # milestone nodes (also installed as "<name>.py", the names used in Project.pdf)
    'control': 'control',
    'localization_add_noise': 'localization_add_noise',
    'localization_ekf_node': 'localization_ekf_node',
    # driving
    'teleop': 'teleop_gui',
    'teleop_cli': 'teleop',
    'autopilot': 'autopilot',
    # visualization / SLAM helpers
    'odom_to_pose2d': 'odom_to_pose2d',
    'odom_tf_broadcaster': 'odom_tf_broadcaster',
    'trajectory_publisher': 'trajectory_publisher',
    'lio_sam_aligner': 'lio_sam_aligner',
    'save_map': 'save_map',
    'map_compare': 'map_compare',
}
ALIASES = ['control', 'localization_add_noise', 'localization_ekf_node']

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssef_elfeky',
    maintainer_email='youssef_elfeky@todo.todo',
    description='CSEN934 project: Webots car, Ackermann control, noisy odometry, EKF, SLAM',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts':
            [f'{name} = {package_name}.{module}:main' for name, module in NODES.items()]
            + [f'{name}.py = {package_name}.{name}:main' for name in ALIASES],
    },
)
