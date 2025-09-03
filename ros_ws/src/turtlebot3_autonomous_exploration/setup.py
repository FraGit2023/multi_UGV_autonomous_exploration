import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_autonomous_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_frame_broad = turtlebot3_autonomous_exploration.global_frame_broad:main',
            'global_map_merger_node = turtlebot3_autonomous_exploration.global_map_merger_node:main',
            'global_frontier_detector_node = turtlebot3_autonomous_exploration.global_frontier_detector_node:main',
            'greedy_task_allocator_node = turtlebot3_autonomous_exploration.greedy_task_allocator_node:main',
        ],
    },
)
