import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lio_localization_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')
            + glob('config/*.config')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yukichi6105',
    maintainer_email='107849799+YUKICHI6105@users.noreply.github.com',
    description='main.md 6-A-1 軽量2D合成データシミュレータ',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_sim_node = lio_localization_sim.imu_sim_node:main',
            'lidar_sim_node = lio_localization_sim.lidar_sim_node:main',
            'evaluator_node = lio_localization_sim.evaluator_node:main',
            'gazebo_driver_node = lio_localization_sim.gazebo_driver_node:main',
        ],
    },
)
