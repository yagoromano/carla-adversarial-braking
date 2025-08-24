from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carla_autobreak_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='papatel46',
    maintainer_email='papatel46@tntech.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            
            
            'lidar = carla_autobreak_test.lidar:main',
            'lidar_control = carla_autobreak_test.lidar_control:main',
            'lidar_topic_attack = carla_autobreak_test.lidar_topic_attack:main'
            
                            
        ],
    },
)
# 'yolo = carla_autobreak_test.yolo:main',
# 'vehicle_control = carla_autobreak_test.vehicle_control:main',
# 'lidar = carla_autobreak_test.lidar:main',
# 'lidar_fusion = carla_autobreak_test.lidar_fusion:main'
# 'lidar_control = carla_autobreak_test.lidar_control:main'
# 'depth = carla_autobreak_test.depth:main'
# 'weatherTest = carla_autobreak_test.weatherTest:main',
# 'setSpawn = carla_autobreak_test.setSpawn:main',
