from setuptools import setup
import os
from glob import glob

package_name = 'jmoab_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='JMOAB ROS2 interface node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'atcart_basic = jmoab_ros2.atcart_basic:main',
        'bno055 = jmoab_ros2.bno055:main',
        'atcart8 = jmoab_ros2.atcart8:main',
        'pwmcart = jmoab_ros2.pwmcart:main'
        ],
    },
)
