from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ai_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Add config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poommipat',
    maintainer_email='poommipat_wat@cmu.ac.th',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_odom = ai_robot.ros_odom:main',
            'ros_teleop = ai_robot.ros_teleop:main',
            'ros_scan = ai_robot.ros_scan:main',
            'ros_ocgm = ai_robot.ros_ocgm:main',
            'ros_joy = ai_robot.ros_joy:main',
            'ros_getmap = ai_robot.ros_getmap:main',
            'ros_rrtstar = ai_robot.ros_rrtstar:main',
            'ros_getmap2 = ai_robot.ros_getmap2:main',
        ],
    },
)