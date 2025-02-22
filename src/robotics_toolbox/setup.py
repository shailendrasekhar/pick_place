from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotics_toolbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools', 'rclpy', 'gazebo_ros'],
    zip_safe=True,
    maintainer='humble',
    maintainer_email='shailendra.sekhar@ihub-data.iiit.ac.in',
    description='Inverse and Forward Kinematics with RTB',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "roboticstoolbox=master_panda.rtb:main"
        ],
    },
)
