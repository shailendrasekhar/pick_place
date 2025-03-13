from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manipulator'

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
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*')),  
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'models/panda'), glob('models/manipulator/*')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools', 'rclpy', 'gazebo_ros','rclcpp','tf2_geometry_msgs','tf2_eigen','geometry_msgs','moveit_core','moveit_ros_planning_interface','moveit_task_constructor_core','moveit_task_constructor_msgs'],
    zip_safe=True,
    maintainer='humble',
    maintainer_email='shailendrasekhar@ihub-data.iiit.ac.in',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "roboticstoolbox=manipulator.rtb:main",
            "moveit2_demo=manipulator.moveit2_demo:main",
            "sync_camera=manipulator.sync_camera:main",
            "planning_demo=manipulator.planning_demo:main"
        ],
    },
)
