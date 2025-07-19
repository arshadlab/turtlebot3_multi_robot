import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'turtlebot3_multi_robot'



setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=['multi_robot_scripts', 'multi_robot_scripts.*']),
    package_dir={'': '.'},
    data_files=[
        (f'share/{package_name}', ['package.xml']),  
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'nav2_bringup'), glob('launch/nav2_bringup/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger'), glob('models/turtlebot3_burger/*') ),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    zip_safe=True,
    maintainer='Arshad Mehmood',
    maintainer_email='arshadm78@yahoo.com.com',
    description='Multi-robot simulation using TurtleBot3 and Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)