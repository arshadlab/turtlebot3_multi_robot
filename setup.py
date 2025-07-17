import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'tb3_multi_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='.', include=['multi_robot_scripts', 'multi_robot_scripts.*']),
    package_dir={'': '.'},
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch/nav2_bringup'), glob('launch/nav2_bringup/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger'), glob('models/turtlebot3_burger/*')),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle'), glob('models/turtlebot3_waffle/*')),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_pi'), glob('models/turtlebot3_waffle_pi/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arshad Mehmood',
    maintainer_email='arshadm78@yahoo.com',
    description='Multi-robot simulation using TurtleBot3 and Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_drive = multi_robot_scripts.turtlebot3_drive:main',
        ],
    },
)
