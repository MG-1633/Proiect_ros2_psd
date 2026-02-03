from setuptools import setup

package_name = 'color_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigate_box.launch.py', 'launch/Maps_box.launch.py']),
        ('share/' + package_name + '/models/colored_box', ['models/colored_box/model.config', 'models/colored_box/model.sdf']),
        ('share/' + package_name + '/models/camera_robot', ['models/camera_robot/model.config', 'models/camera_robot/model.sdf']),
        ('share/' + package_name + '/worlds', ['worlds/box_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for navigating and detecting colors in a Gazebo simulation.',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = color_navigator.color_detector_node:main',
            'robot_controller = color_navigator.robot_controller_node:main',
        ],
    },
)