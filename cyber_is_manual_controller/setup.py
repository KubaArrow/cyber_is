from setuptools import setup

package_name = 'cyber_is_manual_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_manual_mode.launch.py']),
        ('share/' + package_name + '/bash', ['bash/start_streamer.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kuba_Arrow',
    maintainer_email='jakubstrzalkaa@gmail.com',
    description='ROS 2 (Humble) manual controller: joystick teleop and optional MJPG streamer.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_teleop = cyber_is_manual_controller.joystick_node:main',
            'start_streamer = cyber_is_manual_controller.streamer:main',
        ],
    },
)

