from setuptools import setup
from glob import glob
import os

package_name = 'cyber_is_led_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Control module for LEDs on all robot sides (ROS 2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leds_node = cyber_is_led_controller.leds_node:main',
        ],
    },
)
