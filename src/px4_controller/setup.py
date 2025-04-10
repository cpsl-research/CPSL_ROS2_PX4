from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.hunt@duke.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_control_node = px4_controller.px4_control_node:main',
            'px4_keyop_node = px4_controller.px4_keyop_node:main',
            'offboard_control_example=px4_controller.offboard_control_example:main',
            'px4_joy_node=px4_controller.px4_joy_node:main']
    },
)
