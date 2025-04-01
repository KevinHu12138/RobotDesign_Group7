from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Add behaviour tree xml files
        (os.path.join('share', package_name, 'behavior_tree_xml'), glob(os.path.join('behavior_tree_xml', '*.xml'))),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*yaml*'))),
        # Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*yaml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy West',
    maintainer_email='andrew.west@manchester.ac.uk',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_map_saver = navigation_demos.custom_map_saver_node:main',
            'explore = navigation_demos.explore:main',
            'simple_nav = navigation_demos.simple_nav:main',
            'nav2_nav_test = navigation_demos.nav2_nav_test:main',
            'robot_navigator = navigation_demos.robot_navigator:main',
            'patrol_service_node = navigation_demos.patrol_service_node:main',
            'patroling_node = navigation_demos.patroling_node:main',
            'robot_state_machine = navigation_demos.robot_state_machine:main',
            'main = nav_gui.main:main',
            'drive_to_apriltag = navigation_demos.drive_to_apriltag:main',
            'manipulator_test = navigation_demos.manipulator_test:main',
        ],
    },
)
