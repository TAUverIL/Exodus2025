from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include urdf files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro.urdf')),
        # Include rviz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Include node files
        (os.path.join('share', package_name, 'rover_hw'), glob('rover_hw/*')),
        # Include map files
        (os.path.join('share', package_name, 'rover_hw'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brittc',
    maintainer_email='brittanykcohen96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offset_cmd = rover_hw.offset_cmd:main',
        ],
    },
)
