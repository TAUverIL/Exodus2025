from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cubemars_test'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # Include world files
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        # Include rviz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Include map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tauver',
    maintainer_email='tauver@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
