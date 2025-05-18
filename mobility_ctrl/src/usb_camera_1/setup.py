from setuptools import find_packages, setup

package_name = 'usb_camera_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tauver',
    maintainer_email='tauver@todo.todo',
    description='Streams video from a USB camera as ROS2 image messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_1_node = usb_camera_1.usb_camera_1_node:main'
        ],
    },
)
