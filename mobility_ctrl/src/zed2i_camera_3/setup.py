from setuptools import find_packages, setup

package_name = 'zed2i_camera_3'

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
    maintainer='ros2_user',
    maintainer_email='ros2_user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["zed2i_camera_3_node = zed2i_camera_3.zed2i_camera_3_node:main"
        ],
    },
)
