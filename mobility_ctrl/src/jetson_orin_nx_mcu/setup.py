from setuptools import find_packages, setup

package_name = 'jetson_orin_nx_mcu'

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
        'console_scripts': ["jetson_orin_nx_mcu_node = jetson_orin_nx_mcu.jetson_orin_nx_mcu_node:main"
        ],
    },
)
