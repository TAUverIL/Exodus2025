from setuptools import setup

package_name = 'multi_zed_rtab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/multi_zed_rtab']),
        ('share/multi_zed_rtab', ['package.xml']),
        ('share/multi_zed_rtab/launch', ['launch/multi_zed_rtab.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Python-based ROS 2 package for multiple ZED cameras and RTAB-Map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
