from setuptools import find_packages, setup

package_name = 'ardupilot_bridge'

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
    maintainer='ceasarmusk',
    maintainer_email='ceasarmusk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_to_obstacle_distance = ardupilot_bridge.scan_to_obstacle_distance:main',
            'cmd_vel_to_guided = ardupilot_bridge.cmd_vel_to_guided:main',
            'odom_to_mavlink = ardupilot_bridge.odom_to_mavlink:main',
        ],
    },
)
