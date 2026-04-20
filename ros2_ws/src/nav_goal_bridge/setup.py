from setuptools import find_packages, setup

package_name = 'nav_goal_bridge'

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
    maintainer_email='you@example.com',
    description='Bridge Foxglove /move_base_simple/goal to Nav2 navigate_to_pose',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_base_simple_to_nav2 = nav_goal_bridge.move_base_simple_to_nav2:main',
        ],
    },
)
