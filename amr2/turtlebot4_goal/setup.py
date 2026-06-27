from setuptools import find_packages, setup

package_name = 'turtlebot4_goal'

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
    maintainer='sb',
    maintainer_email='dltkdqls120o@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pickup_beep = turtlebot4_goal.pickup_beep_node:main',
            'emergency_beep = turtlebot4_goal.emergency_beep_node:main',
            'battery_manager = turtlebot4_goal.battery_manager_node:main',
            'yolo_detect = turtlebot4_goal.yolo_detect_node:main'
        ],
    },
)
