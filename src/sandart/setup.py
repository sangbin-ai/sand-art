from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sandart'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'sandart', 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sb',
    maintainer_email='dltkdqls120o@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_plan_node = sandart.path_plan_node:main',
            'skeleton_processor_node = sandart.skeleton_processor_node:main',
            'lifecycle_manage_node = sandart.lifecycle_manage_node:main',
            'sandart_movesx_node = sandart.sandart_movesx_node:main',
            'sand_clear_node = sandart.sand_clear_node:main',
        ],
    },
)
