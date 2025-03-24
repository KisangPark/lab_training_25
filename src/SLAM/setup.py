from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'SLAM'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install xacro folder
        (os.path.join('share', package_name, 'xacro'), glob('xacro/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kisangpark',
    maintainer_email='kisangtree@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_generator = SLAM.path_generator:main',
            'path_follower = SLAM.path_follower:main',
        ],
    },
)
