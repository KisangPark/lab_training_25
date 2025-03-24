from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'week3'

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
            'save_costmap = week3.save_costmap:main',
            'rrt_star = week3.RRT_star:main',
            'rrt_star_2 = week3.RRT_star_2:main',
            'pure_pursuit = week3.pure_pursuit:main',
            'rrt_debug = week3.rrt_debug:main',
        ],
    },
)
