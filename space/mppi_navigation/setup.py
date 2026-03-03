from setuptools import setup
import os
from glob import glob

package_name = 'mppi_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='MPPI-based navigation for Mars rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_to_costmap = mppi_navigation.pointcloud_to_costmap:main',
            'mppi_controller = mppi_navigation.mppi_controller:main',
            'goal_publisher = mppi_navigation.goal_publisher:main',
        ],
    },
)