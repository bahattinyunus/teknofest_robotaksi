from setuptools import setup
import os
from glob import glob

package_name = 'robotaksi_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bahattin Yunus Cetin',
    maintainer_email='bahattinyunus@example.com',
    description='Perception layer for Teknofest Robotaksi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = robotaksi_perception.obstacle_detector:main',
        ],
    },
)
