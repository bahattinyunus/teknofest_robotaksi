from setuptools import setup

package_name = 'robotaksi_control'

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
    description='Control layer for Teknofest Robotaksi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = robotaksi_control.pid_controller:main',
        ],
    },
)
