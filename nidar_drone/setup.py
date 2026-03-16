import os
from glob import glob
from setuptools import setup

package_name = 'nidar_drone'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),       # this line picks up all launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='NIDAR autonomous drone package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hello_node = nidar_drone.hello_node:main',
            'altitude_publisher = nidar_drone.altitude_publisher:main',
            'altitude_subscriber = nidar_drone.altitude_subscriber:main',
            'arm_service = nidar_drone.arm_service:main',
            'arm_client = nidar_drone.arm_client:main',
        ],
    },
)