from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ardupilot_gz_gazebo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ArduPilot Team',
    maintainer_email='dev@ardupilot.org',
    description='Gazebo worlds and configurations for ArduPilot integration',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)