from setuptools import setup

package_name = 'circle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='finn',
    maintainer_email='finnpicotoli@gmail.com',
    description='Circle detection node for drone payload delivery',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_detector = circle_detector.circle_detector:main',
        ],
    },
)
