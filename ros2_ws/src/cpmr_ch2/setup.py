from glob import glob
import os
from setuptools import setup

package_name = 'cpmr_ch2'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('maps/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Code associated with Chapter 2 of CPMR 3rd Edition',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_robot = cpmr_ch2.drive_robot:main',
            'add_obstacle = cpmr_ch2.add_obstacle:main',
            'build_map = cpmr_ch2.build_map:main',
            'destroy_map = cpmr_ch2.destroy_map:main',
        ],
    },
)
