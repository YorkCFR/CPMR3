from glob import glob
import os
from setuptools import setup

package_name = 'cpmr_ch6'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Code associated with Chapter 6 of CPMR 3rd Edition',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_by_line = cpmr_ch6.drive_by_line:main',
            'auto_drive_by_line = cpmr_ch6.auto_drive_by_line:main',
            'drive_by_road = cpmr_ch6.drive_by_road:main',
            'auto_drive_by_road = cpmr_ch6.auto_drive_by_road:main',
        ],
    },
)
