import os
from glob import glob
from setuptools import setup

package_name = 'cpmr_ch7'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('gazebo/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Code associated with Chapter 7 of CPMR 3rd edition',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'build_map = cpmr_ch7.build_map:main',
        ],
    },
)
