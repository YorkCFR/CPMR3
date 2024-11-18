import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cpmr_ch11'

setup(
    name=package_name,
    version='0.3.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='CPMR 3rd Edition Chapter 11',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_chair = cpmr_ch11.follow_chair:main',
            'leader_chair = cpmr_ch11.leader_chair:main',
        ],
    },
)
