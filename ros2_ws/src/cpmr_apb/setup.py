import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'cpmr_apb'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='CPMR3rd Edition Appendix B',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'populate_world = cpmr_apb.populate:main',
            'depopulate_world = cpmr_apb.depopulate:main',
        ],
    },
)
