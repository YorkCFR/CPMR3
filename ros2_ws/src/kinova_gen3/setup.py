from setuptools import setup

package_name = 'kinova_gen3'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description="Shallow interface to Kinova's gen3 arm",
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinova_gen3 = kinova_gen3.gen3_lite:main',
            'arm_test = kinova_gen3.arm_test:main',
        ],
    },
)
