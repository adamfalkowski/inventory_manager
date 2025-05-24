from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # For data file there are "non python source file" aka no .py file; Yet they still need to be installed when installing your package 
    # (destination , source)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', ['launch/color_detection.launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam-falkowski',
    maintainer_email='adamfalkowski2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera.camera_publisher:main',
            'color_detection = camera.color_detection:main'
        ],
    },
)
