from setuptools import find_packages, setup
import os
from glob import glob 


package_name = 'glim_starter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'glim_config'), glob('glim_config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xplore',
    maintainer_email='arno.laurie@epfl.ch',
    description='launch file for glim, decouples GLIM TFs from the EKF to avoid conflicts',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'glim_odom_publisher = glim_starter.glim_odom_publisher:main',
        ],
    },
)
