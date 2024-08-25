import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rimelq',
    maintainer_email='rim.elqabli@epfl.ch',
    # keywords=['foo', 'bar'],
    # classifiers=[
    #     'Intended Audience :: Developers',
    #     'License :: TODO',
    #     'Programming Language :: Python',
    #     'Topic :: Software Development',
    # ],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = src.publish_sensor:main',
            'av_imu_node = src.av_imu_node:main',
            'old_imu_node = src.old_publish_sensor:main',


        ],
    },
)
