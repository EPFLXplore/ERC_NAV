from setuptools import setup
import os  
from glob import glob


package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arno Laurie',
    maintainer_email='arno.laurie@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = ros2_aruco.aruco_node:main',
            'multiview_aruco_node = ros2_aruco.multi_aruco_node:main',
            'pose_estimation_node = ros2_aruco.pose_estimation_node:main',
            'pot_arucos_node = ros2_aruco.plot_arucos:main'
        ],
    },
)
