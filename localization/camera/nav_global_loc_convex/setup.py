from setuptools import find_packages, setup

package_name = 'nav_global_loc_convex'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ros2_aruco_interfaces', 'rclpy', 'geometry_msgs'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno.blan334@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'range_bearing_localizer = nav_global_loc_convex.solve_problem:main',
        ],
    },
)
