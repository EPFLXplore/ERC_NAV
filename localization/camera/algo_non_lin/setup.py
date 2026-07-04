from setuptools import find_packages, setup

package_name = 'algo_non_lin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['package.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ros2_aruco_interfaces', 'rclpy', 'geometry_msgs', 'nav_msgs'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno.blan334@gmail.com',
    description='Nonlinear refinement localization (convex seed + least-squares)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'nonlinear_localizer = algo_non_lin.nonlinear_localizer:main',
        ],
    },
)
