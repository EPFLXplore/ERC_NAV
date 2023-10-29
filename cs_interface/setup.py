from setuptools import setup
import os
from glob import glob

package_name = 'cs_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']
        ),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xplore',
    maintainer_email='aurelio.noca@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cs_interface_node = cs_interface.cs_interface_node:main',
            'cs_interface_send_goal = cs_interface.send_goal:main',
            'cs_interface_receive_goal = cs_interface.receive_goal:main'

        ],
    },
)



