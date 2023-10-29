from setuptools import setup
from glob import glob
import os

package_name = 'obstacle_filter'
submodule_name = 'obstacle_filter/filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.[pxy][yma]*')),
    ],
    #py_modules=['obstacle_filter_node'],
    install_requires=['setuptools', 'numpy', 'torch', 'matplotlib'],
    zip_safe=True,
    maintainer='anoca',
    maintainer_email='aurelio.noca@epfl.ch',
    description='obstacle filter node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_filter_node = obstacle_filter.obstacle_filter_node:main',
            'slope_filter_node = obstacle_filter.slope_filter_node:main'
        ],
    },
)
