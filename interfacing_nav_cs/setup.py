from setuptools import find_packages, setup

package_name = 'interfacing_nav_cs'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xplore',
    maintainer_email='se-software@epfl-xplore.ch',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface = interfacing_nav_cs.NavCSInterface:main',
        ],
    },
)
