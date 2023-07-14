import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'urdf_tutorial'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include model and simulation files
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Bongalon',
    maintainer_email='ben.bongalon@gmail.com',
    description='ROS 2 tutorial: Using URDF with robot_state_publisher',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = urdf_tutorial_r2d2.state_publisher:main'
        ],
    },
)
