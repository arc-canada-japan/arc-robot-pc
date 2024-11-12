from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'communication_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Include all misc files.
        (os.path.join('share', package_name, 'misc'), glob(os.path.join('misc', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshidalab',
    maintainer_email='contact@yoshidalab.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_to_zmq_bridge = communication_interface.ros_to_zmq_bridge:main',
            'end_effector_fake_publisher = communication_interface.end_effector_fake_publisher:main',
        ],
    },
)
